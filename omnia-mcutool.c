/*
 * This program interacts with Omnia MCU over I2C bus. It displays version
 * of the MCU firmware - both bootloader and application parts (which are
 * the git hashes compiled into the images) and flash application image
 * to the MCU EEPROM.
 *
 * Copyright (C) 2016, 2022, 2023 CZ.NIC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#define _GNU_SOURCE
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>
#include <endian.h>

#include "crc32.h"
#include "i2c_iface.h"
#include "timeutils.h"

#define MIN(a, b)				\
	({					\
		__auto_type ___a = (a);		\
		__auto_type ___b = (b);		\
		___a < ___b ? ___a : ___b;	\
	})

#define DEV_NAME	"/dev/i2c-1"
#define FLASH_SIZE	43008 /* flash size, 42k for MCU */

#define PKT_DATA_SZ	128 /* 128 data bytes in one packet when flashing */
#define WRITE_DELAY	20 /* default write delay in ms for old protocol */
#define READ_DELAY	0 /* default read delay in ms for old protocol */
#define RETRY		3
#define FILE_CMP_OK	0xBB /* bootloader flash protocol code: flash OK */
#define FILE_CMP_ERROR	0xDD /* bootloader flash protocol code: flash failed */
#define ADDR_CMP	0xFFFF /* bootloader flash protocol code addr */

#define VERSION_HASHLEN		20 /* 20 bytes of SHA-1 git hash */
#define BOOTLOADER_TRANS_DELAY	1 /* Bootloader transition delay */

typedef struct {
	bool bootloader;
	bool force;
	bool even_if_unlocked;

	/* old flashing protocol parameters */
	unsigned write_delay;
	unsigned read_delay;
} flash_opts_t;

static const char *argv0;

__attribute__((__format__(__printf__, 1, 2)))
static void error(const char *fmt, ...)
{
	va_list ap;

	fflush(stdout);
	fflush(stderr);

	while (*fmt == '\n') {
		fputc('\n', stderr);
		++fmt;
	}

	fprintf(stderr, "%s: ", argv0);

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);

	fputc('\n', stderr);
}

__attribute__((__noreturn__, __always_inline__, __format__(__printf__, 1, 2)))
static inline void die(const char *fmt, ...)
{
	error(fmt, __builtin_va_arg_pack());

	exit(EXIT_FAILURE);
}

__attribute__((__noreturn__, __always_inline__, __format__(__printf__, 1, 2)))
static inline void die_suggest_help(const char *fmt, ...)
{
	if (fmt)
		error(fmt, __builtin_va_arg_pack());

	die("try the --help option for usage information");
}

static int parse_uint_option(const char *opt, const char *arg, int max, int def)
{
	unsigned long val;
	char *end;

	if (!arg) {
		if (def < 0)
			die("missing argument for option '--%s'", opt);

		return def;
	}

	val = strtoul(arg, &end, 10);

	if (*arg == '\0' || *end != '\0')
		die("invalid argument '%s' for integer option '--%s'", arg,
		    opt);
	else if (max >= 0 && val > max)
		die("value %s of option '--%s' exceeds maximum value %i", arg,
		    opt, max);

	return val;
}

static void *xmalloc(size_t size)
{
	void *res = malloc(size);

	if (!res)
		die("out of memory");

	return res;
}

static void put_unaligned_le16(uint16_t val, void *dst)
{
	uint8_t *p = dst;

	*p++ = val;
	*p++ = val >> 8;
}

static void put_unaligned_le32(uint32_t val, void *dst)
{
	uint8_t *p = dst;

	*p++ = val;
	*p++ = val >> 8;
	*p++ = val >> 16;
	*p++ = val >> 24;
}

static uint32_t get_unaligned_le32(const void *src)
{
	const uint8_t *p = src;

	return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

static uint32_t crc32(uint32_t crc, const void *_data, uint32_t len)
{
	const uint8_t *data = _data;

	if (len & 3)
		die("length (%u) must be multiple of 4", len);

	for (uint32_t i = 0; i < len; i += 4, data += 4) {
		crc = crc32_one(crc, data[3]);
		crc = crc32_one(crc, data[2]);
		crc = crc32_one(crc, data[1]);
		crc = crc32_one(crc, data[0]);
	}

	return crc;
}

typedef enum {
	I2C_ADDR_MCU = 0x2a,
	I2C_ADDR_LED = 0x2b,
	I2C_ADDR_BOOT = 0x2c,
} i2c_addr_t;

static int open_i2c(i2c_addr_t addr)
{
	int fd;

	if ((fd = open(DEV_NAME, O_RDWR)) < 0)
		die("failed to open I2C bus %s@0x%02x: %m", DEV_NAME, addr);

	if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0)
		die("failed to acquire bus access and/or talk to slave %s@0x%02x: %m",
		    DEV_NAME, addr);

	return fd;
}

static void flash_old_send(const void *src, uint16_t offset, uint16_t size,
			   unsigned write_delay)
{
	struct timespec delay = {
		.tv_sec = write_delay / 1000,
		.tv_nsec = (write_delay % 1000) * 1000000,
	};
	uint16_t sent;
	int fd, i;

	fd = open_i2c(I2C_ADDR_BOOT);

	for (sent = 0, i = 1; sent < size; sent += PKT_DATA_SZ, ++i) {
		struct {
			uint16_t addr;
			char data[PKT_DATA_SZ];
		} pkt;
		uint16_t len, pktlen;

		pkt.addr = htobe16(offset + sent);

		len = MIN(size - sent, PKT_DATA_SZ);
		memcpy(pkt.data, src + sent, len);

		pktlen = sizeof(pkt.addr) + len;

		for (unsigned try = 1; try <= RETRY; ++try) {
			ssize_t res = write(fd, &pkt, pktlen);

			if (write_delay)
				nanosleep(&delay, NULL);

			if (res != pktlen) {
				if (try == RETRY)
					die("\nI2C write operation failed");
			} else
				break; /* Retry loop. */
		}

		putchar('w');
		if (!(i % 64))
			putchar('\n');
		fflush(stdout);
	}

	close(fd);

	if ((i % 64) != 1)
		putchar('\n');
}

static uint16_t flash_old_recv(void *dst, uint16_t offset, uint16_t size,
			       unsigned read_delay)
{
	struct timespec delay = {
		.tv_sec = read_delay / 1000,
		.tv_nsec = (read_delay % 1000) * 1000000,
	};
	uint16_t rcvd, total = 0;
	int fd, i;

	fd = open_i2c(I2C_ADDR_BOOT);

	for (rcvd = 0, i = 1; rcvd < size; rcvd += PKT_DATA_SZ, ++i) {
		uint16_t addr, len;
		ssize_t res;

		addr = htobe16(offset + rcvd);
		if (write(fd, &addr, sizeof(addr)) != sizeof(addr))
			die("\nI2C write operation failed: %m");

		len = MIN(size - rcvd, PKT_DATA_SZ);
		res = read(fd, dst + rcvd, len);
		if (res < 0)
			die("\nI2C read operation failed: %m");

		if (read_delay)
			nanosleep(&delay, NULL);

		total += res;

		putchar('r');
		if (!(i % 64))
			putchar('\n');
		fflush(stdout);

		if (res < len)
			break;
	}

	close(fd);

	if ((i % 64) != 1)
		putchar('\n');

	return total;
}

static int _cmd_write_read(i2c_addr_t addr, const void *wbuf, size_t wlen,
			   void *rbuf, size_t rlen)
{
	struct i2c_rdwr_ioctl_data trans;
	struct i2c_msg msgs[2] = {};
	int ret, fd, saved_errno;

	trans.msgs = msgs;
	trans.nmsgs = 2;
	msgs[0].addr = addr;
	msgs[0].len = wlen;
	msgs[0].buf = (void *)wbuf;
	msgs[1].addr = addr;
	msgs[1].flags = I2C_M_RD | I2C_M_STOP;
	msgs[1].len = rlen;
	msgs[1].buf = rbuf;

	fd = open_i2c(addr);

	ret = ioctl(fd, I2C_RDWR, &trans);
	saved_errno = errno;

	close(fd);

	errno = saved_errno;

	return ret;
}

static int cmd_write_read_mcu(const void *wbuf, size_t wlen, void *rbuf,
			      size_t rlen, bool fail_on_nxio)
{
	int ret;

	ret = _cmd_write_read(I2C_ADDR_MCU, wbuf, wlen, rbuf, rlen);
	if ((fail_on_nxio && ret != 2) || (ret < 0 && errno != ENXIO))
		die("%s: I2C transfer operation failed: %m", __func__);

	return ret < 0 ? ret : 0;
}

static int _cmd_read(i2c_addr_t addr, uint8_t cmd, void *buf, size_t len,
		     bool fail_on_nxio)
{
	int ret;

	ret = _cmd_write_read(addr, &cmd, 1, buf, len);
	if ((fail_on_nxio && ret != 2) || (ret < 0 && errno != ENXIO))
		die("%s: I2C transfer operation failed: %m", __func__);

	return ret < 0 ? ret : 0;
}

static int cmd_read_mcu(uint8_t cmd, void *buf, size_t len, bool fail_on_nxio)
{
	return _cmd_read(I2C_ADDR_MCU, cmd, buf, len, fail_on_nxio);
}

static void _cmd_write(i2c_addr_t addr, const void *buf, size_t len)
{
	struct i2c_rdwr_ioctl_data trans;
	struct i2c_msg msgs[1] = {};
	int fd;

	trans.msgs = msgs;
	trans.nmsgs = 1;
	msgs[0].addr = addr;
	msgs[0].flags = I2C_M_STOP;
	msgs[0].len = len;
	msgs[0].buf = (void *)buf;

	fd = open_i2c(addr);

	if (ioctl(fd, I2C_RDWR, &trans) != 1)
		die("%s: I2C transfer operation failed: %m", __func__);

	close(fd);
}

static void cmd_write_mcu(const void *buf, size_t len)
{
	_cmd_write(I2C_ADDR_MCU, buf, len);
}

static int get_version(void *dst, bool bootloader)
{
	return cmd_read_mcu(bootloader ? CMD_GET_FW_VERSION_BOOT :
					 CMD_GET_FW_VERSION_APP,
			    dst, VERSION_HASHLEN, false);
}

static void print_version(bool bootloader)
{
	const char *pfx = bootloader ? "Bootloader version: " :
				       "Application version:";
	char buf[VERSION_HASHLEN];

	if (get_version(buf, bootloader) < 0) {
		printf("%s unavailable (MCU is in bootloader)\n", pfx);
	} else {
		printf("%s ", pfx);

		for (int i = 0; i < VERSION_HASHLEN; i++)
			printf("%02x", buf[i]);

		putchar('\n');
	}
}

static uint16_t get_status_word(void)
{
	uint16_t status;

	cmd_read_mcu(CMD_GET_STATUS_WORD, &status, 2, true);

	return le16toh(status);
}

static uint8_t get_mcu_type(void)
{
	static uint8_t mcu_type;
	static bool cached;

	if (cached)
		return mcu_type;

	mcu_type = get_status_word() & STS_MCU_TYPE_MASK;
	cached = true;

	return mcu_type;
}

static void print_mcu_type(void)
{
	printf("MCU type: ");
	switch (get_mcu_type()) {
	case STS_MCU_TYPE_STM32:
		printf("STM32\n");
		break;
	case STS_MCU_TYPE_GD32:
		printf("GD32\n");
		break;
	case STS_MCU_TYPE_MKL:
		printf("MKL\n");
		break;
	default:
		printf("unknown\n");
		break;
	}
}

static bool features_cached;

static uint16_t get_features(void)
{
	static uint16_t features;

	if (features_cached)
		return features;

	if (get_status_word() & STS_FEATURES_SUPPORTED)
		cmd_read_mcu(CMD_GET_FEATURES, &features, 2, true);
	else
		features = 0;

	features = le16toh(features);
	features_cached = true;

	return le16toh(features);
}

static void _assert_feature(uint16_t mask, const char *name)
{
	if (!(get_features() & mask))
		die("MCU firmware does not support the %s feature!\n"
		    "You need to upgrade MCU firmware!", name);
}

#define assert_feature(_n) _assert_feature(FEAT_ ## _n, #_n)

static uint16_t get_bootloader_features(void)
{
	uint8_t cmd[2] = { CMD_GET_FEATURES, 0xbb };
	uint16_t features;

	if (!(get_status_word() & STS_FEATURES_SUPPORTED) ||
	    cmd_write_read_mcu(&cmd, 2, &features, 2, false) < 0)
		return FEAT_BOOTLOADER;

	return le16toh(features);
}

static void print_features(void)
{
	uint16_t features = get_features();

	printf("Features: 0x%04x%s\n", features, features ? "" : " (none)");

#define _FEAT(n)			\
	if (features & FEAT_ ## n)	\
		printf("  " # n "\n");
	_FEAT(PERIPH_MCU)
	_FEAT(EXT_CMDS)
	_FEAT(WDT_PING)
	switch (FIELD_GET(FEAT_LED_STATE_EXT_MASK, features)) {
	case 0:
		break;
	case 1:
		printf("  LED_STATE_EXT\n");
		break;
	case 2:
		printf("  LED_STATE_EXT_V32\n");
		break;
	default:
		printf("  unknown LED_STATE_EXT\n");
		break;
	}
	_FEAT(LED_GAMMA_CORRECTION)
	_FEAT(NEW_INT_API)
	_FEAT(BOOTLOADER)
	_FEAT(FLASHING)
	_FEAT(NEW_MESSAGE_API)
	_FEAT(BRIGHTNESS_INT)
	_FEAT(POWEROFF_WAKEUP)
	_FEAT(CAN_OLD_MESSAGE_API)
#undef _FEAT
}

static const char *get_firmware_prefix(void)
{
	static char prefix[32];
	static bool cached;
	const char *mcu;
	bool user_reg;
	unsigned rev;

	if (cached)
		return prefix;

	switch (get_mcu_type()) {
	case STS_MCU_TYPE_STM32:
		mcu = "stm32";
		break;
	case STS_MCU_TYPE_GD32:
		mcu = "gd32";
		break;
	case STS_MCU_TYPE_MKL:
		mcu = "mkl";
		break;
	default:
		return "unknown";
	}

	user_reg = !(get_status_word() & STS_USER_REGULATOR_NOT_SUPPORTED);

	if (get_features() & FEAT_PERIPH_MCU)
		rev = 32;
	else
		rev = 23;

	snprintf(prefix, sizeof(prefix), "%s-rev%u%s", mcu, rev,
		 user_reg ? "-user-regulator" : "");

	cached = true;

	return prefix;
}

static void print_board_firmware_type(void)
{
	printf("Board firmware type: %s\n", get_firmware_prefix());
}

static void print_checksum(void)
{
	uint32_t buf[2];

	/*
	 * If CMD_GET_FW_CHECKSUM command is not supported, the I2C transfer
	 * either fails or returns all ones.
	 */
	if (cmd_read_mcu(CMD_GET_FW_CHECKSUM, &buf, sizeof(buf), false) < 0)
		return;

	if (buf[0] == 0xffffffff)
		return;

	printf("Application firmware length: %u Bytes\n", le32toh(buf[0]));
	printf("Application firmware checksum: %#010x\n", le32toh(buf[1]));
}

typedef enum {
	MCU_PROTO_APP,
	MCU_PROTO_BOOT_OLD,
	MCU_PROTO_BOOT_NEW,
} mcu_proto_t;

static mcu_proto_t _get_mcu_proto(void)
{
	uint16_t status;

	/* Newer bootloaders support CMD_GET_STATUS_WORD and
	 * CMD_GET_FEATURES, with FEAT_BOOTLOADER bit set.
	 */
	if (!cmd_read_mcu(CMD_GET_STATUS_WORD, &status, 2, false)) {
		uint16_t features;

		status = le16toh(status);
		if (!(status & STS_FEATURES_SUPPORTED))
			return MCU_PROTO_APP;

		features = get_features();
		if (!(features & FEAT_BOOTLOADER))
			return MCU_PROTO_APP;

		if (features & FEAT_FLASHING)
			return MCU_PROTO_BOOT_NEW;
		else
			return MCU_PROTO_BOOT_OLD;
	} else {
		/* For older bootloaders, poke bootloader address */
		int fd = open_i2c(I2C_ADDR_BOOT);
		uint8_t c;
		bool res;

		res = read(fd, &c, 1) == 1;

		close(fd);

		return res ? MCU_PROTO_BOOT_OLD : MCU_PROTO_APP;
	}
}

static mcu_proto_t get_mcu_proto(void)
{
	static mcu_proto_t proto;
	static bool cached;

	if (cached)
		return proto;

	proto = _get_mcu_proto();

	/* Cache if already in bootloader. If in application, the value
	 * can change if goto_bootloader() is called.
	 */
	if (proto == MCU_PROTO_BOOT_OLD || proto == MCU_PROTO_BOOT_NEW)
		cached = true;

	return proto;
}

static void get_uptime_wakeup(uint32_t *uptime, uint32_t *wakeup)
{
	uint32_t buf[2];

	assert_feature(POWEROFF_WAKEUP);

	cmd_read_mcu(CMD_GET_UPTIME_AND_WAKEUP, &buf, sizeof(buf), true);

	if (uptime)
		*uptime = le32toh(buf[0]);

	if (wakeup)
		*wakeup = le32toh(buf[1]);
}

static void xctime(const time_t *t, char *d)
{
	char *n;

	ctime_r(t, d);

	n = strchr(d, '\n');
	if (n)
		*n = '\0';
}

static void print_wakeup_time(uint32_t uptime, uint32_t wakeup,
			      time_t wakeup_real)
{
	char str[26];

	if (!wakeup) {
		printf("Wake up time not configured\n");
		return;
	}

	xctime(&wakeup_real, str);
	printf("Wake up configured to %s +- 1 second\n", str);
	printf("- wake up time: %u %s since MCU firmware boot%s\n",
	       wakeup, wakeup == 1 ? "second" : "seconds",
	       wakeup <= uptime ? " (ELAPSED)" : "");
}

static void print_uptime_wakeup(void)
{
	uint32_t uptime, wakeup;
	time_t boot_time;
	char str[26];

	get_uptime_wakeup(&uptime, &wakeup);

	boot_time = time(NULL) - uptime;
	xctime(&boot_time, str);

	printf("MCU %s firmware booted at %s +- 1 second\n",
	       get_mcu_proto() == MCU_PROTO_APP ? "application" : "bootloader",
	       str);
	printf("- uptime: %u %s", uptime, uptime == 1 ? "second" : "seconds");
	if (uptime >= 60) {
		uint32_t d, h, m, s;

		d = uptime / 86400;
		h = (uptime % 86400) / 3600;
		m = (uptime % 3600) / 60;
		s = uptime % 60;

		printf(" (");
		if (uptime >= 3600) {
			if (uptime >= 86400)
				printf("%u %s, ", d, d == 1 ? "day" : "days");
			printf("%u %s, ", h, h == 1 ? "hour" : "hours");
		}
		printf("%u %s, %u %s)",
		       m, m == 1 ? "minute" : "minutes",
		       s, s == 1 ? "second" : "seconds");
	}
	printf("\n");

	print_wakeup_time(uptime, wakeup, boot_time + wakeup);
}

static void set_wakeup_time(const char *timestamp)
{
	uint32_t uptime, wakeup;
	time_t wakeup_real;
	uint8_t cmd[5];
	usec_t p;

	if (!strcasecmp(timestamp, "unset")) {
		wakeup = 0;
	} else {
		if (parse_timestamp(timestamp, &p) < 0)
			die("invalid argument '%s' for option '--wakeup'",
			    timestamp);

		wakeup_real = (time_t) (p / 1000000);

		if (wakeup_real < time(NULL))
			die("timestamp argument '%s' of option '--wakeup' is in the past",
			    timestamp);

		get_uptime_wakeup(&uptime, NULL);

		wakeup = wakeup_real - time(NULL) + uptime;
	}

	cmd[0] = CMD_SET_WAKEUP;
	put_unaligned_le32(wakeup, &cmd[1]);

	cmd_write_mcu(cmd, sizeof(cmd));

	if (wakeup)
		print_wakeup_time(uptime, wakeup, wakeup_real);
	else
		printf("Wake up disabled.\n");
}

static void poweroff(uint16_t arg)
{
	uint32_t uptime, wakeup;
	uint8_t cmd[9];

	get_uptime_wakeup(&uptime, &wakeup);

	printf("Powering the board off immediately!\n");
	printf("This feature should be used from within kernel poweroff driver!\n");
	printf("Data may be lost!\n");
	printf("Power on with front is %s.\n",
	       (arg & 1) ? "enabled" : "disabled");
	print_wakeup_time(uptime, wakeup, time(NULL) - uptime + wakeup);
	fflush(stdout);

	/* wait 1 second for the messages to be print to the user */
	sleep(1);

	/*
	 * This sends the power off command to the MCU, which immediately
	 * disables voltage regulators.
	 */
	cmd[0] = CMD_POWER_OFF;
	put_unaligned_le16(CMD_POWER_OFF_MAGIC, &cmd[1]);
	put_unaligned_le16(arg, &cmd[3]);
	put_unaligned_le32(crc32(0xffffffff, &cmd[1], 4), &cmd[5]);

	cmd_write_mcu(cmd, sizeof(cmd));
}

static int printf_to_file(const char *path, const char *fmt, ...)
{
	int fd = open(path, O_WRONLY), res;
	va_list ap;

	if (fd < 0)
		return fd;

	va_start(ap, fmt);
	res = vdprintf(fd, fmt, ap);
	va_end(ap);

	close(fd);

	return res;
}

static void unbind_driver(const char *path, const char *unb, const char *name)
{
	int res;

	res = printf_to_file(path, unb);
	if (res < 0) {
		if (errno != ENOENT)
			printf("Failed unbinding %s driver\n", name);
	} else
		printf("Unbound %s driver\n", name);
}

static void unbind_drivers(void)
{
	unbind_driver("/sys/bus/i2c/devices/1-002b/driver/unbind",
		      "1-002b", "LEDs");
	unbind_driver("/sys/bus/platform/devices/gpio-keys/driver/unbind",
		      "gpio-keys", "front button input");
	unbind_driver("/sys/bus/i2c/devices/1-002a/driver/unbind",
		      "1-002a", "MCU");
}

static void goto_bootloader(void)
{
	uint8_t cmd[3];

	printf("Switching MCU to bootloader...\n");

	cmd[0] = CMD_GENERAL_CONTROL;
	cmd[1] = CTL_BOOTLOADER;
	cmd[2] = CTL_BOOTLOADER;

	cmd_write_mcu(cmd, sizeof(cmd));
	features_cached = false;

	sleep(BOOTLOADER_TRANS_DELAY);
}

static void flash_firmware_old_proto(const char *image, uint16_t size,
				     const flash_opts_t *opts)
{
	uint16_t rsize;
	uint8_t result;
	char *rimage;

	printf("Sending data...\n");
	flash_old_send(image, 0, size, opts->write_delay);

	printf("Receiving data for comparison...\n");
	rimage = xmalloc(size);
	rsize = flash_old_recv(rimage, 0, size, opts->read_delay);

	if (rsize != size) {
		error("read back only %u B, expected %u B!", rsize, size);
		result = FILE_CMP_ERROR;
	} else if (memcmp(image, rimage, size)) {
		error("read back buffer different from sent buffer!");
		result = FILE_CMP_ERROR;
	} else {
		puts("Read back buffer comparison successful.");
		result = FILE_CMP_OK;
	}

	free(rimage);

	if (result == FILE_CMP_OK)
		printf("Confirming success to MCU... ");
	else
		printf("Informing MCU about failure... ");

	flash_old_send(&result, ADDR_CMP, 1, opts->write_delay);

	if (result != FILE_CMP_OK)
		die("flashing new firmware failed");
}

#define FEATURES_MAGIC		0xfea70235

typedef struct {
	uint32_t magic;
	uint16_t features;
	uint8_t status_features;
	uint8_t reserved;
	uint32_t csum;
} features_t;

static bool get_image_features_heuristic(features_t *dst, const char *image,
					 size_t size)
{
	/*
	 * Older STM32 / GD32 images do not contain features at static address,
	 * or at all. Heuristically determine needed features by inspecting the
	 * ISR vector.
	 */
	uint32_t max_addr = 0;
	int p;

	if (size < 0x110)
		return false;

	for (p = 0x4; p < 0x110; p += 4) {
		uint32_t addr = get_unaligned_le32(&image[p]);

		if (addr && (addr >> 16) != 0x0800)
			break;

		if (addr > max_addr)
			max_addr = addr;
	}

	dst->features = 0;

	if (p == 0xc0) {
		dst->status_features = STS_MCU_TYPE_STM32;
		if (max_addr < 0x08005000)
			dst->features = FEAT_BOOTLOADER;
	} else if (p == 0x110) {
		dst->status_features = STS_MCU_TYPE_GD32 |
				       STS_USER_REGULATOR_NOT_SUPPORTED;
		if (max_addr < 0x08002c00)
			dst->features = FEAT_BOOTLOADER;
	} else {
		return false;
	}

	return true;
}

static bool get_image_features(features_t *dst, const char *image, size_t size,
			       uint32_t addr)
{
	uint32_t csum;
	if (addr + sizeof(*dst) > size)
		return false;

	memcpy(dst, image + addr, sizeof(*dst));
	csum = crc32(0, dst, 8);

	dst->magic = le32toh(dst->magic);
	dst->features = le16toh(dst->features);
	dst->csum = le32toh(dst->csum);

	return dst->magic == FEATURES_MAGIC && dst->csum == csum;
}

static const char *mcutype2str(uint8_t mcu_type)
{
	static char str[4];

	switch (mcu_type) {
	case STS_MCU_TYPE_STM32:
		return "STM32";
	case STS_MCU_TYPE_GD32:
		return "GD32";
	case STS_MCU_TYPE_MKL:
		return "MKL";
	default:
		snprintf(str, sizeof(str), "%u", mcu_type);
		return str;
	}
}

static bool get_image_info(const char *image, size_t size, uint8_t *mcu_typep,
			   bool *bootloaderp, uint32_t *featuresp)
{
	bool expected_bootloader, bootloader;
	uint8_t expected_mcu_type, mcu_type;
	features_t feat;

	if (get_image_features(&feat, image, size, 0xc8)) {
		expected_bootloader = false;
		expected_mcu_type = feat.status_features & STS_MCU_TYPE_MASK;
	} else if (get_image_features(&feat, image, size, 0xd4)) {
		expected_bootloader = true;
		expected_mcu_type = feat.status_features & STS_MCU_TYPE_MASK;
	} else if (get_image_features(&feat, image, size, 0x118)) {
		expected_bootloader = false;
		expected_mcu_type = STS_MCU_TYPE_GD32;
	} else if (get_image_features(&feat, image, size, 0x124)) {
		expected_bootloader = true;
		expected_mcu_type = STS_MCU_TYPE_GD32;
	} else if (get_image_features_heuristic(&feat, image, size)) {
		expected_bootloader = feat.features & FEAT_BOOTLOADER;
		expected_mcu_type = feat.status_features & STS_MCU_TYPE_MASK;
	} else {
		return false;
	}

	bootloader = feat.features & FEAT_BOOTLOADER;
	if (bootloader != expected_bootloader)
		die("image should be for %s but is for %s!",
		    expected_bootloader ? "bootloader" : "application",
		    expected_bootloader ? "application" : "bootloader");

	mcu_type = feat.status_features & STS_MCU_TYPE_MASK;
	if (mcu_type != expected_mcu_type)
		die("image MCU type should be %s but is %s!",
		    mcutype2str(expected_mcu_type), mcutype2str(mcu_type));

	if (bootloaderp)
		*bootloaderp = bootloader;

	if (mcu_typep)
		*mcu_typep = mcu_type;

	if (featuresp)
		*featuresp = feat.features | (feat.status_features << 16);

	return true;
}

static uint8_t flash_cmd(uint8_t cmd, const void *data, uint8_t len,
			 uint32_t crc_init)
{
	uint8_t buf[len + 8], res;

	buf[0] = buf[1] = 0xff;
	buf[2] = CMD_FLASH;
	buf[3] = cmd;
	if (len)
		memcpy(&buf[4], data, len);

	put_unaligned_le32(crc32(crc_init, buf, len + 4), &buf[len + 4]);

	cmd_write_read_mcu(buf + 2, len + 6, &res, 1, true);

	return res;
}

static uint8_t flash_get_state(void)
{
	uint8_t state;

	cmd_read_mcu(CMD_FLASH, &state, 1, true);

	return state;
}

static const char *state2str(uint8_t state)
{
	static char str[4];

	switch (state) {
#define _STSTR(_s)		\
	case _s:		\
		return # _s;
	_STSTR(FLASHING_LOCKED)
	_STSTR(FLASHING_EXPECT_SIZE_AND_CSUM)
	_STSTR(FLASHING_EXPECT_PROGRAM)
	_STSTR(FLASHING_BUSY)
	_STSTR(FLASHING_DONE)
	_STSTR(FLASHING_ERR_ERASING)
	_STSTR(FLASHING_ERR_PROGRAMMING)
#undef _STSTR
	default:
		snprintf(str, sizeof(str), "%u", state);
		return str;
	}
}

static void assert_flashing_state(uint8_t got, flashing_state_t expected)
{
	if (got != expected)
		die("got unexpected state %s (expected %s)!", state2str(got),
		    state2str(expected));
}

static uint8_t wait_until_busy(void)
{
	static const struct timespec ts = {
		.tv_sec = 0,
		.tv_nsec = 1000000,
	};
	uint8_t state;

	for (int i = 0; i < 500; ++i) {
		nanosleep(&ts, NULL);
		state = flash_get_state();
		if (state != FLASHING_BUSY)
			return state;
	}

	die("timed out in FLASHING_BUSY state!");
}

static void flash_firmware_new_proto(const char *image, size_t size,
				     const flash_opts_t *opts)
{
	const uint32_t crc_init = opts->bootloader ? 0x1ef6a061 : 0x08d99d8e;
	uint32_t tmp, flashed;
	uint8_t cmd[132];
	uint8_t state;
	int i;

	state = flash_get_state();
	if (state == FLASHING_ERR_ERASING || state == FLASHING_ERR_PROGRAMMING)
		printf("Last flashing resulted in error, unlocking...\n");
	else if (state != FLASHING_LOCKED && state != FLASHING_DONE) {
		if (!opts->even_if_unlocked)
			die("flashing not locked (state %s), someone else already flashing?\n"
			    "If not, use the --even-if-unlocked option to force flashing.",
			    state2str(state));

		printf("Flashing not locked (state %s), --even-if-unlocked option is present,\n"
		       "resetting state machine...\n", state2str(state));
		state = flash_cmd(FLASH_CMD_RESET, NULL, 0, crc_init);
		assert_flashing_state(state, FLASHING_LOCKED);
	}

	state = flash_cmd(FLASH_CMD_UNLOCK, NULL, 0, crc_init);
	assert_flashing_state(state, FLASHING_EXPECT_SIZE_AND_CSUM);
	printf("Unlocked, sending size and checksum...\n");

	put_unaligned_le32(size, &cmd[0]);
	put_unaligned_le32(crc32(crc_init, image, size), &cmd[4]);

	state = flash_cmd(FLASH_CMD_SIZE_AND_CSUM, cmd, 8, crc_init);

	printf("Sending data...\n");

	tmp = crc_init;
	for (flashed = 0, i = 1; flashed < size; flashed += PKT_DATA_SZ, ++i) {
		uint8_t len = MIN(size - flashed, PKT_DATA_SZ);
		bool last = flashed + len >= size;

		memcpy(cmd, &image[flashed], len);
		tmp = crc32(tmp, cmd, len);
		put_unaligned_le32(tmp, &cmd[len]);

		state = flash_cmd(FLASH_CMD_PROGRAM, cmd, len + 4, crc_init);
		assert_flashing_state(state, FLASHING_BUSY);

		state = wait_until_busy();

		if (state == FLASHING_ERR_ERASING)
			die("\nerror erasing flash!");
		else if (state == FLASHING_ERR_PROGRAMMING)
			die("\nerror programming flash!");
		else if (state != FLASHING_EXPECT_PROGRAM && !last)
			die("\ngot unexpected state %u (expected FLASHING_EXPECT_PROGRAM)!", state);
		else if (state != FLASHING_DONE && last)
			die("\ngot unexpected state %u (expected FLASHING_DONE)!", state);

		putchar('w');
		if (!(i % 64))
			putchar('\n');
		fflush(stdout);
	}

	if ((i % 64) != 1)
		putchar('\n');
}

static void check_flashing(const char *image, size_t size,
			   const flash_opts_t *opts, mcu_proto_t mcu_proto)
{
	static const uint8_t both_message_apis_commit[VERSION_HASHLEN] = {
		0xeb, 0x5c, 0x9a, 0x8d, 0xd9, 0xec, 0xa5, 0xd1, 0xdd, 0x71,
		0x96, 0x26, 0x87, 0xf7, 0x6d, 0x8d, 0x14, 0x2f, 0xd1, 0xe2,
	};
	bool image_supports_both_message_apis;
	bool fw_supports_both_message_apis;
	uint8_t version[VERSION_HASHLEN];
	uint16_t features, status;
	bool image_is_bootloader;
	uint32_t image_features;
	uint8_t image_mcu_type;

	if (!get_image_info(image, size, &image_mcu_type, &image_is_bootloader,
			    &image_features))
		return;

	if (image_is_bootloader != opts->bootloader)
		die("requested flashing %s but image contains %s!",
		    opts->bootloader ? "bootloader" : "application",
		    opts->bootloader ? "application" : "bootloader");

	/* we are in old bootloader */
	if (mcu_proto == MCU_PROTO_BOOT_OLD) {
		error("WARNING: MCU is executing old version of bootloader, cannot determine image validity!");
		return;
	}

	if (get_mcu_type() != image_mcu_type)
		die("MCU type is %s but image is for %s!",
		    mcutype2str(get_mcu_type()), mcutype2str(image_mcu_type));

	features = get_features();
	if ((features & FEAT_PERIPH_MCU) && !(image_features & FEAT_PERIPH_MCU))
		die("board is of revision 32+ but given firmware is not!");
	else if (!(features & FEAT_PERIPH_MCU) &&
		 (image_features & FEAT_PERIPH_MCU))
		die("given firmware is for boards of revision 32+ but the board is older!");

	status = get_status_word();
	if ((status & STS_USER_REGULATOR_NOT_SUPPORTED) !=
	    ((image_features >> 16) & STS_USER_REGULATOR_NOT_SUPPORTED))
		die("board %s user regulator but given firmware %s it!",
		    (status & STS_USER_REGULATOR_NOT_SUPPORTED) ?
			"does not have" : "has",
		    (status & STS_USER_REGULATOR_NOT_SUPPORTED) ?
			"supports" : "does not support");

	image_supports_both_message_apis =
		((image_features & FEAT_NEW_MESSAGE_API) &&
		 (image_features & FEAT_CAN_OLD_MESSAGE_API)) ||
		memmem(image, size, both_message_apis_commit,
		       VERSION_HASHLEN) != NULL;

	if (image_supports_both_message_apis)
		return;

	if (get_version(version, !opts->bootloader) < 0)
		die("cannot get %s version to check for messaging API",
		    opts->bootloader ? "application" : "bootloader");

	if (!opts->bootloader && !(features & FEAT_BOOTLOADER))
		features = get_bootloader_features();

	fw_supports_both_message_apis =
		((features & FEAT_NEW_MESSAGE_API) &&
		 (features & FEAT_CAN_OLD_MESSAGE_API)) ||
		!memcmp(version, both_message_apis_commit, VERSION_HASHLEN);

	if (fw_supports_both_message_apis)
		return;

	if ((features & FEAT_NEW_MESSAGE_API) !=
	    (image_features & FEAT_NEW_MESSAGE_API))
		die("The %s firmware currently flashed in the MCU supports\n"
		    "only the %s messaging API to pass messages with %s, but the\n"
		    "%s image you want to flash only supports the %s API!\n\n"
		    "If you want to try different versions of the %s firmware, please first\n"
		    "flash %s, which supports both message passing APIs.",
		   opts->bootloader ? "application" : "bootloader",
		   (features & FEAT_NEW_MESSAGE_API) ? "new" : "old",
		   opts->bootloader ? "bootloader" : "application",
		   opts->bootloader ? "bootloader" : "application",
		   (image_features & FEAT_NEW_MESSAGE_API) ? "new" : "old",
		   opts->bootloader ? "bootloader" : "application",
		   opts->bootloader ? "the newest application firmware" : "bootloader firmware v2.99");

	if (opts->bootloader && !opts->force)
		die("flashing MCU's bootloader firmware is a dangerous operation!\n"
		    "Flashing MCU's bootloader firmware is DANGEROUS: if a power failure or other\n"
		    "event interrupts the flashing process (which lasts several seconds), you will\n"
		    "end up with a bricked board, and you will either need to buy debug cables, or\n"
		    "send the board to Turris' customer support to have it fixed.\n\n"
		    "To proceed, add the --force option. You have been warned!");
}

static void flash_firmware(const char *firmware, const flash_opts_t *opts)
{
	mcu_proto_t mcu_proto;
	ssize_t size;
	char *image;
	int fd;

	image = xmalloc(FLASH_SIZE);

	if ((fd = open(firmware, O_RDONLY)) < 0)
		die("failed to open file '%s': %m", firmware);

	if ((size = read(fd, image, FLASH_SIZE)) <= 0)
		die("failed to read file '%s': %m", firmware);

	close(fd);

	mcu_proto = get_mcu_proto();

	check_flashing(image, size, opts, mcu_proto);

	if (opts->bootloader && mcu_proto != MCU_PROTO_APP)
		die("MCU is already in bootloader, cannot flash bootloader!\n"
		    "You first need to reboot, or flash application image and"
		    "reboot, and only then flash bootloader.");

	unbind_drivers();

	if (!opts->bootloader && mcu_proto == MCU_PROTO_APP)
		goto_bootloader();

	printf("File %s (%zd B):\n", firmware, size);

	switch (get_mcu_proto()) {
	case MCU_PROTO_BOOT_OLD:
		printf("Using old flashing protocol to flash MCU application.\n");
		flash_firmware_old_proto(image, size, opts);
		break;

	case MCU_PROTO_BOOT_NEW:
	case MCU_PROTO_APP:
		printf("Using new flashing protocol to flash MCU %s.\n",
		       opts->bootloader ? "bootloader" : "application");
		flash_firmware_new_proto(image, size, opts);
		break;

	default:
		die("cannot determine whether MCU is executing application or bootloader");
	}
}

static void usage(void)
{
	printf("Usage: omnia-mcutool [OPTION]...\n\n");
	printf("omnia-mcutool -- Turris Omnia MCU utility\n\n");
	printf("Options:\n");
	printf("  -h, --help                   Print this help\n");
	printf("      --version                Print mcutool's version\n\n");
	printf(" Firmware flashing options:\n");
	printf("  -v, --firmware-version       Print version of the MCU bootloader and\n"
	       "                               application firmware\n\n");
	printf("  -f, --firmware=<FILE>        Flash MCU firmware from file FILE\n\n");
	printf("  -B, --flash-bootloader       DANGEROUS !!! THIS MAY BRICK YOUR BOARD !!!\n"
	       "                               Flash bootloader firmware instead of application\n"
	       "                               firmware. This is only supported with newer\n"
	       "                               versions of the application firmware.\n"
	       "                               If this operation is interrupted (e.g. by power\n"
	       "                               failure), your board won't boot, and you will\n"
	       "                               either need to buy debug cables or send the board\n"
	       "                               to Turris' customer support to fix it. (Although\n"
	       "                               the flashing operation is fast enough that a\n"
	       "                               power failure happening during it is improbable.)\n"
	       "                               This is why the --force option is also required.\n"
	       "                               You have been warned!\n\n");
	printf("      --force                  Force flashing bootloader firmware\n\n");
	printf("      --even-if-unlocked       Force flashing even if the flashing state machine\n"
	       "                               of the new flashing protocol is already unlocked\n\n");
	printf("      --write-delay=<DELAY>    Wait DELAY milliseconds after each sent packet\n"
	       "                               when sending firmware via the old flashing\n"
	       "                               protocol to allow the firmware to process the\n"
	       "                               packet (default %u ms)\n\n", WRITE_DELAY);
	printf("      --read-delay=<DELAY>     Read DELAY milliseconds after each received\n"
	       "                               packet when receiving firmware back for\n"
	       "                               comparison via the old flashing protocol\n"
	       "                               to allow the firmware to process the packet\n"
	       "                               (default %u ms)\n\n", READ_DELAY);
	printf(" Poweroff & wake up control options (use may interfere with kernel driver):\n");
	printf("  -u, --wakeup-status          Show the configured wake up time from potential\n"
	       "                               poweroff (see the --poweroff option) and the\n"
	       "                               time since MCU firmware start\n\n");
	printf("  -w, --wakeup=<TIMESTAMP>     Set wake up time from potential poweroff\n"
	       "                               (see the --poweroff option). The TIMESTAMP can\n"
	       "                               be absolute (e.g. \"YYYY-MM-DD hh:mm:ss\"),\n"
	       "                               relative (e.g. \"+60 minutes\"), or \"unset\"\n"
	       "                               to deconfigure wake up time\n\n");
	printf("      --poweroff[=ARG]         DO NOT USE !!! Sends the POWER_OFF command to\n"
	       "                               the MCU, whichimmediately disables the voltage\n"
	       "                               regulators to the SOC and other peripherals,\n"
	       "                               thus entering low power mode.\n"
	       "                               This option is intended for debugging purposes,\n"
	       "                               it SHOULD NOT BE USED since it skips the proper\n"
	       "                               shutdown procedures of the operating system,\n"
	       "                               which can potentially cause data loss!\n"
	       "                               Instead the kernel should use this feature from\n"
	       "                               within its system shutdown handlers.\n"
	       "                               Once powered off, the board can be powered back\n"
	       "                               on either by pressing the front button (unless\n"
	       "                               ARG is 0), or at a specified time by configuring\n"
	       "                               wake up time via the --wakeup option\n");
}

static const struct option long_options[] = {
	{ "help",		no_argument,		NULL, 'h' },
	{ "version",		no_argument,		NULL, 'V' },
	{ "firmware-version",	no_argument,		NULL, 'v' },
	{ "firmware",		required_argument,	NULL, 'f' },
	{ "flash-bootloader",	no_argument,		NULL, 'B' },
	{ "force",		no_argument,		NULL, 'F' },
	{ "even-if-unlocked",	no_argument,		NULL, 'E' },
	{ "write-delay",	required_argument,	NULL, 'd' },
	{ "read-delay",		required_argument,	NULL, 'D' },
	{ "wakeup-status",	no_argument,		NULL, 'u' },
	{ "wakeup",		required_argument,	NULL, 'w' },
	{ "poweroff",		optional_argument,	NULL, 'p' },
	{},
};

int main(int argc, char *argv[])
{
	flash_opts_t opts = {
		.write_delay = WRITE_DELAY,
		.read_delay = READ_DELAY,
	};
	const char *firmware = NULL;
	bool opt_given = false;

	argv0 = argv[0];

	while (1) {
		int opt;

		opt = getopt_long(argc, argv, "hvf:Buw:", long_options, NULL);
		if (opt == -1)
			break;

		switch (opt) {
		case 'h':
			usage();
			exit(EXIT_SUCCESS);
		case 'V':
			puts("omnia-mcutool " MCUTOOL_VERSION " (built on " __DATE__ " " __TIME__ ")\n"
			     "Copyright (C) 2016, 2022, 2023 CZ.NIC, z.s.p.o.\n"
			     "License GPLv2+: GNU GPL version 2 or later.\n"
			     "This is free software: you are free to change and redistribute it.\n"
			     "There is NO WARRANTY, to the extent permitted by law.\n\n"
			     "Written by Tomas Hlavacek and Marek Behun");
			break;
		case 'v':
			print_version(true);
			print_version(false);
			print_mcu_type();
			print_board_firmware_type();
			print_features();
			print_checksum();
			break;
		case 'f':
			if (firmware)
				die("option '--firmware' already given");
			firmware = optarg;
			break;
		case 'F':
			opts.force = true;
			break;
		case 'E':
			opts.even_if_unlocked = true;
			break;
		case 'B':
			opts.bootloader = true;
			break;
		case 'd':
			opts.write_delay = parse_uint_option("write-delay",
							     optarg, 5000, -1);
			break;
		case 'D':
			opts.read_delay = parse_uint_option("read-delay",
							    optarg, 5000, -1);
		case 'u':
			print_uptime_wakeup();
			break;
		case 'w':
			set_wakeup_time(optarg);
			break;
		case 'p':
			poweroff(parse_uint_option("poweroff", optarg, -1,
						   CMD_POWER_OFF_POWERON_BUTTON));
			break;
		default:
			die_suggest_help(NULL);
		}

		opt_given = true;
	}

	if (optind < argc)
		die_suggest_help("extra operand '%s'", argv[optind]);

	if (!opt_given)
		die_suggest_help("no options given");

	if (firmware) {
		flash_firmware(firmware, &opts);
		if (opts.bootloader)
			puts("MCU's bootloader firmware flashed successfuly.");
		else
			puts("MCU's application firmware flashed successfuly. Please reboot!");
	}

	return EXIT_SUCCESS;
}
