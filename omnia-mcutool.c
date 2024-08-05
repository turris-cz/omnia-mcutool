/*
 * This program interacts with Omnia MCU over I2C bus. It displays version
 * of the MCU firmware - both bootloader and application parts (which are
 * the git hashes compiled into the images) and flash application image
 * to the MCU EEPROM.
 *
 * Copyright (C) 2016, 2022, 2023. 2024 CZ.NIC
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>
#include <endian.h>

#include "crc32.h"
#include "i2c_iface.h"
#include "timeutils.h"
#include "subcommand.h"
#include "defs.h"
#include "utils.h"

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
	int ret, fd;

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

	close_preserve_errno(fd);

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

static int cmd_read_led(uint8_t cmd, void *buf, size_t len, bool fail_on_nxio)
{
	return _cmd_read(I2C_ADDR_LED, cmd, buf, len, fail_on_nxio);
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

static void cmd_write_led(const void *buf, size_t len)
{
	_cmd_write(I2C_ADDR_LED, buf, len);
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

static void set_control(uint8_t bits, uint8_t mask)
{
	uint8_t cmd[3] = { CMD_GENERAL_CONTROL, bits, mask };

	cmd_write_mcu(&cmd, sizeof(cmd));
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

static void set_ext_ctl(uint16_t bits, uint16_t mask)
{
	uint8_t cmd[5];

	assert_feature(EXT_CMDS);

	cmd[0] = CMD_EXT_CONTROL;
	put_unaligned_le16(bits, &cmd[1]);
	put_unaligned_le16(mask, &cmd[3]);

	cmd_write_mcu(&cmd, sizeof(cmd));
}

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
	_FEAT(TRNG)
	_FEAT(CRYPTO)
	_FEAT(BOARD_INFO)
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

static bool get_length_checksum(uint32_t *length, uint32_t *checksum)
{
	uint32_t buf[2];

	/*
	 * If CMD_GET_FW_CHECKSUM command is not supported, the I2C transfer
	 * either fails or returns all ones.
	 */
	if (cmd_read_mcu(CMD_GET_FW_CHECKSUM, &buf, sizeof(buf), false) < 0)
		return false;

	if (buf[0] == 0xffffffff)
		return false;

	if (length)
		*length = le32toh(buf[0]);

	if (checksum)
		*checksum = le32toh(buf[1]);

	return true;
}

static void print_checksum(void)
{
	uint32_t length, checksum;

	if (!get_length_checksum(&length, &checksum))
		return;

	printf("Application firmware length: %u Bytes\n", length);
	printf("Application firmware checksum: %#010x\n", checksum);
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

static bool mcu_proto_cached;

static mcu_proto_t get_mcu_proto(void)
{
	static mcu_proto_t proto;

	if (mcu_proto_cached)
		return proto;

	proto = _get_mcu_proto();

	mcu_proto_cached = true;

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

static void print_watchdog_status(void)
{
	uint8_t state;

	cmd_read_mcu(CMD_GET_WATCHDOG_STATE, &state, sizeof(state), true);

	printf("MCU watchdog is %s\n", state ? "enabled" : "disabled");

	if (get_features() & FEAT_WDT_PING) {
		uint16_t timeleft;

		cmd_read_mcu(CMD_GET_WDT_TIMELEFT, &timeleft, sizeof(timeleft),
			     true);

		timeleft = le16toh(timeleft);

		printf("%s: %u.%u seconds\n",
		       state ? "Time left until watchdog expiration" :
			       "Configured watchdog timeout",
		       timeleft / 10, timeleft % 10);
	} else if (state) {
		printf("Cannot determine time left until expiration\n");
		printf("(MCU firmware does not support the WDT_PING feature.\n"
		       " You need to upgrade MCU firmware.)\n");
	}
}

static void set_watchdog(bool state)
{
	uint8_t cmd[2];

	cmd[0] = CMD_SET_WATCHDOG_STATE;
	cmd[1] = state;

	cmd_write_mcu(&cmd, sizeof(cmd));

	printf("Watchdog %s\n", state ? "enabled" : "disabled");
}

static void set_watchdog_timeout(uint32_t timeout)
{
	uint8_t cmd[3];

	assert_feature(WDT_PING);

	cmd[0] = CMD_SET_WDT_TIMEOUT;

	/* the argument of this command is in deciseconds */
	put_unaligned_le16(timeout * 10, &cmd[1]);

	cmd_write_mcu(&cmd, sizeof(cmd));

	printf("Watchdog timeout set to %u seconds\n", timeout);
}

static void _print_usb_status(uint16_t status)
{
	printf("USB port 0 (front) power enabled: %s\n",
	       (status & STS_USB30_PWRON) ? "yes" : "no");
	printf("USB port 0 (front) overcurrent:   %s\n",
	       (status & STS_USB30_OVC) ? "yes (WARNING!)" : "no");
	printf("USB port 1 (rear) power enabled:  %s\n",
	       (status & STS_USB31_PWRON) ? "yes" : "no");
	printf("USB port 1 (rear) overcurrent:    %s\n",
	       (status & STS_USB31_OVC) ? "yes (WARNING!)" : "no");
}

static void print_usb_status(void)
{
	_print_usb_status(get_status_word());
}

static void set_usb_power(unsigned port, bool state)
{
	uint8_t mask;

	if (port == 1)
		mask = CTL_USB31_PWRON;
	else
		mask = CTL_USB30_PWRON;

	set_control(state ? mask : 0, mask);

	printf("USB port %u (%s) power %s\n", port,
	       port == 1 ? "rear" : "front", state ? "enabled" : "disabled");
}

static void print_leds_status(void)
{
	uint16_t features = get_features();
	const char *gamma_status;
	uint8_t brightness;

	cmd_read_led(CMD_GET_BRIGHTNESS, &brightness, sizeof(brightness), true);

	printf("Global LED brightness: %u%%\n", brightness);

	if (features & FEAT_LED_GAMMA_CORRECTION) {
		uint8_t gamma;

		cmd_read_led(CMD_GET_GAMMA_CORRECTION, &gamma, sizeof(gamma),
			     true);

		gamma_status = gamma ? "enabled" : "disabled";
	} else {
		gamma_status = "not supported (you need to upgrade MCU firmware)";
	}

	printf("LED gamma correction is %s\n", gamma_status);
}

static void set_leds_brightness(uint8_t brightness)
{
	uint8_t cmd[2];

	cmd[0] = CMD_SET_BRIGHTNESS;
	cmd[1] = brightness;

	cmd_write_mcu(&cmd, sizeof(cmd));

	printf("Global LED brightness set to %u%%\n", brightness);
}

static void set_leds_gamma(bool state)
{
	uint8_t cmd[2];

	assert_feature(LED_GAMMA_CORRECTION);

	cmd[0] = CMD_SET_GAMMA_CORRECTION;
	cmd[1] = state;

	cmd_write_led(&cmd, sizeof(cmd));

	printf("LED gamma correction %s\n", state ? "enabled" : "disabled");
}

static void stress_leds(void)
{
	struct timespec last;
	unsigned int count;
	uint8_t cmd[5];

	cmd[0] = CMD_LED_MODE;
	cmd[1] = 0x1c;
	cmd_write_led(cmd, 2);

	cmd[0] = CMD_LED_STATE;
	cmd[1] = 0x1c;
	cmd_write_led(cmd, 2);

	cmd[0] = CMD_LED_COLOR;
	cmd[1] = 0x0c;

	clock_gettime(CLOCK_MONOTONIC, &last);
	count = 0;

	printf("LEDs stress test: sending command to change all LED colors\n");

	while (1) {
		struct timespec now;
		time_t diff;

#define CHANNEL_RISE_LOOP(chnls)				\
		for (unsigned col = 0; col <= 255; col++) {	\
			chnls = col;				\
			cmd_write_led(cmd, 5);			\
			++count;				\
								\
			clock_gettime(CLOCK_MONOTONIC, &now);	\
			diff = now.tv_sec - last.tv_sec;	\
			if (now.tv_nsec < last.tv_nsec)		\
				diff -= 1;			\
			if (diff >= 1) {			\
				printf("%u cmds/sec\n", count);	\
				last = now;			\
				count = 0;			\
			}					\
		}
		CHANNEL_RISE_LOOP(cmd[2])
		CHANNEL_RISE_LOOP(cmd[2] = cmd[3])
		CHANNEL_RISE_LOOP(cmd[3])
		CHANNEL_RISE_LOOP(cmd[3] = cmd[4])
		CHANNEL_RISE_LOOP(cmd[4])
		CHANNEL_RISE_LOOP(cmd[2] = cmd[4])
		CHANNEL_RISE_LOOP(cmd[2] = cmd[3] = cmd[4])
	}
}

static void _print_wan_mode(uint16_t ext_control)
{
	printf("WAN SerDes mux endpoint:          %s (%s)\n",
	       (ext_control & EXT_CTL_PHY_SFP) ? "WAN port" :
						 "SFP cage",
	       (ext_control & EXT_CTL_PHY_SFP_AUTO) ?
			"automatically follows SFP module presence" :
			"manually selected");
}

static void _print_sfp_status(uint32_t ext_status)
{
	printf("SFP cage module detected:         %s\n",
	       (ext_status & EXT_STS_SFP_nDET) ? "no" : "yes");
}

static void print_wan_status(void)
{
	uint16_t ext_control;
	uint32_t ext_status;

	if (!(get_features() & FEAT_PERIPH_MCU))
		die("WAN SerDes mux and SFP module presence are wired to MCU\n"
		    "only board revisions 32+");

	assert_feature(EXT_CMDS);

	cmd_read_mcu(CMD_GET_EXT_STATUS_DWORD, &ext_status, sizeof(ext_status),
		     true);
	cmd_read_mcu(CMD_GET_EXT_CONTROL_STATUS, &ext_control,
		     sizeof(ext_control), true);

	_print_wan_mode(ext_control);
	_print_sfp_status(ext_status);
}

static void set_wan_mode(const char *mode)
{
	uint16_t bits, mask;
	const char *msg;

	if (!(get_features() & FEAT_PERIPH_MCU))
		die("WAN SerDes mux only available on board revisions 32+");

	if (!strcasecmp(mode, "auto")) {
		bits = EXT_CTL_PHY_SFP_AUTO;
		mask = EXT_CTL_PHY_SFP_AUTO;
		msg = "WAN SerDes mux configured to follow SFP module presence";
	} else if (!strcasecmp(mode, "phy")) {
		bits = EXT_CTL_PHY_SFP;
		mask = EXT_CTL_PHY_SFP | EXT_CTL_PHY_SFP_AUTO;
		msg = "WAN SerDes mux switched to WAN ethernet port";
	} else if (!strcasecmp(mode, "sfp")) {
		bits = 0;
		mask = EXT_CTL_PHY_SFP | EXT_CTL_PHY_SFP_AUTO;
		msg = "WAN SerDes mux switched to SFP cage";
	} else {
		die_of_unrecognized_arg("wan-mode", mode);
	}

	set_ext_ctl(bits, mask);
	puts(msg);
}

static void print_wan_mode(void)
{
	uint16_t ext_control;

	if (!(get_features() & FEAT_PERIPH_MCU))
		die("WAN SerDes mux only available on board revisions 32+");

	assert_feature(EXT_CMDS);

	cmd_read_mcu(CMD_GET_EXT_CONTROL_STATUS, &ext_control,
		     sizeof(ext_control), true);

	if (ext_control & EXT_CTL_PHY_SFP_AUTO)
		puts("auto");
	else if (ext_control & EXT_CTL_PHY_SFP)
		puts("phy");
	else
		puts("sfp");
}

static void _print_button_status(uint16_t status)
{
	printf("Front button is pressed:          %s (%s)\n",
	       (status & STS_BUTTON_PRESSED) ? "yes" : "no",
	       (status & STS_BUTTON_MODE) ?
			"press events handled by CPU" :
			"pressing changes global LED brightness");
}

static void print_button_status(void)
{
	_print_button_status(get_status_word());
}

static void set_button_mode(const char *mode)
{
	uint8_t bits, mask = CTL_BUTTON_MODE;
	const char *msg;

	if (!strcasecmp(mode, "mcu")) {
		bits = 0;
		msg = "change LED brightness";
	} else if (!strcasecmp(mode, "cpu")) {
		bits = mask;
		msg = "be handled by CPU";
	} else {
		die_of_unrecognized_arg("button-mode", mode);
	}

	set_control(bits, mask);

	printf("Front button configured to %s\n", msg);
}

static void print_button_mode(void)
{
	if (get_status_word() & STS_BUTTON_MODE)
		puts("cpu");
	else
		puts("mcu");
}

static void print_gpio_status(void)
{
	uint16_t status, features;
	uint32_t ext_status;

	status = get_status_word();

	printf("MiniPCIe/mSATA port card present: %s\n",
	       (status & STS_CARD_DET) ? "yes" : "no");
	printf("MiniPCIe/mSATA port card type:    %s\n",
	       (status & STS_CARD_DET) ? (status & STS_MSATA_IND) ? "mSATA" :
								    "MiniPCIe" :
					 "none");
	_print_button_status(status);
	_print_usb_status(status);

	features = get_features();

	if (!(features & FEAT_EXT_CMDS)) {
		printf("Cannot read status of other GPIO pins\n");
		printf("(MCU firmware does not support the EXT_CMDS feature.\n"
		       " You need to upgrade MCU firmware.)\n");
		return;
	}

	cmd_read_mcu(CMD_GET_EXT_STATUS_DWORD, &ext_status, sizeof(ext_status),
		     true);

	if (features & FEAT_PERIPH_MCU) {
		uint16_t ext_control;

		cmd_read_mcu(CMD_GET_EXT_CONTROL_STATUS, &ext_control,
			     sizeof(ext_control), true);

		printf("\nBoard revision 32+ signals:\n\n");

		_print_sfp_status(ext_status);
		_print_wan_mode(ext_control);
		printf("eMMC reset asserted:              %s (reset is ignored by eMMC)\n",
		       (ext_control & EXT_CTL_nRES_MMC) ? "no" : "yes");
		printf("LAN switch reset asserted:        %s\n",
		       (ext_control & EXT_CTL_nRES_LAN) ? "no" : "yes");
		printf("WAN PHY reset asserted:           %s\n",
		       (ext_control & EXT_CTL_nRES_PHY) ? "no" : "yes");
		printf("MiniPCIe port 0 reset asserted:   %s\n",
		       (ext_control & EXT_CTL_nPERST0) ? "no" : "yes");
		printf("MiniPCIe port 1 reset asserted:   %s\n",
		       (ext_control & EXT_CTL_nPERST1) ? "no" : "yes");
		printf("MiniPCIe port 2 reset asserted:   %s\n",
		       (ext_control & EXT_CTL_nPERST2) ? "no" : "yes");
		printf("VHV voltage regulator:            %s\n",
		       (ext_control & EXT_CTL_nVHV_CTRL) ? "disabled" :
							   "enabled");
	}

	if (features & FEAT_LED_STATE_EXT_MASK) {
		bool v32 = (features & FEAT_LED_STATE_EXT_MASK) ==
			   FEAT_LED_STATE_EXT_V32;
		char wan_char = v32 ? '*' : 'L';

		printf("\nPeripheral LED signals:\n\n");

		printf("Port 0 W%cAN/mSATA LED pin active: %s\n", wan_char,
		       (ext_status & EXT_STS_WLAN0_MSATA_LED) ? "yes" : "no");
		if (!v32)
			printf("Port 0 WPAN LED pin active:        %s\n",
			       (ext_status & EXT_STS_WPAN0_LED) ? "yes" : "no");
		printf("Port 1 W%cAN LED pin active:       %s\n", wan_char,
		       (ext_status & EXT_STS_WLAN1_LED) ? "yes" : "no");
		if (!v32)
			printf("Port 1 WPAN LED pin active:        %s\n",
			       (ext_status & EXT_STS_WPAN1_LED) ? "yes" : "no");
		printf("Port 2 W%cAN LED pin active:       %s\n", wan_char,
		       (ext_status & EXT_STS_WLAN2_LED) ? "yes" : "no");
		if (!v32)
			printf("Port 2 WPAN LED pin active:        %s\n",
			       (ext_status & EXT_STS_WPAN2_LED) ? "yes" : "no");
#define PRINT_LAN_LEDS(n)							\
		printf("Switch port %u LED 0 pin active:   %s\n", n,		\
		       (ext_status & EXT_STS_LAN ## n ## _LED0) ? "yes" : "no");\
		printf("Switch port %u LED 1 pin active:   %s\n", n,		\
		       (ext_status & EXT_STS_LAN ## n ## _LED0) ? "yes" : "no");
		PRINT_LAN_LEDS(0)
		PRINT_LAN_LEDS(1)
		PRINT_LAN_LEDS(2)
		PRINT_LAN_LEDS(3)
		PRINT_LAN_LEDS(4)
		printf("Switch CPU port LED 0 pin active: %s\n",
		       (ext_status & EXT_STS_LAN5_LED0) ? "yes" : "no");
		printf("Switch CPU port LED 1 pin active: %s\n",
		       (ext_status & EXT_STS_LAN5_LED1) ? "yes" : "no");
	}
}

static void set_gpio(const char *name, bool value)
{
	static const struct {
		const char *name;
		bool ext_ctl;
		uint16_t mask;
	} gpios[] = {
		{ "usb0_pwr",  false, CTL_USB30_PWRON },
		{ "usb1_pwr",  false, CTL_USB31_PWRON },
		{ "nres_mmc",  true,  EXT_CTL_nRES_MMC },
		{ "nres_lan",  true,  EXT_CTL_nRES_LAN },
		{ "nres_phy",  true,  EXT_CTL_nRES_PHY },
		{ "nperst0",   true,  EXT_CTL_nPERST0 },
		{ "nperst1",   true,  EXT_CTL_nPERST1 },
		{ "nperst2",   true,  EXT_CTL_nPERST2 },
		{ "phy_sfp",   true,  EXT_CTL_PHY_SFP },
		{ "nvhv_ctrl", true,  EXT_CTL_nVHV_CTRL },
	};
	unsigned idx;

	for (idx = 0; idx < ARRAY_SIZE(gpios); ++idx)
		if (!strcasecmp(gpios[idx].name, name))
			break;

	if (idx == ARRAY_SIZE(gpios))
		die_of_unrecognized_arg(value ? "gpio-set" : "gpio-clear",
					name);

	if (gpios[idx].ext_ctl) {
		if (!(get_features() & FEAT_PERIPH_MCU))
			die("GPIO %s is only wired on board revisions 32+",
			    name);

		set_ext_ctl(value ? gpios[idx].mask : 0, gpios[idx].mask);
	} else {
		set_control(value ? gpios[idx].mask : 0, gpios[idx].mask);
	}

	printf("GPIO %s %s\n", name, value ? "set" : "cleared");
}

static void print_reset_selector(void)
{
	uint8_t reset;

	cmd_read_mcu(CMD_GET_RESET, &reset, sizeof(reset), true);

	printf("%u\n", reset);
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
	unbind_driver(MCU_SYSFS_PATH "/driver/unbind", "1-002a", "MCU");
}

static void goto_bootloader(void)
{
	printf("Switching MCU to bootloader...\n");

	set_control(CTL_BOOTLOADER, CTL_BOOTLOADER);
	features_cached = false;
	mcu_proto_cached = false;

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
			   bool *bootloaderp, uint32_t *featuresp,
			   uint32_t *checksump)
{
	bool expected_bootloader, bootloader;
	uint8_t expected_mcu_type, mcu_type;
	size_t checksum_pos = 0;
	features_t feat;

	if (get_image_features(&feat, image, size, 0xc8)) {
		expected_bootloader = false;
		expected_mcu_type = feat.status_features & STS_MCU_TYPE_MASK;
		checksum_pos = 0xc4;
	} else if (get_image_features(&feat, image, size, 0xd4)) {
		expected_bootloader = true;
		expected_mcu_type = feat.status_features & STS_MCU_TYPE_MASK;
	} else if (get_image_features(&feat, image, size, 0x118)) {
		expected_bootloader = false;
		expected_mcu_type = STS_MCU_TYPE_GD32;
		checksum_pos = 0x114;
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

	if (checksump && !bootloader) {
		if (checksum_pos) {
			memcpy(checksump, &image[checksum_pos], 4);
			*checksump = le32toh(*checksump);
		} else {
			*checksump = 0xffffffff;
		}
	}

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

static const uint8_t both_message_apis_commit[VERSION_HASHLEN] = {
	0xeb, 0x5c, 0x9a, 0x8d, 0xd9, 0xec, 0xa5, 0xd1, 0xdd, 0x71,
	0x96, 0x26, 0x87, 0xf7, 0x6d, 0x8d, 0x14, 0x2f, 0xd1, 0xe2,
};

static void get_version_or_die(void *dst, bool bootloader)
{
	if (get_version(dst, bootloader) < 0)
		die("cannot get %s version", bootloader ? "bootloader" : "application");
}

static bool fw_supports_both_message_apis(bool bootloader)
{
	uint8_t version[VERSION_HASHLEN];

	get_version_or_die(version, bootloader);

	if (!bootloader) {
		uint16_t features = get_features();

		if (!(features & FEAT_BOOTLOADER) &&
		    (features & FEAT_NEW_MESSAGE_API) &&
		    (features & FEAT_CAN_OLD_MESSAGE_API))
			return true;
	}

	return !memcmp(version, both_message_apis_commit, VERSION_HASHLEN);
}

/*
 * These versions of MCU firmware (v2.99 - v4.0) leave enabled some additional
 * interrupts in NVIC_ISER that old versions of firmware expect to be disabled
 * (on GD32 MCUs only).
 */
static const uint8_t leaves_enabled_interrupts_commits[][VERSION_HASHLEN] = {
	{ 0xeb, 0x5c, 0x9a, 0x8d, 0xd9, 0xec, 0xa5, 0xd1, 0xdd, 0x71, 0x96, 0x26, 0x87, 0xf7, 0x6d, 0x8d, 0x14, 0x2f, 0xd1, 0xe2 },
	{ 0x07, 0xeb, 0xa1, 0xfb, 0x7d, 0x18, 0x6f, 0x28, 0x5f, 0x31, 0x94, 0xa8, 0x6f, 0xed, 0xcf, 0xa8, 0x5a, 0xa4, 0x56, 0x8b },
	{ 0x2b, 0x51, 0x19, 0x3a, 0x58, 0x70, 0xba, 0x23, 0x0a, 0x7f, 0xb7, 0x3b, 0xa1, 0x6f, 0xcf, 0x8a, 0x7f, 0xe2, 0xa2, 0x40 },
	{ 0xbd, 0xba, 0x68, 0xbb, 0x23, 0x8e, 0xfe, 0x5e, 0x93, 0x44, 0x5f, 0xbc, 0x12, 0x36, 0xea, 0xe9, 0x40, 0x6f, 0x7c, 0xe2 },
	{ 0x77, 0x66, 0x35, 0x35, 0xa6, 0x4a, 0xfa, 0xbf, 0xd0, 0x79, 0x55, 0x10, 0x95, 0xf3, 0xcc, 0x6e, 0xa3, 0x79, 0x16, 0xf9 },

	/* these are both v3.4, the second one is incorrect but was released as such */
	{ 0x33, 0x09, 0xbb, 0xfd, 0xa6, 0x7c, 0x34, 0xd3, 0x6d, 0xe9, 0x70, 0xe4, 0xe3, 0xe8, 0xc1, 0xe4, 0x89, 0x9c, 0xc4, 0x0c },
	{ 0x67, 0xed, 0xdc, 0x95, 0x40, 0x52, 0x6d, 0x0a, 0x9d, 0x96, 0x60, 0xf7, 0xa7, 0x86, 0x7a, 0xf9, 0xa2, 0x8a, 0x68, 0xd6 },

	{ 0xd5, 0x7b, 0x77, 0x46, 0x08, 0x51, 0x5e, 0x37, 0xc2, 0x38, 0x44, 0x77, 0xd2, 0x4a, 0x4d, 0x60, 0xe8, 0x73, 0xec, 0xf1 }
};

/*
 * These versions of MCU firmware require disabled interrupts on entry that are
 * left enabled by the commits above (on GD32 MCUs only.
 */
static const uint8_t requires_disabled_interrupts_commits[][VERSION_HASHLEN] = {
	{ 0x21, 0x74, 0xb5, 0x70, 0xbd, 0x5c, 0x03, 0x8e, 0x83, 0xe7, 0x50, 0x26, 0xa3, 0x0e, 0xb7, 0x25, 0x0b, 0xce, 0x7c, 0xcc }
};

static bool image_commit_is_one_of(const char *image, size_t size,
				   const uint8_t (*commits)[VERSION_HASHLEN], size_t n)
{
	for (int i = 0; i < n; ++i) {
		if (memmem(image, size, commits[i], VERSION_HASHLEN) != NULL)
			return true;
	}

	return false;
}

static bool version_is_one_of(bool bootloader,
			      const uint8_t (*commits)[VERSION_HASHLEN], size_t n)
{
	uint8_t version[VERSION_HASHLEN];

	get_version_or_die(version, bootloader);

	for (int i = 0; i < n; ++i) {
		if (!memcmp(version, commits[i], VERSION_HASHLEN))
			return true;
	}

	return false;
}

static bool fw_leaves_enabled_interrupts(bool bootloader)
{
	/* old versions of bootloader do not leave enabled additional interrupts */
	if (bootloader && get_mcu_proto() == MCU_PROTO_BOOT_OLD)
		return false;

	return version_is_one_of(bootloader, leaves_enabled_interrupts_commits,
				 ARRAY_SIZE(leaves_enabled_interrupts_commits));
}

static bool fw_requires_disabled_interrupts(bool bootloader)
{
	/* old versions of bootloader require disabled interrupts on entry */
	if (bootloader && get_mcu_proto() == MCU_PROTO_BOOT_OLD)
		return true;

	return version_is_one_of(bootloader, requires_disabled_interrupts_commits,
				 ARRAY_SIZE(requires_disabled_interrupts_commits));
}

static bool image_leaves_enabled_interrupts(const char *image, size_t size)
{
	return image_commit_is_one_of(image, size, leaves_enabled_interrupts_commits,
				      ARRAY_SIZE(leaves_enabled_interrupts_commits));
}

static bool image_requires_disabled_interrupts(const char *image, size_t size)
{
	return image_commit_is_one_of(image, size, requires_disabled_interrupts_commits,
				      ARRAY_SIZE(requires_disabled_interrupts_commits));
}

static const char *msg_bootloader_flash_needs_force =
	"Flashing MCU's bootloader firmware is DANGEROUS: if a power failure or other\n"
	"event interrupts the flashing process (which lasts several seconds), you will\n"
	"end up with a bricked board, and you will either need to buy debug cables, or\n"
	"send the board to Turris' customer support to have it fixed.\n\n"
	"To proceed, add the --force option. You have been warned!";

static void check_flashing(const char *image, size_t size,
			   const flash_opts_t *opts)
{
	bool image_supports_both_message_apis;
	bool image_is_bootloader;
	uint32_t image_features;
	uint8_t image_mcu_type;
	uint16_t features;

	if (opts->bootloader && !opts->force)
		die("flashing MCU's bootloader firmware is a dangerous operation!\n"
		    "%s", msg_bootloader_flash_needs_force);

	if (opts->bootloader) {
		if (get_mcu_proto() != MCU_PROTO_APP)
			die("MCU is already in bootloader, cannot flash bootloader!\n"
			    "You first need to reboot, or flash application image and\n"
			    "reboot, and only then flash bootloader.");

		features = get_features();
		if (!(features & FEAT_FLASHING))
			die("MCU application does not support flashing bootloader!\n"
			    "(Boards with MKL MCU may forbid flashing bootloader.)");
	}

	if (!get_image_info(image, size, &image_mcu_type, &image_is_bootloader,
			    &image_features, NULL))
		return;

	if (image_is_bootloader != opts->bootloader)
		die("requested flashing %s but image contains %s!",
		    opts->bootloader ? "bootloader" : "application",
		    opts->bootloader ? "application" : "bootloader");

	/* avoid incompatible bootloader + application pairs on GD32 */
	if (image_mcu_type == STS_MCU_TYPE_GD32 &&
	    ((image_leaves_enabled_interrupts(image, size) && fw_requires_disabled_interrupts(!opts->bootloader)) ||
	     (image_requires_disabled_interrupts(image, size) && fw_leaves_enabled_interrupts(!opts->bootloader))))
		die("the %s firmware currently flashed in the MCU is not\n"
		    "compatible with the %s firmware you are trying to flash and may cause\n"
		    "problems during further upgrades.\n\n"
		    "Please flash newer application firmware (at least version 4.1) or newer\n"
		    "bootloader firmware (at least version 2.99).",
		    opts->bootloader ? "application" : "bootloader",
		    opts->bootloader ? "bootloader" : "application");

	/*
	 * Request upgrading bootloader if somehow the board ended with
	 * incompatible bootloader + application pair on GD32.
	 */
	if (image_mcu_type == STS_MCU_TYPE_GD32 && !opts->bootloader &&
	    get_mcu_proto() != MCU_PROTO_BOOT_OLD &&
	    fw_requires_disabled_interrupts(true) && fw_leaves_enabled_interrupts(false))
		die("bootloader firmware upgrade is needed!\n\n"
		    "It seems that a previous upgrade caused the MCU to end up with application\n"
		    "firmware that is unable to upgrade itself without upgrading the bootloader\n"
		    "firmware first!\n\n"
		    "Please upgrade the bootloader firmware by running\n"
		    "  %s --upgrade", argv0);

	/* in old bootloader we can't read status word */
	if (get_mcu_proto() == MCU_PROTO_BOOT_OLD) {
		error("WARNING: MCU is executing old version of bootloader, cannot determine all aspects of image validity!");
	} else {
		uint16_t status;

		if (get_mcu_type() != image_mcu_type)
			die("MCU type is %s but image is for %s!",
			    mcutype2str(get_mcu_type()), mcutype2str(image_mcu_type));

		status = get_status_word();
		if ((status & STS_USER_REGULATOR_NOT_SUPPORTED) !=
		    ((image_features >> 16) & STS_USER_REGULATOR_NOT_SUPPORTED))
			die("board %s user regulator but given firmware %s it!",
			    (status & STS_USER_REGULATOR_NOT_SUPPORTED) ? "does not have" : "has",
			    (status & STS_USER_REGULATOR_NOT_SUPPORTED) ? "supports" : "does not support");
	}

	image_supports_both_message_apis =
		((image_features & FEAT_NEW_MESSAGE_API) &&
		 (image_features & FEAT_CAN_OLD_MESSAGE_API)) ||
		memmem(image, size, both_message_apis_commit,
		       VERSION_HASHLEN) != NULL;

	if (image_supports_both_message_apis)
		return;

	/* old bootloader does not support getting features, just say that it is bootloader */
	if (get_mcu_proto() == MCU_PROTO_BOOT_OLD)
		features = FEAT_BOOTLOADER;
	else
		features = get_features();

	if ((features & FEAT_PERIPH_MCU) && !(image_features & FEAT_PERIPH_MCU))
		die("board is of revision 32+ but given firmware is not!");
	else if (!(features & FEAT_PERIPH_MCU) && (image_features & FEAT_PERIPH_MCU))
		die("given firmware is for boards of revision 32+ but the board is older!");

	if (get_mcu_proto() != MCU_PROTO_BOOT_OLD &&
	    fw_supports_both_message_apis(!opts->bootloader))
		return;

	if (!opts->bootloader && !(features & FEAT_BOOTLOADER))
		features = get_bootloader_features();

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
}

static void _flash_firmware(const char *firmware, char *image, size_t size,
			    const flash_opts_t *opts)
{
	check_flashing(image, size, opts);

	unbind_drivers();

	if (!opts->bootloader && get_mcu_proto() == MCU_PROTO_APP)
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

static void flash_firmware(const char *firmware, const flash_opts_t *opts)
{
	char *image;
	size_t size;

	image = read_binary(firmware, &size, FLASH_SIZE);
	if (!image)
		die("failed to read firmware %s: %m", firmware);

	_flash_firmware(firmware, image, size, opts);

	free(image);
}

static char *request_firmware(bool bootloader, bool v2_99, uint32_t *featuresp,
			      uint32_t *checksump, char **imagep, size_t *sizep)
{
	static char fw_path[128];
	char *image;
	size_t size;

	snprintf(fw_path, sizeof(fw_path), "%s/%s%s.%s.bin", MCU_FW_PATH,
		 v2_99 ? "v2.99/" : "", get_firmware_prefix(),
		 bootloader ? "boot" : "app");

	image = read_binary(fw_path, &size, FLASH_SIZE);
	if (!image || !get_image_info(image, size, NULL, NULL, featuresp,
				      checksump)) {
		char *fname = strrchr(fw_path, '/');
		*fname++ = '\0';

		die("Firmware file %s/%s %s!\n"
		    "Please download the file from the following URL:\n"
		    "  https://gitlab.nic.cz/turris/hw/omnia_hw_ctrl/-/releases/%s/downloads/%s\n"
		    "into the\n"
		    "  %s\n"
		    "directory.", fw_path, fname,
		    image ? "is invalid" : "not found",
		    v2_99 ? "v2.99" : "permalink/latest", fname, fw_path);
	}

	*imagep = image;
	*sizep = size;

	return xstrdup(fw_path);
}

static void upgrade_bootloader(const flash_opts_t *_opts)
{
	flash_opts_t opts = *_opts;
	char *path, *image;
	size_t size;

	if (!opts.force)
		die("further upgrade requires flashing MCU's bootloader firmware.\n\n"
		    "%s", msg_bootloader_flash_needs_force);

	path = request_firmware(true, false, NULL, NULL, &image, &size);

	opts.bootloader = true;
	_flash_firmware(path, image, size, &opts);
	free(image);
	free(path);

	puts("\nSuccesfully flashed newest bootloader.");
}

static void upgrade(const flash_opts_t *_opts)
{
	uint32_t image_features, image_checksum, checksum;
	char *path, *image, version[VERSION_HASHLEN];
	flash_opts_t opts = *_opts;
	size_t size;

	/*
	 * First request the latest application firmware and check whether the
	 * current bootloader is able to flash it (if it supports the same
	 * messaging passing API).
	 */
	path = request_firmware(false, false, &image_features, &image_checksum,
				&image, &size);

	/* Check whether we need to upgrade */
	if (get_length_checksum(NULL, &checksum) && checksum == image_checksum &&
	    !get_version(version, false) &&
	    memmem(image, size, version, VERSION_HASHLEN)) {
		puts("Application firmware is up to date.");
		return;
	}

	/*
	 * Somehow the board ended with incompatible bootloader + application
	 * pair on GD32, we first have to upgrade the bootloader.
	 */
	if (get_mcu_type() == STS_MCU_TYPE_GD32 && get_mcu_proto() == MCU_PROTO_APP &&
	    fw_requires_disabled_interrupts(true) && fw_leaves_enabled_interrupts(false))
		upgrade_bootloader(&opts);

	if (!fw_supports_both_message_apis(true) &&
	    !(get_bootloader_features() & FEAT_NEW_MESSAGE_API) !=
	    !(image_features & FEAT_NEW_MESSAGE_API) &&
	    !(image_features & FEAT_CAN_OLD_MESSAGE_API)) {
		/*
		 * Bootloader firmware in MCU does not support the message
		 * passing API as the new application firmware. We first need to
		 * take care of this.
		 */

		if (!(get_features() & FEAT_BOOTLOADER) &&
		    fw_supports_both_message_apis(false)) {
			/*
			 * If MCU is running application that supports both
			 * message passing APIs, we can flash newest bootloader.
			 */
			upgrade_bootloader(&opts);
		} else {
			/*
			 * Otherwise we first need to flash application firmware
			 * v2.99 that supports both message passing APIs and
			 * request a reboot.
			 */
			char *path, *image;
			size_t size;

			path = request_firmware(false, true, NULL, NULL, &image,
						&size);

			opts.bootloader = false;
			_flash_firmware(path, image, size, &opts);
			free(image);
			free(path);

			printf("\n"
			       "Flashed temporary application firmware needed for further upgrade.\n"
			       "Please reboot and run\n"
			       "  %s --upgrade\n"
			       "again.", argv0);

			return;
		}
	}

	/*
	 * Now we have either ensured that the bootloader is compatible, or we
	 * have flashed newest bootloader. We can now flash newest application.
	 */

	opts.bootloader = false;
	_flash_firmware(path, image, size, &opts);
	free(image);
	free(path);

	puts("\n"
	     "Flashed newest application firmware.\n"
	     "Please reboot.");
}

static void usage(void)
{
	printf("Usage: %s [OPTION]...\n", argv0);
	printf("       %s {subcommand} [arg]\n\n", argv0);
	printf("omnia-mcutool -- Turris Omnia MCU utility\n\n");
	printf("Options:\n");
	printf("  -h, --help                   Print this help\n");
	printf("      --version                Print mcutool's version\n\n");
	printf(" Firmware flashing options:\n");
	printf("  -v, --firmware-version       Print version of the MCU bootloader and\n"
	       "                               application firmware\n\n");
	printf("      --upgrade                Upgrade to the newest MCU firmware\n\n");
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
	       "                               wake up time via the --wakeup option\n\n");
	printf(" MCU watchdog control options (use may interfere with kernel driver):\n");
	printf("      --watchdog-status        Show status of the MCU watchdog\n");
	printf("      --watchdog=<on|off>      Enable / disable the MCU watchdog\n");
	printf("      --watchdog-timeout=<N>   Set the timeout of the MCU watchdog to N seconds.\n"
	       "                               (Can also be used to ping the watchdog)\n\n");
	printf(" USB port power control options (use may interfere with kernel driver):\n");
	printf("      --usb-status             Show USB ports power regulator states\n");
	printf("      --usb-port-0=<on|off>    Enable/disable power for front USB port\n");
	printf("      --usb-port-1=<on|off>    Enable/disable power for rear USB port\n\n");
	printf(" LED control options (use may interfere with kernel driver):\n");
	printf("      --leds-status            Print information about LEDs settings\n");
	printf("      --leds-brightness=<VAL>  Set global LED brightness to VAL percent\n");
	printf("      --leds-gamma=<on|off>    Enable/disable LEDs gamma correction\n");
	printf("      --leds-stress-test       Stress the LEDs by rapidly changing colors\n\n");
	printf(" WAN port / SFP cage control options (use may interfere with kernel driver):\n");
	printf("      --wan-status             Print WAN status and SFP module presence status\n");
	printf("      --wan-mode=<MODE>        Set WAN SerDes mux endpoint. MODE can be one of:\n"
	       "                                 phy  : for WAN ethernet port (ethernet PHY)\n"
	       "                                 sfp  : for SFP cage\n"
	       "                                 auto : for SFP cage if SFP module is present,\n"
	       "                                        otherwise to WAN ethernet port\n");
	printf("      --get-wan-mode           Print WAN SerDes mux endpoint configuration\n\n");
	printf(" Front button control options (use may interfere with kernel driver):\n");
	printf("      --button-status          Print front button status\n");
	printf("      --button-mode=<MODE>     Configure front button mode. MODE can be one of:\n"
	       "                                 mcu : pressing the front button is handled by\n"
	       "                                       MCU to change global LED brightness\n"
	       "                                 cpu : pressing the front button is handled by\n"
	       "                                       CPU (needs kernel driver)\n");
	printf("      --get-button-mode        Print configured button mode\n\n");
	printf(" GPIO control options (use may interfere with kernel driver):\n");
	printf("      --gpio-status            Show status of MCU GPIO pins\n");
	printf("      --gpio-set=<GPIO>        Set MCU GPIO pin. GPIO can be one of:\n"
	       "                                 usb0_pwr, usb1_pwr, nres_mmc, nres_lan,\n"
	       "                                 nres_phy, nperst0, nperst1, nperst2, phy_sfp,\n"
	       "                                 nvhv_ctrl\n");
	printf("      --gpio-clear=<GPIO>      Clear MCU GPIO pin\n\n");
	printf(" Miscellaneous options:\n");
	printf("      --goto-bootloader        Request the MCU firmware to jump to bootloader\n");
	printf("      --reset-selector         Show selected factory reset level, determined\n"
	       "                               by how long the rear reset button was held\n\n");
	printf("Available subcommands if board information is stored in MCU:\n");
	printf("  serial-number, serial        Print board serial number\n");
	printf("  mac-address, mac             Print board first MAC address\n\n");
	printf("Available subcommands if board can sign messages:\n");
	printf("  public-key, pubkey, key      Print board ECDSA public key\n");
	printf("  sign [FILE]                  Sign the sha256 hash of file FILE (or standard\n"
	       "                               input if FILE is not given) with board ECDSA key\n");
	printf("  sign-hash <HASH>             Sign the sha256 hash HASH with board ECDSA key\n\n");
}

static const struct option long_options[] = {
	{ "help",		no_argument,		NULL, 'h' },
	{ "version",		no_argument,		NULL, 'V' },
	{ "firmware-version",	no_argument,		NULL, 'v' },
	{ "upgrade",		no_argument,		NULL, 'A' },
	{ "firmware",		required_argument,	NULL, 'f' },
	{ "flash-bootloader",	no_argument,		NULL, 'B' },
	{ "force",		no_argument,		NULL, 'F' },
	{ "even-if-unlocked",	no_argument,		NULL, 'E' },
	{ "write-delay",	required_argument,	NULL, 'd' },
	{ "read-delay",		required_argument,	NULL, 'D' },
	{ "wakeup-status",	no_argument,		NULL, 'u' },
	{ "wakeup",		required_argument,	NULL, 'w' },
	{ "poweroff",		optional_argument,	NULL, 'p' },
	{ "watchdog-status",	no_argument,		NULL, 'z' },
	{ "watchdog",		required_argument,	NULL, 'C' },
	{ "watchdog-timeout",	required_argument,	NULL, 'T' },
	{ "usb-status",		optional_argument,	NULL, 'U' },
	{ "usb-port-0",		required_argument,	NULL, '0' },
	{ "usb-port-1",		required_argument,	NULL, '1' },
	{ "leds-status",	no_argument,		NULL, 'l' },
	{ "leds-brightness",	required_argument,	NULL, 'L' },
	{ "leds-gamma",		required_argument,	NULL, 'G' },
	{ "leds-stress-test",	no_argument,		NULL, 'S' },
	{ "wan-status",		no_argument,		NULL, 'N' },
	{ "wan-mode",		required_argument,	NULL, 'm' },
	{ "get-wan-mode",	no_argument,		NULL, 'M' },
	{ "button-status",	no_argument,		NULL, 'Z' },
	{ "button-mode",	required_argument,	NULL, 'k' },
	{ "get-button-mode",	no_argument,		NULL, 'K' },
	{ "gpio-status",	no_argument,		NULL, 'g' },
	{ "gpio-set",		required_argument,	NULL, 's' },
	{ "gpio-clear",		required_argument,	NULL, 'c' },
	{ "goto-bootloader",	no_argument,		NULL, 'b' },
	{ "reset-selector",	no_argument,		NULL, 'R' },
	{},
};

int main(int argc, char *argv[])
{
	bool opt_given = false, do_upgrade = false;
	flash_opts_t opts = {
		.write_delay = WRITE_DELAY,
		.read_delay = READ_DELAY,
	};
	const char *firmware = NULL;

	argv0 = argv[0];

	if (argc >= 2 && argv[1][0] != '-') {
		find_and_handle_subcommand(argc, argv);
		return 0;
	}

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
			     "Copyright (C) 2016, 2022, 2023, 2024 CZ.NIC, z.s.p.o.\n"
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
		case 'A':
			do_upgrade = true;
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
		case 'l':
			print_leds_status();
			break;
		case 'L':
			set_leds_brightness(parse_uint_option("leds-brightness",
							      optarg, 100, -1));
			break;
		case 'G':
			set_leds_gamma(parse_bool_option("leds-gamma", optarg));
			break;
		case 'S':
			stress_leds();
			break;
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
		case 'z':
			print_watchdog_status();
			break;
		case 'C':
			set_watchdog(parse_bool_option("watchdog", optarg));
			break;
		case 'T':
			set_watchdog_timeout(parse_uint_option("watchdog-timeout",
							       optarg, 6553,
							       -1));
			break;
		case 'U':
			print_usb_status();
			break;
		case '0':
			set_usb_power(0, parse_bool_option("usb-port-0",
							   optarg));
			break;
		case '1':
			set_usb_power(1, parse_bool_option("usb-port-1",
							   optarg));
			break;
		case 'N':
			print_wan_status();
			break;
		case 'm':
			set_wan_mode(optarg);
			break;
		case 'M':
			print_wan_mode();
			break;
		case 'Z':
			print_button_status();
			break;
		case 'k':
			set_button_mode(optarg);
			break;
		case 'K':
			print_button_mode();
			break;
		case 'g':
			print_gpio_status();
			break;
		case 's':
			set_gpio(optarg, true);
			break;
		case 'c':
			set_gpio(optarg, false);
			break;
		case 'b':
			goto_bootloader();
			break;
		case 'R':
			print_reset_selector();
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

	if (do_upgrade && (firmware || opts.bootloader ||
			   opts.even_if_unlocked))
		die("option '--upgrade' cannot be used with option '--firmware', '--flash-bootloader' nor '--even-if-unlocked'");

	if (do_upgrade) {
		upgrade(&opts);
	} else if (firmware) {
		flash_firmware(firmware, &opts);
		if (opts.bootloader)
			puts("MCU's bootloader firmware flashed successfuly.");
		else
			puts("MCU's application firmware flashed successfuly. Please reboot!");
	}

	return EXIT_SUCCESS;
}
