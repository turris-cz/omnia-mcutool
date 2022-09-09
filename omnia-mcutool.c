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

#include "i2c_iface.h"

#define MIN(a, b)				\
	({					\
		__auto_type ___a = (a);		\
		__auto_type ___b = (b);		\
		___a < ___b ? ___a : ___b;	\
	})

#define DEV_NAME	"/dev/i2c-1"
#define DEV_ADDR_BOOT	0x2c /* 0x2c for MCU in bootloader */
#define DEV_ADDR_APP	0x2a /* 0x2a for MCU in application */
#define FLASH_SIZE	43008 /* flash size, 42k for MCU */

#define PKT_DATA_SZ	128 /* 128 data bytes in one packet when flashing */
#define WRITE_DELAY	20 /* ms */
#define RETRY		3
#define FILE_CMP_OK	0xBB /* bootloader flash protocol code: flash OK */
#define FILE_CMP_ERROR	0xDD /* bootloader flash protocol code: flash failed */
#define ADDR_CMP	0xFFFF /* bootloader flash protocol code addr */

#define VERSION_HASHLEN		20 /* 20 bytes of SHA-1 git hash */
#define BOOTLOADER_TRANS_DELAY	1 /* Bootloader transition delay */

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

static int open_i2c(int addr)
{
	int fd;

	if ((fd = open(DEV_NAME, O_RDWR)) < 0)
		die("failed to open I2C bus %s@0x%02x: %m", DEV_NAME, addr);

	if (ioctl(fd, I2C_SLAVE_FORCE, addr) < 0)
		die("failed to acquire bus access and/or talk to slave %s@0x%02x: %m",
		    DEV_NAME, addr);

	return fd;
}

static void flash_send(const void *src, uint16_t offset, uint16_t size)
{
	struct timespec delay = {
		.tv_sec = 0,
		.tv_nsec = WRITE_DELAY * 1000000,
	};
	uint16_t sent;
	int fd, i;

	fd = open_i2c(DEV_ADDR_BOOT);

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

static uint16_t flash_recv(void *dst, uint16_t offset, uint16_t size)
{
	uint16_t rcvd, total = 0;
	int fd, i;

	fd = open_i2c(DEV_ADDR_BOOT);

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

static int cmd_read(uint8_t cmd, void *buf, size_t len, bool fail_on_nxio)
{
	struct i2c_rdwr_ioctl_data trans;
	struct i2c_msg msgs[2] = {};
	int ret, fd, saved_errno;

	trans.msgs = msgs;
	trans.nmsgs = 2;
	msgs[0].addr = DEV_ADDR_APP;
	msgs[0].len = 1;
	msgs[0].buf = &cmd;
	msgs[1].addr = DEV_ADDR_APP;
	msgs[1].flags = I2C_M_RD | I2C_M_STOP;
	msgs[1].len = len;
	msgs[1].buf = buf;

	fd = open_i2c(DEV_ADDR_APP);

	ret = ioctl(fd, I2C_RDWR, &trans);
	saved_errno = errno;

	close(fd);

	if ((fail_on_nxio && ret != 2) || (ret < 0 && saved_errno != ENXIO))
		die("%s: I2C transfer operation failed: %m", __func__);

	errno = saved_errno;
	return ret < 0 ? ret : 0;
}

static void cmd_write(const void *buf, size_t len)
{
	struct i2c_rdwr_ioctl_data trans;
	struct i2c_msg msgs[1] = {};
	int fd;

	trans.msgs = msgs;
	trans.nmsgs = 1;
	msgs[0].addr = DEV_ADDR_APP;
	msgs[0].flags = I2C_M_STOP;
	msgs[0].len = len;
	msgs[0].buf = (void *)buf;

	fd = open_i2c(DEV_ADDR_APP);

	if (ioctl(fd, I2C_RDWR, &trans) != 1)
		die("%s: I2C transfer operation failed: %m", __func__);

	close(fd);
}

static void print_version(bool bootloader)
{
	const char *pfx = bootloader ? "Bootloader version: " :
				       "Application version:";
	char buf[VERSION_HASHLEN];

	if (cmd_read(bootloader ? CMD_GET_FW_VERSION_BOOT :
				  CMD_GET_FW_VERSION_APP,
		     buf, VERSION_HASHLEN, false) < 0) {
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

	cmd_read(CMD_GET_STATUS_WORD, &status, 2, true);

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

static uint16_t get_features(void)
{
	static uint16_t features;
	static bool cached;

	if (cached)
		return features;

	if (get_status_word() & STS_FEATURES_SUPPORTED)
		cmd_read(CMD_GET_FEATURES, &features, 2, true);
	else
		features = 0;

	features = le16toh(features);
	cached = true;

	return features;
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
	if (cmd_read(CMD_GET_FW_CHECKSUM, &buf, sizeof(buf), false) < 0)
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

static mcu_proto_t get_mcu_proto(void)
{
	uint16_t status;

	/* Newer bootloaders support CMD_GET_STATUS_WORD and
	 * CMD_GET_FEATURES, with FEAT_BOOTLOADER bit set.
	 */
	if (!cmd_read(CMD_GET_STATUS_WORD, &status, 2, false)) {
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
		int fd = open_i2c(DEV_ADDR_BOOT);
		uint8_t c;
		bool res;

		res = read(fd, &c, 1) == 1;

		close(fd);

		return res ? MCU_PROTO_BOOT_OLD : MCU_PROTO_APP;
	}
}

static bool is_in_bootloader(void)
{
	mcu_proto_t mcu_proto = get_mcu_proto();

	return mcu_proto == MCU_PROTO_BOOT_OLD ||
	       mcu_proto == MCU_PROTO_BOOT_NEW;
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

	cmd_write(cmd, sizeof(cmd));

	sleep(BOOTLOADER_TRANS_DELAY);
}

static void flash_file(char *filename)
{
	char image[FLASH_SIZE], rimage[FLASH_SIZE];
	ssize_t size, rsize;
	uint8_t result;
	int fd;

	if ((fd = open(filename, O_RDONLY)) < 0)
		die("failed to open file: %m");

	if ((size = read(fd, image, FLASH_SIZE)) <= 0)
		die("failed to read file: %m");

	close(fd);

	printf("File %s (%zd B)\n", filename, size);
	printf("Sending data...\n");
	flash_send(image, 0, size);

	printf("Receiving data for comparison...\n");
	rsize = flash_recv(rimage, 0, size);

	if (rsize != size) {
		error("read back only %zd B, expected %zd B!", rsize, size);
		result = FILE_CMP_ERROR;
	} else if (memcmp(image, rimage, size)) {
		error("read back buffer different from sent buffer!");
		result = FILE_CMP_ERROR;
	} else {
		puts("Read back buffer comparison successful.");
		result = FILE_CMP_OK;
	}

	if (result == FILE_CMP_OK)
		printf("Confirming success to MCU... ");
	else
		printf("Informing MCU about failure... ");

	flash_send(&result, ADDR_CMP, 1);

	if (result != FILE_CMP_OK)
		die("flashing new firmware failed");
}

static void usage(void)
{
	printf("omnia-mcutool -- Turris Omnia MCU flash utility\n");
	printf("Usage: omnia-mcutool [COMMAND] [FILE].\n");
	printf("  -h : Print this help.\n");
	printf("  -v : Print version of MCU bootloader and app.\n");
	printf("  -f [FILE] : Flash application image.\n");
}

int main(int argc, char *argv[])
{
	int opt;

	argv0 = argv[0];

	opt = getopt(argc, argv, "hvf:");
	switch (opt) {
	case 'h':
		usage();
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
		unbind_drivers();
		if (!is_in_bootloader())
			goto_bootloader();
		flash_file(optarg);
		printf("Writing finished. Please reboot!\n");
		break;
	default:
		die("try the -h option for usage information");
	}

	return EXIT_SUCCESS;
}
