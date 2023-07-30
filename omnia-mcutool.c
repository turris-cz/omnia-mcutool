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
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <getopt.h>

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

#define PKT_ADDR_SZ	2 /* 2 bytes in one packet for address */
#define PKT_DATA_SZ	128 /* 128 data bytes in one packet when flashing */
#define WRITE_DELAY	20 /* ms */
#define RETRY		3
#define FILE_CMP_OK	0xBB /* bootloader flash protocol code: flash OK */
#define FILE_CMP_ERROR	0xDD /* bootloader flash protocol code: flash failed */
#define ADDR_CMP	0xFFFF /* bootloader flash protocol code addr */

#define CMD_CONTROL		0x02 /* app protocol cmd */
#define CMD_CONTROL_FLASH	0x8080 /* app protocol cmd argument */
#define CMD_VERSION_APP		0x0A /* app protocol cmd */
#define CMD_VERSION_BOOT	0x0E /* app protocol cmd */
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

static void set_addr(char buf[], int addr)
{
	buf[0] = (addr & 0xFF00) >> 8;
	buf[1] = (addr & 0xFF);
}

static void writebuff(char *buff, int startaddr, int len)
{
	struct timespec delay = {
		.tv_sec = 0,
		.tv_nsec = WRITE_DELAY * 1000000,
	};
	int size, offset, fd, r, w;
	char b[PKT_DATA_SZ + PKT_ADDR_SZ];

	fd = open_i2c(DEV_ADDR_BOOT);

	printf("Writing data ");
	fflush(stdout);
	for (offset = 0; offset < len; offset += PKT_DATA_SZ) {
		set_addr(b, startaddr + offset);
		size = MIN(len - offset, PKT_DATA_SZ);

		memcpy(b + PKT_ADDR_SZ, buff + offset, size);

		for (r = 1; r <= RETRY; r++) {
			w = write(fd, b, size + PKT_ADDR_SZ);
			nanosleep(&delay, NULL);

			if (w != size + PKT_ADDR_SZ) {
				if (r == RETRY)
					die("\nI2C write operation failed");
			} else
				break; /* Retry loop. */
		}

		printf("w");
		fflush(stdout);
	}

	close(fd);

	printf("\n");
	fflush(stdout);
}

static int readbuff(char *buff, int startaddr, int len)
{
	int size, offset, fd, rb, readtotal = 0;
	char b[PKT_ADDR_SZ];

	fd = open_i2c(DEV_ADDR_BOOT);

	for (offset = 0; offset < len; offset += PKT_DATA_SZ) {
		set_addr(b, startaddr + offset);
		size = MIN(len - offset, PKT_DATA_SZ);

		if (write(fd, b, PKT_ADDR_SZ) != 2)
			die("\nI2C write operation failed: %m");

		rb = read(fd, buff + offset, size);
		if (rb < 0)
			die("\nI2C read operation failed: %m");

		readtotal += rb;

		printf("r");
		fflush(stdout);

		if (rb < size)
			break;
	}

	close(fd);

	printf("\n");
	fflush(stdout);

	return readtotal;
}

static void read_version(char version[], char cmd)
{
	int rb, fd;

	fd = open_i2c(DEV_ADDR_APP);

	if (write(fd, &cmd, 1) != 1)
		die("I2C write operation failed: %m");

	rb = read(fd, version, VERSION_HASHLEN);
	if (rb < 0)
		die("I2C read operation failed: %m");

	close(fd);
}

static void print_version(char version[])
{
	int i;
	for (i = 0; i < VERSION_HASHLEN; i++)
		printf("%02x", version[i]);

	printf("\n");
}

static void print_app_version(void)
{
	char buf[VERSION_HASHLEN];

	read_version(buf, CMD_VERSION_APP);
	printf("Application version: ");
	print_version(buf);
}

static void print_bootloader_version(void)
{
	char buf[VERSION_HASHLEN];

	read_version(buf, CMD_VERSION_BOOT);
	printf("Bootloader version:  ");
	print_version(buf);
}

static void write_cmp_result(char result)
{
	writebuff(&result, ADDR_CMP, 1);
}

static void goto_bootloader(void)
{
	int fd;
	char cmd[3];

	printf("Switching MCU to bootloader...\n");

	cmd[0] = CMD_CONTROL;
	cmd[1] = (CMD_CONTROL_FLASH & 0xFF00) >> 8;
	cmd[2] = (CMD_CONTROL_FLASH & 0xFF);

	fd = open_i2c(DEV_ADDR_APP);

	if (write(fd, cmd, 3) != 3)
		die("I2C write operation failed: %m");

	close(fd);

	sleep(BOOTLOADER_TRANS_DELAY);
}

static void flash_file(char *filename)
{
	int fd, size, rsize;
	char buff[FLASH_SIZE], rbuff[FLASH_SIZE], result;

	if ((fd = open(filename, O_RDONLY)) < 0)
		die("failed to open file: %m");

	if ((size = read(fd, buff, FLASH_SIZE)) <= 0)
		die("failed to read file: %m");

	close(fd);

	printf("File %s (%d B):", filename, size);
	writebuff(buff, 0, size);

	printf("Readback from MCU: ");
	rsize = readbuff(rbuff, 0, size);

	if ((rsize != size) || memcmp(buff, rbuff, size)) {
		printf("WARNING: Readback failed!\n");
		result = FILE_CMP_ERROR;
	} else {
		printf("SUCCESS: Readback OK.\n");
		result = FILE_CMP_OK;
	}

	write_cmp_result(result);
}

static void usage(void)
{
	printf("omnia-mcutool -- Turris Omnia MCU flash utility\n");
	printf("Usage: omnia-mcutool [COMMAND] [FILE].\n");
	printf("  -h : Print this help.\n");
	printf("  -v : Print version of MCU bootloader and app.\n");
	printf("  -f [FILE] : Flash application image.\n");
	printf("  -r [FILE] : Flash application image. Rescue in bootloader.\n");
}

int main(int argc, char *argv[])
{
	int opt;

	argv0 = argv[0];

	opt = getopt(argc, argv, "hvf:r:");
	switch (opt) {
	case 'h':
		usage();
		break;
	case 'v':
		print_bootloader_version();
		print_app_version();
		break;
	case 'f':
		goto_bootloader();
	case 'r':
		flash_file(optarg);
		printf("Writing finished. Please reboot!\n");
		break;
	default:
		die("try the -h option for usage information");
	}

	return EXIT_SUCCESS;
}
