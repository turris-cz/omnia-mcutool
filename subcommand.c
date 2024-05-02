/*
 * omnia-mcutool subcommands
 *
 * Copyright (C) 2024 Marek Behún
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
#include <fcntl.h>
#include <unistd.h>
#include <getopt.h>
#include <openssl/evp.h>

#include "i2c_iface.h"
#include "utils.h"
#include "defs.h"

static void print_serial_number(void)
{
	print_first_line(MCU_SYSFS_PATH "/serial_number",
			 "board serial number");
}

static void print_mac_address(void)
{
	print_first_line(MCU_SYSFS_PATH "/first_mac_address",
			 "board MAC address");
}

static void print_public_key(void)
{
	print_first_line(MCU_SYSFS_PATH "/public_key",
			 "board ECDSA public key");
}

static void do_sign(const uint8_t msg[static 32])
{
	char sig_hex[128];
	uint8_t sig[64];
	int fd;

	fd = open(DEBUGFS_DO_SIGN, O_RDWR);
	if (fd < 0) {
		if (errno == ENOENT)
			die("error accessing signing command (is the kernel compiled with CONFIG_DEBUG_FS option?)");
		goto fail;
	}

	if (write(fd, msg, 32) != 32)
		goto fail;

	if (read(fd, sig, 64) != 64)
		goto fail;

	close(fd);

	bin2hex(sig_hex, sig, 64);
	printf("%.*s\n", 128, sig_hex);

	return;

fail:
	die("failed signing message: %m");
}

static void do_sign_hash(const char *hex)
{
	uint8_t bin[32];

	if (strlen(hex) != 64 || !hex2bin(bin, hex, 32))
		die("argument is not a sha256 hash: '%s'", hex);

	do_sign(bin);
}

static void do_sign_file(const char *path)
{
	uint8_t buf[4096], hash[32];
	EVP_MD_CTX *ctx;
	ssize_t rd;
	int fd;

	ctx = EVP_MD_CTX_new();
	if (!ctx)
		die("EVP_MD_CTX_new failed");

	if (!EVP_DigestInit(ctx, EVP_sha256()))
		die("EVP_DigestInit failed");

	if (path) {
		fd = open(path, O_RDONLY);
		if (fd < 0)
			die("could not open %s for reading: %m", path);
	} else {
		fd = STDIN_FILENO;
	}

	do {
		rd = read(fd, buf, sizeof(buf));
		if (rd < 0)
			die("error reading %s: %m", path ?: "stdin");

		if (!EVP_DigestUpdate(ctx, buf, rd))
			die("EVP_DigestUpdate failed");
	} while (rd);

	if (path)
		close(fd);

	if (!EVP_DigestFinal(ctx, hash, NULL))
		die("EVP_DigestFinal failed");

	do_sign(hash);
}

static const struct {
	void (*handler)();
	int has_arg;
	uint16_t required_features;
	const char * const *aliases;
} subcmds[] = {
#define _SUBCMD(_h, _a, _f, ...)					\
	{ .handler = _h, .has_arg = _a ## _argument,			\
	  .required_features = FEAT_ ## _f,				\
	  .aliases = (const char * const[]){ __VA_ARGS__, NULL } }

	_SUBCMD(print_serial_number, no, BOARD_INFO, "serial-number", "serial"),
	_SUBCMD(print_mac_address, no, BOARD_INFO, "mac-address", "mac" ),
	_SUBCMD(print_public_key, no, CRYPTO, "public-key", "pubkey", "key"),
	_SUBCMD(do_sign_file, optional, CRYPTO, "sign" ),
	_SUBCMD(do_sign_hash, required, CRYPTO, "sign-hash" ),
};

static void handle_subcommand(typeof(subcmds[0]) *cmd, int argc, char *argv[])
{
	unsigned long features;

	if (cmd->has_arg == required_argument && argc < 3)
		die_suggest_help("too few operands for subcommand '%s'",
				 argv[1]);
	else if (argc > (cmd->has_arg == no_argument ? 2 : 3))
		die_suggest_help("too many operands for subcommand '%s'",
				 argv[1]);

	/* all subcommands require the turris-omnia-mcu kernel driver bound */
	if (read_file_ulong(MCU_SYSFS_PATH "/fw_features", &features, 0))
		die("subcommand '%s' requires the turris-omnia-mcu kernel driver to be bound",
		    argv[1]);

	if ((cmd->required_features & features) != cmd->required_features)
		die("MCU does not support subcommand '%s'", argv[1]);

	if (cmd->has_arg == no_argument)
		cmd->handler();
	else
		cmd->handler(argc == 3 ? argv[2] : NULL);
}

__attribute__((__noreturn__))
void find_and_handle_subcommand(int argc, char *argv[])
{
	for (unsigned i = 0; i < ARRAY_SIZE(subcmds); ++i) {
		for (const char * const *alias = subcmds[i].aliases; *alias;
		     ++alias) {
			if (!strcmp(argv[1], *alias)) {
				handle_subcommand(&subcmds[i], argc, argv);
				exit(EXIT_SUCCESS);
			}
		}
	}

	die_suggest_help("unrecognized subcommand '%s'", argv[1]);
}
