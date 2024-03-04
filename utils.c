/*
 * omnia-mcutool utility functions
 *
 * Copyright (C) 2024 Marek Behún
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
#include <limits.h>
#include <strings.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>

#include "utils.h"

const char *argv0;

__attribute__((__format__(__printf__, 1, 2)))
void error(const char *fmt, ...)
{
	int saved_errno;
	va_list ap;

	saved_errno = errno;

	fflush(stdout);
	fflush(stderr);

	while (*fmt == '\n') {
		fputc('\n', stderr);
		++fmt;
	}

	fprintf(stderr, "%s: ", argv0);

	errno = saved_errno;

	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);

	fputc('\n', stderr);
}

__attribute__((__noreturn__))
void die_of_unrecognized_arg(const char *opt, const char *arg)
{
	die("unrecognized argument '%s' for option '--%s'", arg, opt);
}

int parse_uint_option(const char *opt, const char *arg, int max, int def)
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

bool parse_bool_option(const char *opt, const char *arg)
{
	if (!strcasecmp(optarg, "on") || !strcasecmp(optarg, "true") ||
	    !strcasecmp(optarg, "enable") || !strcmp(optarg, "1"))
		return true;
	else if (!strcasecmp(optarg, "off") || !strcasecmp(optarg, "false") ||
		 !strcasecmp(optarg, "disable") || !strcmp(optarg, "0"))
		return false;
	else
		die_of_unrecognized_arg(opt, arg);
}

void *xmalloc(size_t size)
{
	void *res = malloc(size);

	if (!res)
		die("out of memory");

	return res;
}

char *xstrdup(const char *s)
{
	char *res = strdup(s);

	if (!res)
		die("out of memory");

	return res;
}

void xctime(const time_t *t, char *d)
{
	char *n;

	ctime_r(t, d);

	n = strchr(d, '\n');
	if (n)
		*n = '\0';
}

void close_preserve_errno(int fd)
{
	int saved_errno = errno;

	close(fd);

	errno = saved_errno;
}

int printf_to_file(const char *path, const char *fmt, ...)
{
	int fd = open(path, O_WRONLY), res;
	va_list ap;

	if (fd < 0)
		return fd;

	va_start(ap, fmt);
	res = vdprintf(fd, fmt, ap);
	va_end(ap);

	close_preserve_errno(fd);

	return res;
}

char *read_binary(const char *binary, size_t *sizep, size_t max_size)
{
	ssize_t size;
	char *image;
	int fd;

	if ((fd = open(binary, O_RDONLY)) < 0)
		return NULL;

	image = xmalloc(max_size);

	size = read(fd, image, max_size);
	if (size == max_size) {
		char dummy;

		/* if more data is available, file is too large */
		if (read(fd, &dummy, 1) == 1) {
			errno = EFBIG;
			size = -1;
		}
	}

	close_preserve_errno(fd);

	if (size <= 0) {
		if (!size)
			errno = ENODATA;
		goto err_free;
	}

	*sizep = size;

	return image;

err_free:
	free(image);
	return NULL;
}

static int read_first_line(const char *path, char buf[static 128])
{
	ssize_t rd;
	char *nl;
	int fd;

	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -1;

	rd = read(fd, buf, 128);

	close_preserve_errno(fd);

	if (rd <= 0) {
		if (!rd)
			errno = EIO;

		return -1;
	}

	nl = memchr(buf, '\n', rd);
	if (!nl) {
		errno = EIO;
		return -1;
	}

	*nl = '\0';

	return 0;
}

int read_file_ulong(const char *path, unsigned long *dst, int base)
{
	char buf[128], *end;

	if (read_first_line(path, buf) < 0)
		return -1;

	*dst = strtoul(buf, &end, base);
	if (*dst == ULONG_MAX && errno == ERANGE)
		return -1;

	if (*end != '\0') {
		errno = EIO;
		return -1;
	}

	return 0;
}

void print_first_line(const char *path, const char *name)
{
	char buf[128];

	if (read_first_line(path, buf) < 0)
		die("error reading %s: %m", name);

	printf("%s\n", buf);
}

void bin2hex(char *dst, const uint8_t *src, size_t len)
{
	static const char b2h[16] = "0123456789abcdef";

	while (len--) {
		*dst++ = b2h[*src >> 4];
		*dst++ = b2h[*src++ & 15];
	}
}

static int h2b(char x)
{
	if (x >= '0' && x <= '9')
		return x - '0';
	else if (x >= 'a' && x <= 'f')
		return x - 'a' + 10;
	else if (x >= 'A' && x <= 'F')
		return x - 'A' + 10;
	else
		return -1;
}

bool hex2bin(uint8_t *dst, const char *src, size_t len)
{
	while (len--) {
		int h, l;

		h = h2b(*src++);
		l = h2b(*src++);

		if (h < 0 || l < 0)
			return false;

		*dst++ = (h << 4) | l;
	}

	return true;
}
