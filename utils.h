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
#ifndef UTILS_H
#define UTILS_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>

#define ARRAY_SIZE(__x)	(sizeof((__x)) / sizeof((__x)[0]))
#define MIN(a, b)				\
	({					\
		__auto_type ___a = (a);		\
		__auto_type ___b = (b);		\
		___a < ___b ? ___a : ___b;	\
	})

extern const char *argv0;

__attribute__((__format__(__printf__, 1, 2)))
void error(const char *fmt, ...);

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

__attribute__((__noreturn__))
void die_of_unrecognized_arg(const char *opt, const char *arg);

int parse_uint_option(const char *opt, const char *arg, int max, int def);

bool parse_bool_option(const char *opt, const char *arg);

void *xmalloc(size_t size);

char *xstrdup(const char *s);

void xctime(const time_t *t, char *d);

int printf_to_file(const char *path, const char *fmt, ...);

char *read_binary(const char *firmware, size_t *sizep, size_t max_size);

int read_file_ulong(const char *path, unsigned long *dst, int base);

void print_first_line(const char *path, const char *name);

void bin2hex(char *dst, const uint8_t *src, size_t len);
bool hex2bin(uint8_t *dst, const char *src, size_t len);

static inline void put_unaligned_le16(uint16_t val, void *dst)
{
	uint8_t *p = dst;

	*p++ = val;
	*p++ = val >> 8;
}

static inline void put_unaligned_le32(uint32_t val, void *dst)
{
	uint8_t *p = dst;

	*p++ = val;
	*p++ = val >> 8;
	*p++ = val >> 16;
	*p++ = val >> 24;
}

static inline uint32_t get_unaligned_le32(const void *src)
{
	const uint8_t *p = src;

	return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
}

#endif /* !UTILS_H */
