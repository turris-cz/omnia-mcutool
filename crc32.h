/*
 * omnia-mcutool CRC32 for some MCU commands checksumming
 *
 * Copyright (C) 2024 Marek Behún
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */
#ifndef CRC32_H
#define CRC32_H

#include <stdint.h>

uint32_t crc32(uint32_t crc, const void *_data, uint32_t len);

#endif /* CRC32_H */
