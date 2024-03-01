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
#ifndef SUBCOMMAND_H
#define SUBCOMMAND_H

__attribute__((__noreturn__))
void find_and_handle_subcommand(int argc, char *argv[]);

#endif /* !SUBCOMMAND_H */
