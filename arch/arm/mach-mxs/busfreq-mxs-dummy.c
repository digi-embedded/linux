/*
 * Copyright (C) 2014 Digi International, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*!
 * @file busfreq-mxs-dummy.c
 *
 * @brief Busfreq API dummy functions and variables.
 *
 * This file provides empty functions and global variables needed when
 * the busfreq API is not defined in the kernel configuration.
 *
 */

#include <linux/types.h>
#include <linux/busfreq-imx6.h>
#include <linux/export.h>

void request_bus_freq(enum bus_freq_mode mode) {};
EXPORT_SYMBOL(request_bus_freq);

void release_bus_freq(enum bus_freq_mode mode) {};
EXPORT_SYMBOL(release_bus_freq);
