/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file xpol_abi.h
 *
 * ABI between the ext_policy host module and an uploaded policy image.
 *
 * A policy image is position-independent machine code with no external
 * symbols and no relocations, prefixed by an xpol_header_t:
 *
 *   [ xpol_header_t | code (loaded at any address) ]
 *
 * The policy is a pure function of the flat state:
 *   x = [p(3), v(3), a(3), j(3)]  local NED frame, SI units
 *   u = snap(3)
 * Math functions are provided by the host through xpol_math_t; the policy
 * must not use libc, globals or static mutable data.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define XPOL_MAGIC        0x4C4F5058u  /* "XPOL" as little-endian uint32 */
#define XPOL_ABI_VERSION  1u
#define XPOL_NO_ENTRY     0xFFFFFFFFu
#define XPOL_NX           12u
#define XPOL_NU           3u

typedef struct {
	uint32_t magic;          /* XPOL_MAGIC */
	uint16_t abi_version;    /* XPOL_ABI_VERSION */
	uint16_t header_size;    /* sizeof(xpol_header_t) */
	uint32_t image_size;     /* header + code, bytes */
	uint32_t crc32;          /* CRC-32 (IEEE 802.3) of the whole image with this field set to 0 */
	uint32_t init_offset;    /* entry offsets from the start of the code, XPOL_NO_ENTRY if absent */
	uint32_t step_offset;
	uint32_t state_size;     /* bytes of zeroed state memory the host allocates for the policy */
	uint16_t nx;             /* XPOL_NX */
	uint16_t nu;             /* XPOL_NU */
	char     name[16];       /* NUL padded */
} xpol_header_t;

typedef struct {
	float (*sinf)(float);
	float (*cosf)(float);
	float (*sqrtf)(float);
	float (*atan2f)(float, float);
	float (*expf)(float);
} xpol_math_t;

/** Optional. Return 0 on success. */
typedef int (*xpol_init_fn)(void *state, const xpol_math_t *m);

/** Required. x[XPOL_NX] in, u[XPOL_NU] out. */
typedef void (*xpol_step_fn)(void *state, const xpol_math_t *m, const float *x, float *u);

#ifdef __cplusplus
}

static_assert(sizeof(xpol_header_t) == 48, "xpol_header_t layout changed");
#endif
