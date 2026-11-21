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

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "xpol_abi.h"

namespace ext_policy
{

/** CRC-32 (IEEE 802.3, reflected, init/xorout 0xFFFFFFFF), same as zlib.crc32. */
uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc = 0);

/** A verified policy image mapped into executable memory. */
struct PolicyImage {
	void *exec_mem{nullptr};
	size_t exec_size{0};
	void *state{nullptr};
	xpol_step_fn step{nullptr};
	uint32_t crc{0};
	uint32_t code_size{0};
	char name[sizeof(xpol_header_t::name) + 1] {};

	bool loaded() const { return step != nullptr; }
};

/** Math function table handed to every policy. */
const xpol_math_t *math_table();

/**
 * Read, verify (header, CRC, entry points) and map a policy image, run its init and a dry run.
 * @return 0 on success, otherwise fills err.
 */
int load_policy(const char *path, PolicyImage &out, char *err, size_t err_len);

void unload_policy(PolicyImage &img);

} // namespace ext_policy
