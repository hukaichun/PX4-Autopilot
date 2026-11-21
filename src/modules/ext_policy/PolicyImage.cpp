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

#include "PolicyImage.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__PX4_POSIX)
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace ext_policy
{

static constexpr size_t MAX_IMAGE_SIZE = 64 * 1024;
static constexpr uint32_t MAX_STATE_SIZE = 4096;

uint32_t crc32(const uint8_t *data, size_t len, uint32_t crc)
{
	crc = ~crc;

	for (size_t i = 0; i < len; i++) {
		crc ^= data[i];

		for (int k = 0; k < 8; k++) {
			crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
		}
	}

	return ~crc;
}

static float m_sinf(float x) { return sinf(x); }
static float m_cosf(float x) { return cosf(x); }
static float m_sqrtf(float x) { return sqrtf(x); }
static float m_atan2f(float y, float x) { return atan2f(y, x); }
static float m_expf(float x) { return expf(x); }

static const xpol_math_t g_math{m_sinf, m_cosf, m_sqrtf, m_atan2f, m_expf};

const xpol_math_t *math_table() { return &g_math; }

// --- executable memory (platform specific) ---

static void *exec_alloc(const uint8_t *code, size_t len, size_t &mapped)
{
#if defined(__PX4_POSIX)
	const size_t page = (size_t)sysconf(_SC_PAGESIZE);
	mapped = (len + page - 1) / page * page;
	void *mem = mmap(nullptr, mapped, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);

	if (mem == MAP_FAILED) {
		return nullptr;
	}

	memcpy(mem, code, len);

	if (mprotect(mem, mapped, PROT_READ | PROT_EXEC) != 0) {
		munmap(mem, mapped);
		return nullptr;
	}

	return mem;
#else
	// TODO(NuttX): copy into an executable RAM region, clean D-cache, invalidate I-cache, set the Thumb bit
	(void)code;
	(void)len;
	mapped = 0;
	return nullptr;
#endif
}

static void exec_free(void *mem, size_t mapped)
{
#if defined(__PX4_POSIX)

	if (mem) {
		munmap(mem, mapped);
	}

#else
	(void)mem;
	(void)mapped;
#endif
}

// --- loader ---

static int fail(char *err, size_t err_len, const char *msg, unsigned a = 0, unsigned b = 0)
{
	snprintf(err, err_len, msg, a, b);
	return -1;
}

int load_policy(const char *path, PolicyImage &out, char *err, size_t err_len)
{
	out = PolicyImage{};

	FILE *f = fopen(path, "rb");

	if (!f) {
		return fail(err, err_len, "cannot open file");
	}

	uint8_t *buf = (uint8_t *)malloc(MAX_IMAGE_SIZE + 1);

	if (!buf) {
		fclose(f);
		return fail(err, err_len, "out of memory");
	}

	const size_t size = fread(buf, 1, MAX_IMAGE_SIZE + 1, f);
	fclose(f);

	int ret = -1;
	xpol_header_t hdr;
	uint32_t crc_calc;
	const uint8_t *code;
	uint32_t code_size;

	if (size > MAX_IMAGE_SIZE) {
		ret = fail(err, err_len, "image larger than %u bytes", MAX_IMAGE_SIZE);
		goto out;
	}

	if (size < sizeof(hdr)) {
		ret = fail(err, err_len, "file too small (%u bytes)", (unsigned)size);
		goto out;
	}

	memcpy(&hdr, buf, sizeof(hdr));

	if (hdr.magic != XPOL_MAGIC) {
		ret = fail(err, err_len, "bad magic 0x%08x", hdr.magic);
		goto out;
	}

	if (hdr.abi_version != XPOL_ABI_VERSION || hdr.header_size != sizeof(hdr)) {
		ret = fail(err, err_len, "unsupported ABI %u / header size %u", hdr.abi_version, hdr.header_size);
		goto out;
	}

	if (hdr.image_size != size) {
		ret = fail(err, err_len, "size mismatch: header %u, file %u", hdr.image_size, (unsigned)size);
		goto out;
	}

	// CRC over the whole image with the crc field zeroed
	{
		xpol_header_t zeroed = hdr;
		zeroed.crc32 = 0;
		crc_calc = crc32((const uint8_t *)&zeroed, sizeof(zeroed));
		crc_calc = crc32(buf + sizeof(hdr), size - sizeof(hdr), crc_calc);
	}

	if (crc_calc != hdr.crc32) {
		ret = fail(err, err_len, "CRC mismatch: header 0x%08x, computed 0x%08x", hdr.crc32, crc_calc);
		goto out;
	}

	if (hdr.nx != XPOL_NX || hdr.nu != XPOL_NU) {
		ret = fail(err, err_len, "dimension mismatch nx=%u nu=%u", hdr.nx, hdr.nu);
		goto out;
	}

	code = buf + sizeof(hdr);
	code_size = (uint32_t)(size - sizeof(hdr));

	if (hdr.step_offset >= code_size
	    || (hdr.init_offset != XPOL_NO_ENTRY && hdr.init_offset >= code_size)) {
		ret = fail(err, err_len, "entry point outside code (%u bytes)", code_size);
		goto out;
	}

	if (hdr.state_size > MAX_STATE_SIZE) {
		ret = fail(err, err_len, "state size %u > %u", hdr.state_size, MAX_STATE_SIZE);
		goto out;
	}

	out.exec_mem = exec_alloc(code, code_size, out.exec_size);

	if (!out.exec_mem) {
		ret = fail(err, err_len, "cannot allocate executable memory");
		goto out;
	}

	out.state = calloc(1, hdr.state_size > 0 ? hdr.state_size : 1);

	if (!out.state) {
		ret = fail(err, err_len, "out of memory");
		goto out;
	}

	out.step = (xpol_step_fn)((uintptr_t)out.exec_mem + hdr.step_offset);
	out.crc = hdr.crc32;
	out.code_size = code_size;
	memcpy(out.name, hdr.name, sizeof(hdr.name));
	out.name[sizeof(hdr.name)] = '\0';

	if (hdr.init_offset != XPOL_NO_ENTRY) {
		xpol_init_fn init = (xpol_init_fn)((uintptr_t)out.exec_mem + hdr.init_offset);

		if (init(out.state, &g_math) != 0) {
			ret = fail(err, err_len, "policy init failed");
			goto out;
		}
	}

	// dry run on a few states: outputs must be finite
	{
		const float probes[3][XPOL_NX] = {
			{0, 0, -5, 0, 0, 0, 0, 0, 0, 0, 0, 0},
			{1.f, -2.f, -4.f, 0.5f, 0.3f, -0.2f, 0.1f, -0.1f, 0.2f, 0.f, 0.3f, 0.f},
			{-3.f, 2.f, -6.f, -1.f, 1.f, 0.5f, -0.5f, 0.5f, 0.f, 1.f, -1.f, 0.5f},
		};

		for (const auto &x : probes) {
			float u[XPOL_NU] {};
			out.step(out.state, &g_math, x, u);

			for (float ui : u) {
				if (!isfinite(ui)) {
					ret = fail(err, err_len, "dry run produced non-finite output");
					goto out;
				}
			}
		}

		// reset the policy state after probing
		memset(out.state, 0, hdr.state_size);

		if (hdr.init_offset != XPOL_NO_ENTRY) {
			((xpol_init_fn)((uintptr_t)out.exec_mem + hdr.init_offset))(out.state, &g_math);
		}
	}

	ret = 0;

out:
	free(buf);

	if (ret != 0) {
		unload_policy(out);
	}

	return ret;
}

void unload_policy(PolicyImage &img)
{
	exec_free(img.exec_mem, img.exec_size);
	free(img.state);
	img = PolicyImage{};
}

} // namespace ext_policy
