/*
* Copyright (c) 2011 Sveriges Television AB <info@casparcg.com>
* Copyright (c) 2013 Technical University of Lodz Multimedia Centre <office@cm.p.lodz.pl>
*
* This file is part of CasparCG (www.casparcg.com).
*
* CasparCG is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* CasparCG is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with CasparCG. If not, see <http://www.gnu.org/licenses/>.
*
* Author: Jan Starzak, jan@ministryofgoodsteps.com
*		  Krzysztof Pyrkosz, pyrkosz@o2.pl
*/

#include "frame_operations.h"

#include <memory>
#include <tmmintrin.h>
#include <tbb/parallel_for.h>

#define OPTIMIZE_RGB_TO_BGRA
#define OPTIMIZE_BGRA_TO_RGB
//#define REPLAY_USE_ASM
//#define OPTIMIZE_BLEND_IMAGES

namespace caspar { namespace replay {

/*
#pragma warning(disable:4309)
	void bgra_to_rgb(const mmx_uint8_t* src, mmx_uint8_t* dst, int line_width)
	{
#if !defined(OPTIMIZE_BGRA_TO_RGB)
		tbb::parallel_for(tbb::blocked_range<int>(0, line_width), [=](const tbb::blocked_range<int>& r) {
			for (int i = r.begin(); i != r.end(); i++) {
				*(dst + i * 3) = *(src + i * 4 + 2);
				*(dst + i * 3 + 1) = *(src + i * 4 + 1);
				*(dst + i * 3 + 2) = *(src + i * 4);
			}
		});
#elif defined(OPTIMIZE_BGRA_TO_RGB) && !defined(REPLAY_USE_ASM)
		// bgrabgra bgrabgra => rgbrgbrg brgb     |
		// bgrabgra bgrabgra =>              rgbr | gbrgbrgb XXXXXXXX

		__m128i mask1 = _mm_set_epi8(0x80, 0x80, 0x80, 0x80,   12,   13,   14,    8,  9, 10,  4, 5, 6,  0, 1, 2);
		__m128i mask2 = _mm_set_epi8(   6,    0,    1,    2, 0x80, 0x80, 0x80, 0x80, 12, 13, 14, 8, 9, 10, 4, 5);
		__m128i mask3 = _mm_set_epi8(0xFF, 0xFF, 0xFF, 0xFF,    0,    0,    0,    0,  0,  0,  0, 0, 0,  0, 0, 0);
		__m128i src1, src2, tmp;

		const __m128i* in_vec = (__m128i*)src;
		__m64* out_vec = (__m64*)dst;

		line_width /= 32;

		while (line_width-- > 0)
		{
			src1 = in_vec[0];
			src1 = _mm_shuffle_epi8(src1, mask1);
			src2 = in_vec[1];
			src2 = _mm_shuffle_epi8(src2, mask2);
			tmp = _mm_and_si128(src2, mask3);
			src1 = _mm_or_si128(src1, tmp);
			_mm_storeu_si128((__m128i*)&out_vec[0], src1);
			_mm_storel_epi64((__m128i*)&out_vec[2], src2);

			in_vec += 2;
			out_vec +=3;
		}
#else
		int8_t mask[16] = {2,1,0,6,5,4,10,9,8,14,13,12,0xFF,0xFF,0xFF,0xFF};//{0xFF, 0xFF, 0xFF, 0xFF, 13, 14, 15, 9, 10, 11, 5, 6, 7, 1, 2, 3};

		__m128i* src_sse = (__m128i*)src;
		__m128i* dst_sse = (__m128i*)dst;

		__asm
		{
			mov esi, src_sse
			mov edi, dst_sse
			mov ecx, line_width
			lddqu xmm1, mask
l1:
			lddqu xmm0, [ esi ]
			pshufb xmm0, xmm1
			movdqu [ edi ], xmm0
			add esi, 16
			add edi, 12
			sub ecx, 4
			jnz l1
		}
#endif
	}
#pragma warning(default:4309)
*/

/*
#pragma warning(disable:4309)
	void rgb_to_bgra(const mmx_uint8_t* src, mmx_uint8_t* dst, int line_width)
	{

#ifndef OPTIMIZE_RGB_TO_BGRA
		tbb::parallel_for(tbb::blocked_range<int>(0, line_width), [=](const tbb::blocked_range<int>& r) {
			for (int i = r.begin(); i != r.end(); i++) {
				*(dst + i * 4 + 3) = 255;
				*(dst + i * 4 + 2) = *(src + i * 3);
				*(dst + i * 4 + 1) = *(src + i * 3 + 1);
				*(dst + i * 4) = *(src + i * 3 + 2);
			}
		});
#else
		const __m128i* in_vec = (__m128i*)src;
		__m128i* out_vec = (__m128i*)dst;

		line_width /= 16;

		while (line_width-- > 0)
		{
            //             0  1  2  3  4  5  6  7  8  9  10 11 12 13 14 15
            // in_vec[0]   Ra Ga Ba Rb Gb Bb Rc Gc Bc Rd Gd Bd Re Ge Be Rf
            // in_vec[1]   Gg Bg Rh Gh Bh Ri Gi Bi Rj Gj Bj Rk Gk Bk Rl Gl
            // in_vec[2]   Bl Rm Gm Bm Rn Gn Bn Ro Go Bo Rp Gp Bp Rq Gq Bq

			__m128i in1, in2, in3;
			__m128i out;

			in1 = in_vec[0];

			out = _mm_shuffle_epi8(in1, _mm_set_epi8(0xff, 9, 10, 11, 0xff, 6, 7, 8, 0xff, 3, 4, 5, 0xff, 0, 1, 2));
			out = _mm_or_si128(out,     _mm_set_epi8(0xff, 0,  0,  0, 0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0));
			out_vec[0] = out;

			in2 = in_vec[1];

			in1 = _mm_and_si128(in1, _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,    0,    0,    0,    0,    0,    0,    0,    0));
			out = _mm_and_si128(in2, _mm_set_epi8(0,       0,    0,    0,    0,    0,    0,    0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff));
			out = _mm_or_si128(out, in1);
			out = _mm_shuffle_epi8(out, _mm_set_epi8(0xff, 5, 6, 7, 0xff, 2, 3, 4, 0xff, 15, 0, 1, 0xff, 12, 13, 14));
			out = _mm_or_si128(out, _mm_set_epi8(0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0));
			out_vec[1] = out;

			in3 = in_vec[2];
			in_vec += 3;

			in2 = _mm_and_si128(in2, _mm_set_epi8(0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0, 0, 0, 0, 0));
			out = _mm_and_si128(in3, _mm_set_epi8(0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff));
			out = _mm_or_si128(out, in2);
			out = _mm_shuffle_epi8(out, _mm_set_epi8(0xff, 1, 2, 3, 0xff, 14, 15, 0, 0xff, 11, 12, 13, 0xff, 8, 9, 10));
			out = _mm_or_si128(out, _mm_set_epi8(0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0));
			out_vec[2] = out;

			out = _mm_shuffle_epi8(in3, _mm_set_epi8(0xff, 13, 14, 15, 0xff, 10, 11, 12, 0xff, 7, 8, 9, 0xff, 4, 5, 6));
			out = _mm_or_si128(out, _mm_set_epi8(0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0, 0xff, 0, 0, 0));
			out_vec[3] = out;

			out_vec += 4;
		}
#endif
	}
#pragma warning(default:4309)
*/

/*
    void split_frame_to_fields(const mmx_uint8_t* src, mmx_uint8_t* dst1, mmx_uint8_t* dst2, uint32_t width, uint32_t height, uint32_t stride)
	{
		uint32_t full_row = width * stride;
		tbb::parallel_for(tbb::blocked_range<uint32_t>(0, height/2), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); ++i)
			{
				memcpy((dst1 + i * full_row), (src + i * 2 * full_row), full_row);
				memcpy((dst2 + i * full_row), (src + (i * 2 + 1) * full_row), full_row);
			}
		});
    }
*/

	void interlace_fields(const mmx_uint8_t* src1, const mmx_uint8_t* src2, mmx_uint8_t* dst, uint32_t width, uint32_t height, uint32_t stride)
	{
		uint32_t full_row = width * stride;
		tbb::parallel_for(tbb::blocked_range<uint32_t>(0, height/2), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); ++i)
			{
				memcpy((dst + i * 2 * full_row), (src1 + i * full_row), full_row);
				memcpy((dst + (i * 2 + 1) * full_row), (src2 + i * full_row), full_row);
			}
		});
	}

    void interlace_frames(const mmx_uint8_t* src1, const mmx_uint8_t* src2, mmx_uint8_t* dst, uint32_t width, uint32_t height, uint32_t stride)
	{
		uint32_t full_row = width * stride;
		tbb::parallel_for(tbb::blocked_range<uint32_t>(0, height/2), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); ++i)
			{
				memcpy((dst + i * 2 * full_row), (src1 + i * 2 * full_row), full_row);
				memcpy((dst + (i * 2 + 1) * full_row), (src2 + (i * 2 + 1) * full_row), full_row);
			}
		});
    }
	
	void field_double(const mmx_uint8_t* src, mmx_uint8_t* dst, uint32_t width, uint32_t height, uint32_t stride)
	{
		uint32_t full_row = width * stride;
		/* tbb::parallel_for(tbb::blocked_range<uint32_t>(0, height/2), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); ++i)
			{
				memcpy((dst + i * 2 * full_row), (src + i * full_row), full_row);
				memcpy((dst + (i * 2 + 1) * full_row), (src + i  * full_row), full_row);
			}
        }); */
		tbb::parallel_for(tbb::blocked_range<uint32_t>(0, height/2 - 1), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); ++i)
			{
				for (uint32_t j = 0; j < full_row; ++j)
				{
					dst[i * 2 * full_row + j] = src[i * full_row + j];
					dst[(i * 2 + 1) * full_row + j] = (src[i * full_row + j] >> 1) + (src[(i + 1) * full_row + j] >> 1);
				}
			}
		});
	}

#pragma warning(disable:4309 4244)
	// max level is 63
	// level = 63 means 100% src1, level = 0 means 100% src2
	void blend_images(const mmx_uint8_t* src1, mmx_uint8_t* src2, mmx_uint8_t* dst, uint32_t width, uint32_t height, uint32_t stride, uint8_t level)
	{
		uint32_t full_size = width * height * stride;
		uint16_t level_16 = (uint16_t)level;
#ifndef OPTIMIZE_BLEND_IMAGES
		tbb::parallel_for(tbb::blocked_range<uint32_t>(0, full_size), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); i++)
			{
				dst[i] = (uint8_t)((((int)src1[i] * level_16) >> 6) + (((int)src2[i] * (64 - level_16)) >> 6));
			}
		});
#else
		const __m64* in1_vec = (__m64*)src1;
		const __m64* in2_vec = (__m64*)src2;
		__m64* out_vec = (__m64*)dst;
		__m128i mul = _mm_set_epi16(level_16, level_16, level_16, level_16, level_16, level_16, level_16, level_16);
		__m128i in1, in2;
		__m128i mask = _mm_set_epi8(0x80, 7, 0x80, 6, 0x80, 5, 0x80, 4, 0x80, 3, 0x80, 2, 0x80, 1, 0x80, 0);
		__m128i umask = _mm_set_epi8(0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 14, 12, 10, 8, 6, 4, 2, 0);

		full_size /= 24;
		while (full_size-- > 0)
		{
			/*             01 23 45 67 89 AB CD EF
			 * in_vec[0]   Ra Ga Ba Rb Gb Bb Rc Gc     
			 * in_vec[1]   Bc Rd Gd Bd Re Ge Be Rf 
			 * in_vec[2]   Gg Bg Rh Gh Bh Ri Gi Bi
             */

			in1 = _mm_loadl_epi64((__m128i*)&in1_vec[0]);
			in2 = _mm_loadl_epi64((__m128i*)&in2_vec[0]);
			in1 = _mm_shuffle_epi8(in1, mask);
			in2 = _mm_shuffle_epi8(in2, mask);
			in1 = _mm_sub_epi16(in1, in2);
			in1 = _mm_mullo_epi16 (in1, mul);
			in1 = _mm_srli_epi16(in1, 6);
			in1 = _mm_add_epi16(in1, in2);
			in1 = _mm_shuffle_epi8(in1, umask);
			_mm_storel_epi64((__m128i*)&out_vec[0], in1);

			in1 = _mm_loadl_epi64((__m128i*)&in1_vec[1]);
			in2 = _mm_loadl_epi64((__m128i*)&in2_vec[1]);
			in1 = _mm_shuffle_epi8(in1, mask);
			in2 = _mm_shuffle_epi8(in2, mask);
			in1 = _mm_sub_epi16(in1, in2);
			in1 = _mm_mullo_epi16 (in1, mul);
			in1 = _mm_srli_epi16(in1, 6);
			in1 = _mm_add_epi16(in1, in2);
			in1 = _mm_shuffle_epi8(in1, umask);
			_mm_storel_epi64((__m128i*)&out_vec[1], in1);

			in1 = _mm_loadl_epi64((__m128i*)&in1_vec[2]);
			in2 = _mm_loadl_epi64((__m128i*)&in2_vec[2]);
			in1 = _mm_shuffle_epi8(in1, mask);
			in2 = _mm_shuffle_epi8(in2, mask);
			in1 = _mm_sub_epi16(in1, in2);
			in1 = _mm_mullo_epi16 (in1, mul);
			in1 = _mm_srli_epi16(in1, 6);
			in1 = _mm_add_epi16(in1, in2);
			in1 = _mm_shuffle_epi8(in1, umask);
			_mm_storel_epi64((__m128i*)&out_vec[2], in1);

			in1_vec += 3;
			in2_vec += 3;
			out_vec += 3;
		}
#endif
	}

    void black_frame(mmx_uint8_t* dst, uint32_t width, uint32_t height, uint32_t stride)
	{
		uint32_t full_size = width * height;
		tbb::parallel_for(tbb::blocked_range<uint32_t>(0, full_size), [=](const tbb::blocked_range<uint32_t>& r)
		{
			for (auto i = r.begin(); i != r.end(); i++)
			{
				dst[i*stride] = 0;
				dst[i*stride+1] = 0;
				dst[i*stride+2] = 0;
				if (stride == 4)
					dst[i*stride+3] = 255;
			}
		});
	}
#pragma warning(default:4309)
}}
