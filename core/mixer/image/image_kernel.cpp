/*
* copyright (c) 2010 Sveriges Television AB <info@casparcg.com>
*
*  This file is part of CasparCG.
*
*    CasparCG is free software: you can redistribute it and/or modify
*    it under the terms of the GNU General Public License as published by
*    the Free Software Foundation, either version 3 of the License, or
*    (at your option) any later version.
*
*    CasparCG is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*    GNU General Public License for more details.

*    You should have received a copy of the GNU General Public License
*    along with CasparCG.  If not, see <http://www.gnu.org/licenses/>.
*
*/
#include "../../stdafx.h"

#include "image_kernel.h"

#include "blending_glsl.h"
#include "../gpu/shader.h"
#include "../gpu/device_buffer.h"

#include <common/exception/exceptions.h>
#include <common/gl/gl_check.h>

#include <core/video_format.h>
#include <core/producer/frame/pixel_format.h>
#include <core/producer/frame/image_transform.h>

#include <GL/glew.h>

#include <boost/noncopyable.hpp>

#include <unordered_map>

namespace caspar { namespace core {
	
GLubyte upper_pattern[] = {
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00};
		
GLubyte lower_pattern[] = {
	0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 
	0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff,
	0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff,	0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff,
	0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff,	0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,	0xff, 0xff, 0xff, 0xff};

struct image_kernel::implementation : boost::noncopyable
{	
	std::unique_ptr<shader> shader_;
	
	core::video_mode::type	last_mode_;
	size_t					last_width_;
	size_t					last_height_;
	
	std::string get_blend_color_func()
	{
		return 
			
		get_blend_glsl()
		
		+
			
		"vec3 get_blend_color(vec3 back, vec3 fore)											\n"
		"{																					\n"
		"	switch(blend_mode)																\n"
		"	{																				\n"
		"	case  0: return BlendNormal(back, fore);										\n"
		"	case  1: return BlendLighten(back, fore);										\n"
		"	case  2: return BlendDarken(back, fore);										\n"
		"	case  3: return BlendMultiply(back, fore);										\n"
		"	case  4: return BlendAverage(back, fore);										\n"
		"	case  5: return BlendAdd(back, fore);											\n"
		"	case  6: return BlendSubstract(back, fore);										\n"
		"	case  7: return BlendDifference(back, fore);									\n"
		"	case  8: return BlendNegation(back, fore);										\n"
		"	case  9: return BlendExclusion(back, fore);										\n"
		"	case 10: return BlendScreen(back, fore);										\n"
		"	case 11: return BlendOverlay(back, fore);										\n"
		//"	case 12: return BlendSoftLight(back, fore);										\n"
		"	case 13: return BlendHardLight(back, fore);										\n"
		"	case 14: return BlendColorDodge(back, fore);									\n"
		"	case 15: return BlendColorBurn(back, fore);										\n"
		"	case 16: return BlendLinearDodge(back, fore);									\n"
		"	case 17: return BlendLinearBurn(back, fore);									\n"
		"	case 18: return BlendLinearLight(back, fore);									\n"
		"	case 19: return BlendVividLight(back, fore);									\n"
		"	case 20: return BlendPinLight(back, fore);										\n"
		"	case 21: return BlendHardMix(back, fore);										\n"
		"	case 22: return BlendReflect(back, fore);										\n"
		"	case 23: return BlendGlow(back, fore);											\n"
		"	case 24: return BlendPhoenix(back, fore);										\n"
		"	case 25: return BlendHue(back, fore);											\n"
		"	case 26: return BlendSaturation(back, fore);									\n"
		"	case 27: return BlendColor(back, fore);											\n"
		"	case 28: return BlendLuminosity(back, fore);									\n"
		"	}																				\n"
		"	return BlendNormal(back, fore);													\n"
		"}																					\n"
		"																					\n"																			  
		"vec4 blend_color(vec4 fore)														\n"
		"{																					\n"
		"   vec4 back = texture2D(background, gl_TexCoord[1].st);							\n"
		"   if(levels)																		\n"
		"		fore.rgb = LevelsControl(fore.rgb, min_input, max_input, gamma, min_output, max_output); \n"
		"	if(csb)																			\n"
		"		fore.rgb = ContrastSaturationBrightness(fore.rgb, brt, sat, con);			\n"
		"   fore.rgb = get_blend_color(back.bgr, fore.rgb);									\n"
		"																					\n"
		"	return vec4(mix(back.rgb, fore.rgb, fore.a), back.a + fore.a);					\n"
		"}																					\n";
	}
		
	std::string get_simple_blend_color_func()
	{
		return 	

		"vec4 blend_color(vec4 fore)														\n"
		"{																					\n"
		"   vec4 back = texture2D(background, gl_TexCoord[1].st);							\n"
		"	return vec4(mix(back.rgb, fore.rgb, fore.a), back.a + fore.a);					\n"
		"}																					\n";
	}

	std::string get_vertex()
	{
		return 

		"void main()																		\n"
		"{																					\n"
		"	gl_TexCoord[0] = gl_MultiTexCoord0;												\n"
		"	gl_TexCoord[1] = gl_MultiTexCoord1;												\n"
		"	gl_FrontColor  = gl_Color;														\n"
		"	gl_Position    = ftransform();													\n"
		"}																					\n";
	}

	std::string get_fragment(bool compability_mode)
	{
		return

		"#version 120																		\n"
		"uniform sampler2D	background;														\n"
		"uniform sampler2D	plane[4];														\n"
		"uniform sampler2D	local_key;														\n"
		"uniform sampler2D	layer_key;														\n"
		"																					\n"
		"uniform bool		is_hd;															\n"
		"uniform bool		has_local_key;													\n"
		"uniform bool		has_layer_key;													\n"
		"uniform int		blend_mode;														\n"
		"uniform int		alpha_mode;														\n"
		"uniform int		pixel_format;													\n"
		"																					\n"
		"uniform bool		levels;															\n"
		"uniform float		min_input;														\n"
		"uniform float		max_input;														\n"
		"uniform float		gamma;															\n"
		"uniform float		min_output;														\n"
		"uniform float		max_output;														\n"
		"																					\n"
		"uniform bool		csb;															\n"
		"uniform float		brt;															\n"
		"uniform float		sat;															\n"
		"uniform float		con;															\n"
		"																					\n"

		+

		(compability_mode ? get_simple_blend_color_func() : get_blend_color_func())
		
		+

		"																					\n"
		"// NOTE: YCbCr, ITU-R, http://www.intersil.com/data/an/an9717.pdf					\n"
		"// TODO: Support for more yuv formats might be needed.								\n"
		"vec4 ycbcra_to_rgba_sd(float y, float cb, float cr, float a)						\n"
		"{																					\n"
		"	cb -= 0.5;																		\n"
		"	cr -= 0.5;																		\n"
		"	y = 1.164*(y-0.0625);															\n"
		"																					\n"
		"	vec4 color;																		\n"
		"	color.r = y + 2.018 * cb;														\n"
		"	color.b = y + 1.596 * cr;														\n"
		"	color.g = y - 0.813 * cr - 0.391 * cb;											\n"
		"	color.a = a;																	\n"
		"																					\n"
		"	return color;																	\n"
		"}																					\n"			
		"																					\n"
		"vec4 ycbcra_to_rgba_hd(float y, float cb, float cr, float a)						\n"
		"{																					\n"
		"	cb -= 0.5;																		\n"
		"	cr -= 0.5;																		\n"
		"	y = 1.164*(y-0.0625);															\n"
		"																					\n"
		"	vec4 color;																		\n"
		"	color.r = y + 2.115 * cb;														\n"
		"	color.b = y + 1.793 * cr;														\n"
		"	color.g = y - 0.534 * cr - 0.213 * cb;											\n"
		"	color.a = a;																	\n"
		"																					\n"
		"	return color;																	\n"
		"}																					\n"			
		"																					\n"			
		"vec4 ycbcra_to_rgba(float y, float cb, float cr, float a)							\n"
		"{																					\n"
		"	if(is_hd)																		\n"
		"		return ycbcra_to_rgba_hd(y, cb, cr, a);										\n"
		"	else																			\n"
		"		return ycbcra_to_rgba_sd(y, cb, cr, a);										\n"
		"}																					\n"
		"																					\n"
		"vec4 get_rgba_color()																\n"
		"{																					\n"
		"	switch(pixel_format)															\n"
		"	{																				\n"
		"	case 0:		//gray																\n"
		"		return vec4(texture2D(plane[0], gl_TexCoord[0].st).rrr, 1.0);				\n"
		"	case 1:		//bgra,																\n"
		"		return texture2D(plane[0], gl_TexCoord[0].st).bgra;							\n"
		"	case 2:		//rgba,																\n"
		"		return texture2D(plane[0], gl_TexCoord[0].st).rgba;							\n"
		"	case 3:		//argb,																\n"
		"		return texture2D(plane[0], gl_TexCoord[0].st).argb;							\n"
		"	case 4:		//abgr,																\n"
		"		return texture2D(plane[0], gl_TexCoord[0].st).gbar;							\n"
		"	case 5:		//ycbcr,															\n"
		"		{																			\n"
		"			float y  = texture2D(plane[0], gl_TexCoord[0].st).r;					\n"
		"			float cb = texture2D(plane[1], gl_TexCoord[0].st).r;					\n"
		"			float cr = texture2D(plane[2], gl_TexCoord[0].st).r;					\n"
		"			return ycbcra_to_rgba(y, cb, cr, 1.0);									\n"
		"		}																			\n"
		"	case 6:		//ycbcra															\n"
		"		{																			\n"
		"			float y  = texture2D(plane[0], gl_TexCoord[0].st).r;					\n"
		"			float cb = texture2D(plane[1], gl_TexCoord[0].st).r;					\n"
		"			float cr = texture2D(plane[2], gl_TexCoord[0].st).r;					\n"
		"			float a  = texture2D(plane[3], gl_TexCoord[0].st).r;					\n"
		"			return ycbcra_to_rgba(y, cb, cr, a);									\n"
		"		}																			\n"
		"	}																				\n"
		"	return vec4(0.0, 0.0, 0.0, 0.0);												\n"
		"}																					\n"
		"																					\n"
		"void main()																		\n"
		"{																					\n"
		"	vec4 color = get_rgba_color();													\n"
		"	if(has_local_key)																\n"
		"		color.a *= texture2D(local_key, gl_TexCoord[1].st).r;						\n"
		"	if(has_layer_key)																\n"
		"		color.a *= texture2D(layer_key, gl_TexCoord[1].st).r;						\n"
		"	gl_FragColor = blend_color(color.bgra * gl_Color);								\n"
		"}																					\n";
	}


	implementation() 
		: last_mode_(core::video_mode::progressive)
		, last_width_(0)
		, last_height_(0)
	{			
	}

	
	void draw(const render_item&							item,
			  const safe_ptr<device_buffer>&				background,
			  const std::shared_ptr<device_buffer>&			local_key,			  
			  const std::shared_ptr<device_buffer>&			layer_key)
	{
		static const double epsilon = 0.001;

		CASPAR_ASSERT(item.pix_desc.planes.size() == item.textures.size());

		if(item.textures.empty())
			return;

		if(item.transform.get_opacity() < epsilon)
			return;

		if(!shader_)
		{
			try
			{
				shader_.reset(new shader(get_vertex(), get_fragment(false)));
			}
			catch(...)
			{
				CASPAR_LOG_CURRENT_EXCEPTION();
				CASPAR_LOG(warning) << "Failed to compile shader. Trying to compile without blend-modes.";
				shader_.reset(new shader(get_vertex(), get_fragment(true)));
			}

			GL(glEnable(GL_TEXTURE_2D));
		}

		if(last_mode_ != item.mode)
		{
			last_mode_ = item.mode;
			
			if(item.mode == core::video_mode::progressive)			
				GL(glDisable(GL_POLYGON_STIPPLE));			
			else			
			{
				GL(glEnable(GL_POLYGON_STIPPLE));			

				if(item.mode == core::video_mode::upper)
					glPolygonStipple(upper_pattern);
				else if(item.mode == core::video_mode::lower)
					glPolygonStipple(lower_pattern);
			}
		}

		if(last_width_ != background->width() || last_height_ != background->height())
		{
			last_width_ = background->width();
			last_height_ = background->height();
			GL(glViewport(0, 0, background->width(), background->height()));
		}

		// Bind textures

		for(size_t n = 0; n < item.textures.size(); ++n)
			item.textures[n]->bind(n);

		if(local_key)
			local_key->bind(4);
		
		if(layer_key)
			layer_key->bind(5);

		background->bind(6);

		// Setup shader

		shader_->use();	

		shader_->set("plane[0]",		0);
		shader_->set("plane[1]",		1);
		shader_->set("plane[2]",		2);
		shader_->set("plane[3]",		3);
		shader_->set("local_key",		4);
		shader_->set("layer_key",		5);
		shader_->set("background",		6);
		shader_->set("is_hd",			item.pix_desc.planes.at(0).height > 700 ? 1 : 0);
		shader_->set("has_local_key",	local_key ? 1 : 0);
		shader_->set("has_layer_key",	layer_key ? 1 : 0);
		shader_->set("blend_mode",		item.transform.get_is_key() ? core::image_transform::blend_mode::normal : item.transform.get_blend_mode());
		shader_->set("alpha_mode",		item.transform.get_alpha_mode());
		shader_->set("pixel_format",	item.pix_desc.pix_fmt);	

		auto levels = item.transform.get_levels();

		if(levels.min_input  > epsilon		||
		   levels.max_input  < 1.0-epsilon	||
		   levels.min_output > epsilon		||
		   levels.max_output < 1.0-epsilon	||
		   std::abs(levels.gamma - 1.0) > epsilon)
		{
			shader_->set("levels", true);	
			shader_->set("min_input", levels.min_input);	
			shader_->set("max_input", levels.max_input);
			shader_->set("min_output", levels.min_output);
			shader_->set("max_output", levels.max_output);
			shader_->set("gamma", levels.gamma);
		}
		else
			shader_->set("levels", false);	

		if(std::abs(item.transform.get_brightness() - 1.0) > epsilon ||
		   std::abs(item.transform.get_saturation() - 1.0) > epsilon ||
		   std::abs(item.transform.get_contrast() - 1.0) > epsilon)
		{
			shader_->set("csb",	true);	
			
			shader_->set("brt", item.transform.get_brightness());	
			shader_->set("sat", item.transform.get_saturation());
			shader_->set("con", item.transform.get_contrast());
		}
		else
			shader_->set("csb",	false);	
		
		// Setup drawing area

		GL(glColor4d(item.transform.get_gain(), item.transform.get_gain(), item.transform.get_gain(), item.transform.get_opacity()));
						
		auto m_p = item.transform.get_clip_translation();
		auto m_s = item.transform.get_clip_scale();
		double w = static_cast<double>(background->width());
		double h = static_cast<double>(background->height());

		GL(glEnable(GL_SCISSOR_TEST));
		GL(glScissor(static_cast<size_t>(m_p[0]*w), static_cast<size_t>(m_p[1]*h), static_cast<size_t>(m_s[0]*w), static_cast<size_t>(m_s[1]*h)));
			
		auto f_p = item.transform.get_fill_translation();
		auto f_s = item.transform.get_fill_scale();
		
		// Draw

		glBegin(GL_QUADS);
			glMultiTexCoord2d(GL_TEXTURE0, 0.0, 0.0); glMultiTexCoord2d(GL_TEXTURE1,  f_p[0]        ,  f_p[1]        );		glVertex2d( f_p[0]        *2.0-1.0,  f_p[1]        *2.0-1.0);
			glMultiTexCoord2d(GL_TEXTURE0, 1.0, 0.0); glMultiTexCoord2d(GL_TEXTURE1, (f_p[0]+f_s[0]),  f_p[1]        );		glVertex2d((f_p[0]+f_s[0])*2.0-1.0,  f_p[1]        *2.0-1.0);
			glMultiTexCoord2d(GL_TEXTURE0, 1.0, 1.0); glMultiTexCoord2d(GL_TEXTURE1, (f_p[0]+f_s[0]), (f_p[1]+f_s[1]));		glVertex2d((f_p[0]+f_s[0])*2.0-1.0, (f_p[1]+f_s[1])*2.0-1.0);
			glMultiTexCoord2d(GL_TEXTURE0, 0.0, 1.0); glMultiTexCoord2d(GL_TEXTURE1,  f_p[0]        , (f_p[1]+f_s[1]));		glVertex2d( f_p[0]        *2.0-1.0, (f_p[1]+f_s[1])*2.0-1.0);
		glEnd();

		GL(glDisable(GL_SCISSOR_TEST));	
	}
};

image_kernel::image_kernel() : impl_(new implementation()){}
void image_kernel::draw(const render_item& item, const safe_ptr<device_buffer>& background, const std::shared_ptr<device_buffer>& local_key, const std::shared_ptr<device_buffer>& layer_key)
{
	impl_->draw(item, background, local_key, layer_key);
}

bool operator==(const render_item& lhs, const render_item& rhs)
{
	return lhs.textures == rhs.textures && lhs.transform == rhs.transform && lhs.tag == rhs.tag && lhs.mode == rhs.mode;
}

}}