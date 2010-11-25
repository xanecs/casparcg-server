#include "../../StdAfx.h"

#include "image_scroll_producer.h"

#include "image_loader.h"

#include "../../processor/frame.h"
#include "../../processor/composite_frame.h"
#include "../../format/video_format.h"
#include "../../processor/frame_processor_device.h"
#include "../../server.h"

#include <tbb/parallel_for.h>
#include <tbb/parallel_invoke.h>
#include <tbb/scalable_allocator.h>

#include <boost/assign.hpp>
#include <boost/algorithm/string/case_conv.hpp>

using namespace boost::assign;

namespace caspar { namespace core { namespace image{

enum direction
{
	Up, Down, Left, Right
};

struct image_scroll_producer : public frame_producer
{
	static const int DEFAULT_SCROLL_SPEED = 50;

	image_scroll_producer(const std::wstring& filename, const std::vector<std::wstring>& params) 
		: speed_(image_scroll_producer::DEFAULT_SCROLL_SPEED), direction_(direction::Up), offset_(0), filename_(filename)
	{
		auto pos = filename.find_last_of(L'_');
		if(pos != std::wstring::npos && pos + 1 < filename.size())
		{
			std::wstring speedStr = filename.substr(pos + 1);
			pos = speedStr.find_first_of(L'.');
			if(pos != std::wstring::npos)
			{
				speedStr = speedStr.substr(0, pos);		
				speed_ = common::lexical_cast_or_default<int>(speedStr, image_scroll_producer::DEFAULT_SCROLL_SPEED);
			}
		}

		loop_ = std::find(params.begin(), params.end(), L"LOOP") != params.end();
	}

	void load_and_pad_image(const std::wstring& filename)
	{
		auto pBitmap = load_image(filename);

		size_t width = FreeImage_GetWidth(pBitmap.get());
		size_t height = FreeImage_GetHeight(pBitmap.get());

		image_width_ = std::max(width, format_desc_.width);
		image_height_ = std::max(height, format_desc_.height);

		image_ = std::shared_ptr<unsigned char>(static_cast<unsigned char*>(scalable_aligned_malloc(image_width_*image_height_*4, 16)));
		memset(image_.get(), 0, image_width_*image_height_*4);

		unsigned char* pBits = FreeImage_GetBits(pBitmap.get());
		
		for (size_t i = 0; i < height; ++i)
			memcpy(&image_.get()[i * image_width_ * 4], &pBits[i* width * 4], width * 4);
	}

	frame_ptr do_render_frame()
	{
		frame_ptr frame = frame_processor_->create_frame(format_desc_.width, format_desc_.height);
		memset(frame->data(), 0, format_desc_.size);

		const int delta_x = direction_ == direction::Left ? speed_ : -speed_;
		const int delta_y = direction_ == direction::Up ? speed_ : -speed_;

		unsigned char* frame_data = frame->data();
		unsigned char* image_data = image_.get();
	
		if (direction_ == direction::Up || direction_ == direction::Down)
		{
			tbb::parallel_for(static_cast<size_t>(0), format_desc_.height, static_cast<size_t>(1), [&](size_t i)
			{
				int srcRow = i + offset_;
				int dstInxex = i * format_desc_.width * 4;
				int srcIndex = srcRow * format_desc_.width * 4;
				int size = format_desc_.width * 4;

				memcpy(&frame_data[dstInxex], &image_data[srcIndex], size);	
			});				
			
			offset_ += delta_y;
		}
		else
		{
			tbb::parallel_for(static_cast<size_t>(0), format_desc_.height, static_cast<size_t>(1), [&](size_t i)
			{
				int correctOffset = offset_;
				int dstIndex = i * format_desc_.width * 4;
				int srcIndex = (i * image_width_ + correctOffset) * 4;
			
				int stopOffset = std::min<int>(correctOffset + format_desc_ .width, image_width_);
				int size = (stopOffset - correctOffset) * 4;

				memcpy(&frame_data[dstIndex], &image_data[srcIndex], size);
			});

			offset_ += delta_x;
		}

		return frame;
	}
		
	frame_ptr render_frame()
	{		
		if(format_desc_.update != video_update_format::progressive)				
		{
			frame_ptr frame1;
			frame_ptr frame2;
			tbb::parallel_invoke([&]{ frame1 = do_render_frame(); }, [&]{ frame2 = do_render_frame(); });
			return composite_frame::interlace(frame1, frame2, format_desc_.update);
		}			

		return render_frame();	
	}
	
	void initialize(const frame_processor_device_ptr& frame_processor)
	{
		frame_processor_ = frame_processor;
		format_desc_ = frame_processor_->get_video_format_desc();
				
		if(image_width_ - format_desc_.width > image_height_ - format_desc_.height)
			direction_ = speed_ < 0 ? direction::Right : direction::Left;
		else
			direction_ = speed_ < 0 ? direction::Down : direction::Up;

		if (direction_ == direction::Down)
			offset_ = image_height_ - format_desc_.height;
		else if (direction_ == direction::Right)
			offset_ = image_width_ - format_desc_.width;

		speed_ = static_cast<int>(abs(static_cast<double>(speed_) / format_desc_.fps));
		
		load_and_pad_image(filename_);
	}

	const video_format_desc& get_video_format_desc() const { return format_desc_; } 
	
	int image_width_;
	int image_height_;
	int speed_;
	int offset_;
	direction direction_;

	tbb::atomic<bool> loop_;
	std::shared_ptr<unsigned char> image_;
	video_format_desc format_desc_;

	std::wstring filename_;

	frame_processor_device_ptr frame_processor_;
};

frame_producer_ptr create_image_scroll_producer(const std::vector<std::wstring>& params)
{
	static const std::vector<std::wstring> extensions = list_of(L"spng")(L"stga")(L"sbmp")(L"sjpg")(L"sjpeg");
	std::wstring filename = server::media_folder() + L"\\" + params[0];
	
	auto ext = std::find_if(extensions.begin(), extensions.end(), [&](const std::wstring& ex) -> bool
		{					
			return boost::filesystem::is_regular_file(boost::filesystem::wpath(filename).replace_extension(ex));
		});

	if(ext == extensions.end())
		return nullptr;

	return std::make_shared<image_scroll_producer>(filename + L"." + *ext, params);
}

}}}