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
* Author: Robert Nagy, ronag89@gmail.com
*		  Jan Starzak, jan@ministryofgoodsteps.com
*		  Krzysztof Pyrkosz, pyrkosz@o2.pl
*/

#include "replay_producer.h"

#include "../util/frame_operations.h"
#include "../util/file_operations.h"

#include <algorithm>
#include <sys/stat.h>
#include <math.h>
#include <limits>
#include <boost/assign.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/regex.hpp>
#include <boost/timer.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>
#include <tbb/concurrent_queue.h>

#include <core/frame/draw_frame.h>
#include <core/frame/pixel_format.h>
#include <core/frame/frame_factory.h>
#include <core/frame/frame.h>
#include <common/future.h>
#include <core/video_format.h>
#include <common/env.h>
#include <common/diagnostics/graph.h>

using namespace boost::assign;

namespace caspar { namespace replay {

struct replay_producer : public core::frame_producer
{	
	core::monitor::state					state_;

	const std::wstring						filename_;
	core::draw_frame						frame_;
	core::draw_frame						last_frame_;
	boost::mutex frame_buffer_mutex_;
	std::queue<std::pair<core::draw_frame, uint64_t>>	frame_buffer_;
	bool									frame_stable_;
	mjpeg_file_handle						in_file_;
	mjpeg_file_handle						in_idx_file_;

	spl::shared_ptr<mjpeg_file_header>		index_header_;
	spl::shared_ptr<mjpeg_file_header_ex>	index_header_ex_;
	const spl::shared_ptr<core::frame_factory> frame_factory_;
	tbb::atomic<uint64_t>					framenum_;
	tbb::atomic<uint64_t>					real_framenum_;
	tbb::atomic<uint64_t>					real_last_framenum_;
	tbb::atomic<uint64_t>					first_framenum_;
	tbb::atomic<uint64_t>					last_framenum_;
	tbb::atomic<uint64_t>					result_framenum_;
	tbb::atomic<int>						runstate_;
	uint8_t*								leftovers_;
	int										leftovers_duration_;
	int32_t*								leftovers_audio_;
	uint32_t								leftovers_audio_size_;
	bool									interlaced_;
	int										audio_;
	float									speed_;
	float									abs_speed_;
	int										frame_divider_;
	int										frame_multiplier_;
	bool									reverse_;
	bool									seeked_;
	const spl::shared_ptr<diagnostics::graph> graph_;
	std::thread*							decoder_;

#pragma warning(disable:4244)
	explicit replay_producer(
			const spl::shared_ptr<core::frame_factory>& frame_factory,
			const std::wstring& filename,
			const int sign,
			const unsigned long long start_frame,
			const unsigned long long last_frame,
			const float start_speed,
			const int audio = 0)
		: filename_(filename)
		, frame_(core::draw_frame::empty())
		, last_frame_(core::draw_frame::empty())
		, frame_factory_(frame_factory)
	{
		in_file_ = safe_fopen((filename_).c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE);
		if (in_file_ != NULL)
		{
			uint64_t size = 0;

			in_idx_file_ = safe_fopen((boost::filesystem::wpath(filename_).replace_extension(L".idx").wstring()).c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE);
			if (in_idx_file_ != NULL)
			{
				while (size == 0)
				{
					size = (uint64_t)boost::filesystem::file_size(boost::filesystem::wpath(filename_).replace_extension(L".idx").string());

					if (size > 0) {
						mjpeg_file_header* header;
						mjpeg_file_header_ex* header_ex;
						read_index_header(in_idx_file_, &header);
						index_header_ = spl::shared_ptr<mjpeg_file_header>(header);
						CASPAR_LOG(info) << print() << L" File starts at: " << boost::posix_time::to_iso_wstring(index_header_->begin_timecode);

						if (index_header_->version >= 2)
						{
							read_index_header_ex(in_idx_file_, &header_ex);
							index_header_ex_ = spl::shared_ptr<mjpeg_file_header_ex>(header_ex);

							CASPAR_LOG(info) << print() << L" File contains " << index_header_ex_->audio_channels << L" audio channels.";
						}
						else
						{
							header_ex = new mjpeg_file_header_ex();
							header_ex->audio_channels = 0;
						}

                        /*
						if (index_header_->field_mode == caspar::core::field_mode::progressive)
						{
							interlaced_ = false;
						}
						else
						{
							interlaced_ = true;
						}
                        */
                        interlaced_ = false;

						set_playback_speed(start_speed);
						audio_ = audio;
						result_framenum_ = 0;
						framenum_ = 0;
						last_framenum_ = 0;
						first_framenum_ = 0;
						real_framenum_ = 0;
						runstate_ = 0;
						
						leftovers_ = NULL;
						leftovers_duration_ = 0;
						leftovers_audio_ = NULL;
						leftovers_audio_size_ = 0;

						seeked_ = false;

						if (start_frame > 0)
						{
							if (interlaced_)
								seek(start_frame * 2, sign);
							else
								seek(start_frame, sign);
						}

						if (last_frame > 0)
						{
							last_framenum_ = start_frame + last_frame;
							if (interlaced_)
								last_framenum_ = last_framenum_ * 2;
						}

						graph_->set_color("frame-time", diagnostics::color(0.1f, 1.0f, 0.1f));
						graph_->set_color("underflow", diagnostics::color(0.6f, 0.3f, 0.9f));
						graph_->set_text(print());
						diagnostics::register_graph(graph_);

						decoder_ = new std::thread(
							[&]
							{
								while (runstate_ == 0)
								{
									size_t cur_size = 0;
									{
										boost::lock_guard<boost::mutex> lock(frame_buffer_mutex_);
										cur_size = frame_buffer_.size();
									}

									if (cur_size < REPLAY_PRODUCER_BUFFER_SIZE)
									{
										try
										{
											boost::timer frame_timer;
											real_last_framenum_ = length_index(in_idx_file_);
											// in interlaced mode make sure that number of fields is even
											if (interlaced_ && !(real_last_framenum_ & 1))
												real_last_framenum_--;
											auto frame_pair = render_frame(0);
											{
												boost::lock_guard<boost::mutex> lock(frame_buffer_mutex_);
												frame_buffer_.push(frame_pair);
											}
											update_diag(frame_timer.elapsed()*0.5*index_header_->fps);
										} 
										catch (...)
										{
											CASPAR_LOG(error) << print() << L" Unknown exception in the decoding thread!";
										}
									}
									else
									{
#ifdef _WIN32                                                                          
										Sleep(1000 / (index_header_->fps * 2));
#else
                                                                                usleep(1000000 / (index_header_->fps * 2));
#endif
									}
								}
							}
						);
					}
					else
					{
						std::this_thread::sleep_for(std::chrono::milliseconds(10));
					}
				}
			}
			else
			{
				CASPAR_LOG(error) << print() << L" Index file " << boost::filesystem::wpath(filename_).replace_extension(L".idx").string() << " not found";
				throw file_not_found();
			}
		}
		else
		{
			CASPAR_LOG(error) << print() << L" Video essence file " << filename_ << " not found";
			throw file_not_found();
		}
	}

#pragma warning(default:4244)
	core::draw_frame make_frame(uint8_t* frame_data, uint32_t size, uint32_t width, uint32_t height, bool drop_first_line,
		const int32_t* audio_data = 0, uint32_t audio_data_length = 0)
	{
		core::pixel_format_desc desc;
		desc.format = core::pixel_format::rgb;
		desc.planes.push_back(core::pixel_format_desc::plane((int)width, (int)height, 3));

		auto frame = frame_factory_->create_frame(this, desc);
		if (!drop_first_line)
		{
			std::copy_n(frame_data, size, frame.image_data(0).begin());
		}
		else
		{
			uint32_t line = width * 3;
			std::copy_n(frame_data, size - line, frame.image_data(0).begin() + line);
		}

		/*if (audio_ && audio_data_length > 0)
		{
			core::mutable_audio_buffer samples(audio_data, audio_data + audio_data_length/4);
			frame.audio_data() = samples;
		}*/

		frame_ = core::draw_frame(std::move(frame));

		return frame_;
	}

	std::future<std::wstring> call(const std::vector<std::wstring>& param) override
	{
		return make_ready_future(std::move(do_call(boost::algorithm::join(param, L" "))));
	}

	std::wstring do_call(const std::wstring& param)
	{
		static const boost::wregex speed_exp(L"SPEED\\s+(?<VALUE>[\\d.-]+)", boost::regex::icase);
		static const boost::wregex pause_exp(L"PAUSE", boost::regex::icase);
		static const boost::wregex seek_exp(L"SEEK\\s+(?<SIGN>[\\+\\-\\|])?(?<VALUE>[\\d]+)", boost::regex::icase);
		static const boost::wregex length_exp(L"LENGTH\\s+(?<VALUE>[\\d]+)", boost::regex::icase);
		static const boost::wregex audio_exp(L"AUDIO\\s+(?<VALUE>[\\d]+)", boost::regex::icase);
		
		boost::wsmatch what;
		// PAUSE
		if(boost::regex_match(param, what, pause_exp))
		{
			set_playback_speed(0.0f);
			return L"";
		}
		// SPEED
		if(boost::regex_match(param, what, speed_exp))
		{
			if(!what["VALUE"].str().empty())
			{
				float speed = boost::lexical_cast<float>(what["VALUE"].str());
				set_playback_speed(speed);
			}
			return L"";
		}
		// SEEK
		if(boost::regex_match(param, what, seek_exp))
		{
			int sign = 0;
			if(!what["SIGN"].str().empty())
			{
				if (what["SIGN"].str() == L"+")
					sign = 1;
				else if (what["SIGN"].str() == L"|")
					sign = -2;
				else if (what["SIGN"].str() == L"-")
					sign = -1;
			}
			if(!what["VALUE"].str().empty())
			{
				unsigned long long position = boost::lexical_cast<unsigned long long>(what["VALUE"].str());
				if (interlaced_)
					seek(position * 2, sign);
				else
					seek(position, sign);
			}
			return L"";
		}
		// LENGTH
		if(boost::regex_match(param, what, length_exp))
		{
			if(!what["VALUE"].str().empty())
			{
				long long last_frame = boost::lexical_cast<long long>(what["VALUE"].str());
				if (last_frame == 0)
					last_framenum_ = 0;
				else
				{
					last_framenum_ = first_framenum_ / 2 + last_frame;
					if (interlaced_)
						last_framenum_ = last_framenum_ * 2;
				}
			}
			return L"";
		}
		if(boost::regex_match(param, what, audio_exp))
		{
			if(!what["VALUE"].str().empty())
			{
				audio_ = (boost::lexical_cast<int>(what["VALUE"].str()) == 1 ? 1 : 0);
			}
			return L"";
		}

		BOOST_THROW_EXCEPTION(invalid_argument());
	}

	void seek(unsigned long long frame_pos, int sign)
	{
		if (sign == 0)
		{
			if (frame_pos > real_last_framenum_)
				framenum_ = real_last_framenum_;
			else
				framenum_ = frame_pos;
		}
		else if (sign == -2)
		{
			if (real_last_framenum_ < frame_pos - 4)
				framenum_ = 0;
			else
				framenum_ = real_last_framenum_ - frame_pos - 4;
		}
		else if (sign == -1)
		{
			if (framenum_ < frame_pos)
				framenum_ = 0;
			else
				framenum_ -= frame_pos;
		}
		else if (sign == 1)
		{
			if (framenum_ + frame_pos > real_last_framenum_)
				framenum_ = real_last_framenum_;
			else
				framenum_ += frame_pos;
		}
		if (seek_index(in_idx_file_, framenum_, FILE_BEGIN))
			CASPAR_LOG(error) << L" seek_index@seek " << framenum_;
		first_framenum_ = framenum_;
		seeked_ = true;
	}

	void set_playback_speed(float speed)
	{
		speed_ = speed;
		abs_speed_ = fabs(speed);
		if (speed != 0.0f)
			frame_divider_ = abs((int)(1.0f / speed));
		else 
			frame_divider_ = 0;
		frame_multiplier_ = abs((int)(speed));
		reverse_ = (speed >= 0.0f) ? false : true;
	}

	void update_diag(double elapsed)
	{
		graph_->set_text(print());
		graph_->set_value("frame-time", elapsed*0.5);

		state_["/profiler/time"] = (elapsed / (1.0 / index_header_->fps));
        state_["/file/time"]     = ((interlaced_ ? (long unsigned int)real_framenum_ / 2 : (long unsigned int)real_framenum_) / index_header_->fps);
        state_["/file/frame"]    = static_cast<int32_t>((interlaced_ ? (long unsigned int)real_framenum_ / 2 : (long unsigned int)real_framenum_));
        state_["/file/vframe"]   = static_cast<int32_t>((real_framenum_ - first_framenum_) / (interlaced_ ? 2 : 1));
        state_["/file/fps"]      = index_header_->fps;
        state_["/file/path"]     = filename_;
        state_["/file/speed"]    = speed_;
	}

	void move_to_next_frame()
	{
		int frame_multiplier = frame_multiplier_ > 1 ? frame_multiplier_ : 1;
		bool seek_needed = 0;
		if (reverse_)
		{
			if (framenum_ < frame_multiplier)
				framenum_ = 0;
			else
				framenum_ -= frame_multiplier;
			seek_needed = 1;
		}
		else
		{
			if (framenum_ + frame_multiplier >= real_last_framenum_)
			{
				framenum_ = real_last_framenum_;
				seek_needed = 1;
			}
			else
			{
				framenum_ += frame_multiplier;
				if (frame_multiplier > 1)
					seek_needed = 1;
			}
		}
		if (seek_needed && seek_index(in_idx_file_, framenum_, FILE_BEGIN))
			CASPAR_LOG(error) << L" move_to_next_frame() seek_index(in_idx_file_, " << framenum_ << ", FILE_BEGIN)";
	}

	void sync_to_frame()
	{
		if (interlaced_ && framenum_ % 2 != 0)
		{
			//CASPAR_LOG(warning) << L" Frame number was " << framenum_ << L", syncing to First Field";
			if (framenum_ + 1 >= real_last_framenum_)
			{
				seek_index(in_idx_file_, -1, FILE_CURRENT);
				framenum_--;
			}
			else
			{
				(void)read_index(in_idx_file_);
				framenum_++;
			}
		}
	}

	void proper_interlace(const mmx_uint8_t* field1, const mmx_uint8_t* field2, mmx_uint8_t* dst)
	{
		/*if (index_header_->field_mode == caspar::core::field_mode::lower)
		{
			interlace_fields(field2, field1, dst, index_header_->width, index_header_->height, 3);
		}
		else
		{
			interlace_fields(field1, field2, dst, index_header_->width, index_header_->height, 3);
		}*/
	}

#pragma warning(disable:4244)
	bool slow_motion_playback(uint8_t* result, int32_t** result_audio, uint32_t* result_audio_size)
	{
		uint32_t frame_size = index_header_->width * index_header_->height * 3;
		int filled = 0;
		uint8_t* buffer1 = new uint8_t[frame_size];
		uint8_t* buffer2 = new uint8_t[frame_size];
		black_frame(buffer1, index_header_->width, index_header_->height, 3);
		std::copy_n(buffer1, frame_size, buffer2);

		if (leftovers_ != NULL)
		{
			// result is in buffer2
			blend_images(leftovers_, buffer1, buffer2, index_header_->width, index_header_->height, 3, 64);
			
			*result_audio = new int32_t[leftovers_audio_size_ / 4];
			*result_audio_size = leftovers_audio_size_;
			std::copy_n(leftovers_audio_, leftovers_audio_size_ / 4, *result_audio);

			filled += leftovers_duration_;
			if (filled > 64)
			{
				leftovers_duration_ = filled - 64;
			}
			else
			{
				delete leftovers_;
				leftovers_ = NULL;
				leftovers_duration_ = 0;
				if (leftovers_audio_ != NULL)
					delete leftovers_audio_;
				leftovers_audio_ = NULL;
				leftovers_audio_size_ = 0;
			}
		}

		int frame_duration = ((1 / abs_speed_) * 64.0f);

		while (filled < 64)
		{
			long long field_pos = read_index(in_idx_file_);

			if (field_pos == -1)
			{	// There are no more frames

				delete buffer1;
				delete buffer2;
				if (*result_audio != NULL)
					delete *result_audio;
				*result_audio = NULL;
				*result_audio_size = 0;

				return false;
			}

			move_to_next_frame();

			seek_frame(in_file_, field_pos, FILE_BEGIN);

			mmx_uint8_t* field = NULL;
			uint32_t field_width;
			uint32_t field_height;
			uint32_t audio_size;
			int32_t* audio = NULL;
			(void) read_frame(in_file_, &field_width, &field_height, &field, &audio_size, &audio);

			// Interpolate the field to a full frame if this is a field-based mode
			if (interlaced_)
			{
				field_double(field, buffer1, index_header_->width, index_header_->height, 3);
				delete field;
				field = new uint8_t[frame_size];
				int drop_first_line = (int)(framenum_ % 2 == 0 ? index_header_->width * 3 : 0);
				std::copy_n(buffer1, frame_size - drop_first_line, field + drop_first_line);
			}

			uint8_t level = 0;
			if (filled == 0)
			{
				level = 64;
			}
			else
			{
				level = (uint8_t)(((frame_duration + filled) <= 64 ? frame_duration : 64 - filled));
			}
			blend_images(field, buffer2, buffer1, index_header_->width, index_header_->height, 3, level);

			if (*result_audio != NULL)
				delete *result_audio;
			*result_audio = new int32_t[audio_size / 4];
			*result_audio_size = audio_size;
			std::copy_n(audio, audio_size / 4, *result_audio);

			if (leftovers_ != NULL)
				delete leftovers_;
			if (leftovers_audio_ != NULL)
				delete leftovers_audio_;

			// Store the last frame as leftover
			leftovers_ = field;
			leftovers_audio_ = audio;
			leftovers_audio_size_ = audio_size;

			filled += frame_duration;

			// Switch the buffers around so that the final result is always in buffer2
			uint8_t* temp = buffer2;
			buffer2 = buffer1;
			buffer1 = temp;
		}

		if (filled >= 64)
		{
			leftovers_duration_ = filled - 64;
		}

		std::copy_n(buffer2, frame_size, result);

		delete buffer1;
		delete buffer2;

		return true;
	}
#pragma warning(default:4244)

	// TODO: Move the file operations and frame rendering to a separate function and put the rendered frames to a buffer
	std::pair<core::draw_frame, uint64_t> render_frame(int hints)
	{
		int eof = 0;
		if (!seeked_ && (
			(speed_ == 0.0f) ||                                                  // paused
			(reverse_ && framenum_ == 0) ||                                      // front
			(!reverse_ && framenum_ >= real_last_framenum_) ||                   // end
			(last_framenum_ > 0 && reverse_ && first_framenum_ >= framenum_) ||  // user defined front
			(last_framenum_ > 0 && !reverse_ && last_framenum_ <= framenum_)))   // user defined end
		{
			eof = 1;
			//frame_ = core::basic_frame::eof(); // Uncomment this to keep a steady frame after the length has run through
			if (frame_stable_)
			{
				return std::make_pair(core::draw_frame::still(frame_), framenum_);
			}
		}
		seeked_ = false;

		// IF trickplay is possible 0 - 1.0 || 1.0 - 2.0 || 2.0 - 3.0
		if (((abs_speed_ > 0.0f) && (abs_speed_ < 1.0f)) || ((abs_speed_ > 1.0f) && (abs_speed_ < 2.0f)) || ((abs_speed_ > 2.0f) && (abs_speed_ < 3.0f)))
		{
			uint32_t frame_size = index_header_->width * index_header_->height * 3;
			uint8_t* field1 = new uint8_t[frame_size];
			uint8_t* field2 = NULL;
			int32_t* audio1 = NULL;
			uint32_t audio1_size = 0;
			int32_t* audio2 = NULL;
			uint32_t audio2_size = 0;
			uint8_t* full_frame = NULL;

			if (interlaced_)
			{
				field2 = new uint8_t[frame_size];
				full_frame = new uint8_t[frame_size];
			}

			if (!slow_motion_playback(field1, &audio1, &audio1_size))
			{
				return std::make_pair(frame_, framenum_);
			}
			else
			{
				if (!interlaced_)
				{
					make_frame(field1, frame_size, index_header_->width, index_header_->height, false, audio1, audio1_size);
					frame_stable_ = true;
					delete field1;
					if (audio1 != NULL)
						delete audio1;

					return std::make_pair(frame_, framenum_);
				}

				if (!slow_motion_playback(field2, &audio2, &audio2_size))
				{
					make_frame(field1, frame_size, index_header_->width, index_header_->height, false, audio1, audio1_size);
					frame_stable_ = true;
					delete field1;
					delete field2;
					delete full_frame;
					if (audio1 != NULL)
						delete audio1;
					if (audio2 != NULL)
						delete audio2;

					return std::make_pair(frame_, framenum_);
				}
				else
				{
					int32_t* audio = new int32_t[(audio1_size + audio2_size) / 4];
					std::copy_n(audio1, audio1_size / 4, audio);
					std::copy_n(audio2, audio2_size / 4, audio + (audio1_size / 4));

					interlace_frames(field1, field2, full_frame, index_header_->width, index_header_->height, 3);
					make_frame(full_frame, frame_size, index_header_->width, index_header_->height, false, audio, audio1_size + audio2_size);
					frame_stable_ = false;
					delete field1;
					delete field2;
					delete full_frame;
					delete audio;
					if (audio1 != NULL)
						delete audio1;
					if (audio2 != NULL)
						delete audio2;

					return std::make_pair(frame_, framenum_);
				}
			}
		}

		if (leftovers_ != NULL)
		{
			delete leftovers_;
			leftovers_ = NULL;
			if (leftovers_audio_ != NULL)
				delete leftovers_audio_;
			leftovers_audio_ = NULL;
			leftovers_audio_size_ = 0;
		}

		// ELSE
		if (abs_speed_ >= 1.0f)
			sync_to_frame();

		long long field1_pos = read_index(in_idx_file_);

		if (field1_pos == -1)
		{	// There are no more frames
			return std::make_pair(frame_, framenum_);
		}

		move_to_next_frame(); // CHECK THIS

		seek_frame(in_file_, field1_pos, FILE_BEGIN);

		mmx_uint8_t* field1 = NULL;
		mmx_uint8_t* field2 = NULL;
		mmx_uint8_t* full_frame = NULL;
		int32_t* audio1 = NULL;
		int32_t* audio2 = NULL;
		int32_t* audio = NULL;
		uint32_t field1_width;
		uint32_t field1_height;
		uint32_t audio1_size;
		uint32_t audio2_size;
		uint32_t field1_size = read_frame(in_file_, &field1_width, &field1_height, &field1, &audio1_size, &audio1);
		if (field1 == nullptr)
		{
			delete audio1;
			return std::make_pair(frame_, framenum_);
		}

		if (!interlaced_)
		{
			make_frame(field1, field1_size, index_header_->width, index_header_->height, false, audio1, audio1_size);
			frame_stable_ = true;

			delete field1;
			delete audio1;

			return std::make_pair(frame_, framenum_);
		}

		if ((speed_ == 0.0f || eof) && interlaced_)
		{
			mmx_uint8_t* full_frame1 = new mmx_uint8_t[field1_size * 2];

			field_double(field1, full_frame1, index_header_->width, index_header_->height, 3);
			make_frame(full_frame1, field1_size * 2, index_header_->width, index_header_->height, false);
			frame_stable_ = true;

			delete field1;
			delete audio1;
			delete full_frame1;

			return std::make_pair(frame_, framenum_);
		}

		long long field2_pos = read_index(in_idx_file_);

		move_to_next_frame();

		seek_frame(in_file_, field2_pos, FILE_BEGIN);

		uint32_t field2_size = read_frame(in_file_, &field1_width, &field1_height, &field2, &audio2_size, &audio2);
		if (field2 == nullptr)
		{
			delete field1;
			delete audio1;
			delete audio2;
			return std::make_pair(frame_, framenum_);
		}

		audio = new int32_t[(audio1_size + audio2_size)/4];
		memcpy(audio, audio1, audio1_size);
		memcpy(audio + audio1_size/4, audio2, audio2_size);
		delete audio1;
		delete audio2;

		full_frame = new mmx_uint8_t[field1_size + field2_size];

		proper_interlace(field1, field2, full_frame);
		
		make_frame(full_frame, field1_size + field2_size, index_header_->width, index_header_->height, false, audio, audio1_size + audio2_size);
		frame_stable_ = false;

		if (field1 != NULL)
			delete field1;
		if (field2 != NULL)
			delete field2;
		delete audio;
		delete full_frame;

		return std::make_pair(frame_, framenum_);
	}

	core::draw_frame receive_impl(int nb_samples) override
	{
		boost::lock_guard<boost::mutex> lock(frame_buffer_mutex_);

		if (frame_buffer_.size() < 1)
		{
			result_framenum_++;

			graph_->set_tag(caspar::diagnostics::tag_severity::WARNING, "underflow");
			return last_frame_;	// repeat last frame
		}

		auto frame = last_frame_= frame_buffer_.front().first;
		real_framenum_ = frame_buffer_.front().second;
		frame_buffer_.pop();

		result_framenum_++;

		return frame;
	}
		
	core::draw_frame last_frame() override
	{
		return core::draw_frame::still(last_frame_);
	}

#pragma warning (disable: 4244)
	virtual uint32_t nb_frames() const override
	{
		if (last_framenum_ > 0)
		{
			return (uint32_t)((interlaced_ ? ((last_framenum_ - first_framenum_) / 2) : (last_framenum_ - first_framenum_)) / speed_);
		}
		return std::numeric_limits<uint32_t>::max();
	}
#pragma warning (default: 4244)

	virtual std::wstring print() const override
	{
		return L"replay_producer[" + filename_ + L"|" + boost::lexical_cast<std::wstring>(interlaced_ ? (long unsigned int)real_framenum_ / 2 : (long unsigned int)real_framenum_)
			 + L"|" + boost::lexical_cast<std::wstring>(speed_)
			 + L"]";
	}

    /*
    virtual boost::property_tree::wptree info() const override
	{
		boost::property_tree::wptree info;
		info.add(L"type", L"replay-producer");
		info.add(L"filename",			filename_);
		info.add(L"play-head",			(interlaced_ ? (long unsigned int)real_framenum_ / 2 : (long unsigned int)real_framenum_)); // deprecated
		info.add(L"start-timecode",		boost::posix_time::to_iso_wstring(index_header_->begin_timecode));
		info.add(L"speed",				speed_);
		info.add(L"width",				index_header_->width);
		info.add(L"height",				index_header_->height);
		info.add(L"progressive",		interlaced_);
		info.add(L"audio",				(audio_ ? true : false));
		info.add(L"fps",				index_header_->fps);
		info.add(L"loop",				false); // not implemented
		info.add(L"frame-number",		static_cast<int32_t>((real_framenum_ - first_framenum_) / (interlaced_ ? 2 : 1)));
		info.add(L"nb-frames",			static_cast<int32_t>(((last_framenum_ > 0 ? last_framenum_ : real_last_framenum_)  - first_framenum_) / (interlaced_ ? 2 : 1)));
		info.add(L"file-frame-number",	static_cast<int32_t>((interlaced_ ? (long unsigned int)real_framenum_ / 2 : (long unsigned int)real_framenum_)));
		info.add(L"file-nb-frames",		static_cast<int32_t>(real_last_framenum_ / (interlaced_ ? 2 : 1)));
		return info;
	}*/

	~replay_producer()
	{
		runstate_ = 1;
		if (decoder_ != NULL)
		{
			if (decoder_->joinable())
			{
				decoder_->join();
			}
		}

		if (in_file_ != NULL)
			safe_fclose(in_file_);

		if (in_idx_file_ != NULL)
			safe_fclose(in_idx_file_);
	}

	std::wstring name() const override
	{
		return L"replay";
	}
};

spl::shared_ptr<core::frame_producer> create_producer(const core::frame_producer_dependencies& dependencies, const std::vector<std::wstring>& params)
{
    if (!boost::iequals(params.at(0), "[REPLAY]"))
        return core::frame_producer::empty();

	static const std::vector<std::wstring> extensions = list_of(L"mav");
    std::wstring filename = env::media_folder() + params.at(1);
	
	auto ext = std::find_if(extensions.begin(), extensions.end(), [&](const std::wstring& ex) -> bool
	{					
		return boost::filesystem::is_regular_file(boost::filesystem::wpath(filename).replace_extension(ex));
	});

	int sign = 0;
	int audio = 0;
	unsigned long long start_frame = 0;
	unsigned long long last_frame = 0;
	float start_speed = 1.0f;
	if (params.size() >= 3)
	{
		for (uint16_t i=0; i<params.size(); i++)
		{
			if (boost::iequals(params[i], L"SEEK"))
			{
				static const boost::wregex seek_exp(L"(?<SIGN>[\\|])?(?<VALUE>[\\d]+)", boost::regex::icase);
				boost::wsmatch what;
				if(boost::regex_match(params[i+1], what, seek_exp))
				{
				
					if(!what["SIGN"].str().empty())
					{
						if (what["SIGN"].str() == L"|")
							sign = -2;
						else
							sign = 0;
					}
					if(!what["VALUE"].str().empty())
					{
						start_frame = boost::lexical_cast<unsigned long long>(what["VALUE"].str());
					}
				}
			}
			else if (boost::iequals(params[i], L"SPEED"))
			{
				static const boost::wregex speed_exp(L"(?<VALUE>[\\d.-]+)", boost::regex::icase);
				boost::wsmatch what;
				if (boost::regex_match(params[i+1], what, speed_exp))
				{
					if (!what["VALUE"].str().empty())
					{
						start_speed = boost::lexical_cast<float>(what["VALUE"].str());
					}
				}
			}
			else if (boost::iequals(params[i], L"LENGTH"))
			{
				static const boost::wregex length_exp(L"(?<VALUE>[\\d]+)", boost::regex::icase);
				boost::wsmatch what;
				if (boost::regex_match(params[i+1], what, length_exp))
				{
					if (!what["VALUE"].str().empty())
					{
						last_frame = boost::lexical_cast<unsigned long long>(what["VALUE"].str());
					}
				}
			}
			else if (boost::iequals(params[i], L"AUDIO"))
			{
				static const boost::wregex audio_exp(L"(?<VALUE>[\\d]+)", boost::regex::icase);
				boost::wsmatch what;
				if (boost::regex_match(params[i+1], what, audio_exp))
				{
					if (!what["VALUE"].str().empty())
					{
						audio = (boost::lexical_cast<int>(what["VALUE"].str()) == 1 ? 1 : 0);
					}
				}
			}
		}
	}

	//return spl::make_shared_ptr(std::make_shared<replay_producer>(dependencies.frame_factory, filename + L"." + *ext, sign, start_frame, last_frame, start_speed, audio));
	return spl::make_shared<replay_producer>(dependencies.frame_factory, filename + L"." + *ext, sign, start_frame, last_frame, start_speed, audio);
}

}}
