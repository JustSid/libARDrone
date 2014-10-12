//
//  ARVideoService.h
//  libARDronde
//
//  Created by Sidney Just
//  Copyright (c) 2014 by Sidney Just
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
//  documentation files (the "Software"), to deal in the Software without restriction, including without limitation
//  the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
//  and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
//  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
//  PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
//  FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
//  ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

#ifndef __libARDrone__ARVideoService__
#define __libARDrone__ARVideoService__

#include <vector>
#include <functional>
#include "ARService.h"
#include "ARSocket.h"

namespace AR
{
	struct PAVE
	{
		uint8_t  signature[4];
		uint8_t  version;
		uint8_t  video_codec;
		uint16_t header_size;
		uint32_t payload_size;             /* Amount of data following this PaVE */
		uint16_t encoded_stream_width;     /* ex: 640 */
		uint16_t encoded_stream_height;    /* ex: 368 */
		uint16_t display_width;            /* ex: 640 */
		uint16_t display_height;           /* ex: 360 */
		uint32_t frame_number;             /* frame position inside the current stream */
		uint32_t timestamp;                /* in milliseconds */
		uint8_t  total_chuncks;            /* number of UDP packets containing the current decodable payload */
		uint8_t  chunck_index;             /* position of the packet - first chunk is #0 */
		uint8_t  frame_type;               /* I-frame, P-frame */
		uint8_t  control;                  /* Special commands like end-of-stream or advertised frames */
		uint32_t stream_byte_position_lw;  /* Byte position of the current payload in the encoded stream  - lower 32-bit word */
		uint32_t stream_byte_position_uw;  /* Byte position of the current payload in the encoded stream  - upper 32-bit word */
		uint16_t stream_id;                /* This ID indentifies packets that should be recorded together */
		uint8_t  total_slices;             /* number of slices composing the current frame */
		uint8_t  slice_index ;             /* position of the current slice in the frame */
		uint8_t  header1_size;             /* H.264 only : size of SPS inside payload - no SPS present if value is zero */
		uint8_t  header2_size;             /* H.264 only : size of PPS inside payload - no PPS present if value is zero */
		uint8_t  reserved2[2];             /* Padding to align on 48 bytes */
		uint32_t advertised_size;          /* Size of frames announced as advertised frames */
		uint8_t  reserved3[12];            /* Padding to align on 64 bytes */
	};
	
	typedef enum
	{
		PAVEVideoCodecUnknown,
		PAVEVideoCodecVLIB,
		PAVEVideoCodecP264,
		PAVEVideoCodecMPEG4Visual,
		PAVEVideoCodecMPEG4AVC
	} PAVEVideoCodec;
	
	typedef enum
	{
		PAVEFrameTypeUnknown,
		PAVEFrameTypeIDRFrame,
		PAVEFrameTypeIFrame,
		PAVEFrameTypePFrame
	} PAVEFrameType;
	
	typedef enum
	{
		PAVEControlTypeData = 0,
		PAVEControlTypeAdvertisement = (1 << 0),
		PAVEControlTypeLastFrame = (1 << 1)
	} PAVEControlType;
	
	class VideoService : public Service
	{
	public:
		VideoService(Drone *drone);
		~VideoService() override;
		
		void AddVideoDataSubscriber(std::function<void (PAVE *, const uint8_t *)> &&callback, void *token);
		void RemoveVideoDataSubscriber(void *token);
		
	protected:
		void Tick(uint32_t reason) override;
		State ConnectInternal() override;
		void DisconnectInternal() override;
		
	private:
		void HandleFrame(std::vector<uint8_t> &frame);
		size_t FindPaveHeader(size_t offset);
		
		std::mutex _mutex;
		std::vector<std::pair<std::function<void (PAVE *, const uint8_t *)>, void *>> _subscribers;
		
		Socket *_socket;
		uint8_t *_buffer;
		
		size_t _bufferSize;
		size_t _bufferOffset;
		size_t _frameBegin;
	};
}

#endif /* defined(__libARDrone__ARVideoService__) */
