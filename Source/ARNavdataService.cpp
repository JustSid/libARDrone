//
//  ARNavdataService.cpp
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

#include "ARNavdataService.h"
#include "ARDrone.h"

namespace AR
{
	struct __NavdataRaw
	{
		uint32_t header;
		uint32_t state;
		uint32_t sequence;
		int32_t vision;
		
		uint8_t data[];
	} __attribute__((packed));
	
	NavdataService::NavdataService(Drone *drone, const std::string &address) :
		Service(drone, "Navdata"),
		_socket(new Socket(address, 5554))
	{}
	
	NavdataService::~NavdataService()
	{
		delete _socket;
	}
	
	
	Service::State NavdataService::ConnectInternal()
	{
		if(!_socket->Connect())
			return State::Disconnected;
		
		_opened = false;
		return State::Connecting;
	}
	
	void NavdataService::DisconnectInternal()
	{
		_socket->Disconnect();
	}
	
	
	void NavdataService::Open()
	{
		int32_t flag = 1;
		
		_socket->Send(&flag, sizeof(flag));
		_sequence = 0;
	}
	
	uint32_t NavdataService::CalculateChecksum(const uint8_t *data, size_t size) const
	{
		uint32_t checksum = 0;
		
		for(size_t i = 0; i < size; i ++)
		{
			checksum += data[i];
		}
		
		return checksum;
	}
	
	void NavdataService::Tick(uint32_t reason)
	{
		union
		{
			uint8_t buffer[4096];
			__NavdataRaw data;
		} x;
		
		if(!_opened)
		{
			Open();
			_opened = true;
		}
		
		size_t received = 0;
		Socket::Result result = _socket->Receive(x.buffer, 4096, &received);
		
		if(result == Socket::Result::Timeout)
		{
			Open();
			return;
		}
		
		if(result == Socket::Result::Success && x.data.header == 0x55667788)
		{
			SetState(State::Connected);
			
			Navdata *navdata = new Navdata();
			
			navdata->state    = x.data.state;
			navdata->sequence = x.data.sequence;
			navdata->vision   = x.data.vision;
			
			if(navdata->sequence <= _sequence)
			{
				// This is a missed package that we received out of order
				delete navdata;
				return;
			}
			
			_sequence = navdata->sequence;
			
			
			if(navdata->state & ARDRONE_NAVDATA_BOOTSTRAP)
			{
				GetDrone()->PublishNavdata(navdata);
				return;
			}
			
			uint8_t *temp = x.buffer + sizeof(__NavdataRaw);
			size_t left   = received - sizeof(__NavdataRaw);
			
			bool checksumVerified = false;
			
			while(left > sizeof(NavdataOption))
			{
				NavdataOption *option = reinterpret_cast<NavdataOption *>(temp);
				
				switch(option->tag)
				{
					case NavdataTag::Demo:
					{
						NavdataOptionDemo *data = static_cast<NavdataOptionDemo *>(option);
						navdata->options.emplace_back(new NavdataOptionDemo(*data));
						break;
					}
						
					case NavdataTag::RawMeasures:
					{
						NavdataOptionRawMeasures *data = static_cast<NavdataOptionRawMeasures *>(option);
						navdata->options.emplace_back(new NavdataOptionRawMeasures(*data));
						break;
					}
						
					case NavdataTag::Magneto:
					{
						NavdataOptionMagneto *data = static_cast<NavdataOptionMagneto *>(option);
						navdata->options.emplace_back(new NavdataOptionMagneto(*data));
						break;
					}
						
					case NavdataTag::GPS:
					{
						NavdataOptionGPS *data = static_cast<NavdataOptionGPS *>(option);
						navdata->options.emplace_back(new NavdataOptionGPS(*data));
						break;
					}
						
					case NavdataTag::Checksum:
					{
						NavdataOptionChecksum *data = static_cast<NavdataOptionChecksum *>(option);
						navdata->options.emplace_back(new NavdataOptionChecksum(*data));
						
						uint32_t checksum = CalculateChecksum(x.buffer, received - sizeof(NavdataOptionChecksum));
						
						if(checksum == data->checksum)
							checksumVerified = true;
					}
						
					default:
						break;
				}
				
				temp += option->size;
				left -= option->size;
			}
			
			if(!checksumVerified)
			{
				std::cout << "Checksum verification failed" << std::endl;
				return;
			}
			
			GetDrone()->PublishNavdata(navdata);
		}
	}
}
