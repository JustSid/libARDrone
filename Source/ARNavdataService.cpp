//
//  ARNavdataService.cpp
//  libARDrone
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

#define __ARNavdataServiceCPP 1 // So we pull in the ARNavdataCopyOption() macro

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
		} bridge;
		
		if(!_opened)
		{
			Open();
			_opened = true;
		}
		
		size_t received = 0;
		Socket::Result result = _socket->Receive(bridge.buffer, 4096, &received);
		
		if(result == Socket::Result::Timeout)
		{
			Open();
			return;
		}
		
		if(result == Socket::Result::Success && bridge.data.header == 0x55667788)
		{
			SetState(State::Connected);
			
			Navdata *navdata = new Navdata();
			
			navdata->state    = bridge.data.state;
			navdata->sequence = bridge.data.sequence;
			navdata->vision   = bridge.data.vision;
			
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
			
			uint8_t *temp = bridge.buffer + sizeof(__NavdataRaw);
			size_t left   = received - sizeof(__NavdataRaw);
			
			bool checksumVerified = false;
			
			while(left > sizeof(NavdataOption))
			{
				NavdataOption *option = reinterpret_cast<NavdataOption *>(temp);
				
				switch(option->tag)
				{
					ARNavdataCopyOption(Demo)
					ARNavdataCopyOption(Time)
					ARNavdataCopyOption(RawMeasures)
					ARNavdataCopyOption(PhysMeasures)
					ARNavdataCopyOption(GyrosOffsets)
					ARNavdataCopyOption(Trims)
					ARNavdataCopyOption(RCReferences)
					ARNavdataCopyOption(PWM)
					ARNavdataCopyOption(Altitude)
					ARNavdataCopyOption(VisionRaw)
					// ARNavdataCopyOption(VisionOf)
					ARNavdataCopyOption(Vision)
					ARNavdataCopyOption(VisionPerf)
					ARNavdataCopyOption(ADCDataFrame)
					ARNavdataCopyOption(PressureRaw)
					ARNavdataCopyOption(Magneto)
					ARNavdataCopyOption(Wind)
					ARNavdataCopyOption(KalmanPressure)
					ARNavdataCopyOption(Wifi)
					ARNavdataCopyOption(GPS)
					
					case NavdataTag::Checksum:
					{
						NavdataOptionChecksum *data = static_cast<NavdataOptionChecksum *>(option);
						navdata->options.emplace_back(new NavdataOptionChecksum(*data));
						
						uint32_t checksum = CalculateChecksum(bridge.buffer, received - sizeof(NavdataOptionChecksum));
						
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
