//
//  ARNavdataService.h
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

#ifndef __libARDrone__ARNavdataService__
#define __libARDrone__ARNavdataService__

#include <mutex>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <functional>

#include "ARService.h"
#include "ARSocket.h"
#include "ARNavdataOptions.h"

namespace AR
{
	template<class ...Args>
	uint32_t NavdataOptions(Args... args)
	{
		std::initializer_list<uint32_t> list { static_cast<uint32_t>(args)... };
		uint32_t result = 0;
		
		for(uint32_t val : list)
			result |= (UINT32_C(1) << val);
		
		return result;
	}
	
	
#define ARNavdataCopyOption(name) \
	case NavdataTag::name: \
	{ \
		NavdataOption##name *data = static_cast<NavdataOption##name *>(option); \
		navdata->options.emplace_back(new NavdataOption##name(*data)); \
	}
	
	struct Navdata
	{
		uint32_t state;
		uint32_t sequence;
		uint32_t vision;
		
		template<class T>
		T *GetOptionWithTag(NavdataTag tag)
		{
			for(std::unique_ptr<NavdataOption> &option : options)
			{
				if(option->tag == tag)
					return static_cast<T *>(option.get());
			}
			
			return nullptr;
		}
		
		Navdata *CopyWithTags(uint32_t tags)
		{
			Navdata *navdata = new Navdata();
			navdata->state = state;
			navdata->sequence = sequence;
			navdata->vision = vision;
			
			for(auto &temp : options)
			{
				NavdataOption *option = static_cast<NavdataOption *>(temp.get());
				
				if(!(tags & (UINT32_C(1) << static_cast<uint32_t>(option->tag))))
					continue;
				
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
					
					default:
						break;
				}
			}
			
			return navdata;
		}
		
		std::vector<std::unique_ptr<NavdataOption>> options;
	};
	
	class NavdataService : public Service
	{
	public:
		NavdataService(Drone *drone, const std::string &address);
		~NavdataService() override;
		
	protected:
		void Tick(uint32_t reason) final;
		
		State ConnectInternal() final;
		void DisconnectInternal() final;
		
	private:
		void Open();
		uint32_t CalculateChecksum(const uint8_t *data, size_t size) const;
		
		Socket *_socket;
		
		bool _opened;		
		uint32_t _sequence;
	};
	
// Just make sure we don't pull this into any scope
#ifndef __ARNavdataServiceCPP
#undef ARNavdataCopyOption
#endif
}

#endif /* defined(__libARDrone__ARNavdataService__) */
