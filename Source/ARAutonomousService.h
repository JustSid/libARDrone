//
//  ARAutonomousService.h
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

#ifndef __libARDrone__ARAutonomousService__
#define __libARDrone__ARAutonomousService__

#include <chrono>
#include "ARService.h"
#include "ARNavdataService.h"
#include "ARATService.h"
#include "ARLocation.h"

namespace AR
{
	class ControlService;
	class AutonomousService : public Service
	{
	public:
		static constexpr float kDirectionAny = HUGE_VALF;
		
		enum class AutonomyState
		{
			Stopped,
			Bootstrapping,
			WaitingForGPS,
			Running,
			Finished
		};
		
		AutonomousService(Drone *drone);
		~AutonomousService();
		
		void Start();
		void Stop();
		
		void AddWaypoint(const Location &location, float direction, uint32_t altitude);
		
		AutonomyState GetAutonomyState() const { return _state; }
	
	protected:
		State ConnectInternal() override;
		void Tick(uint32_t reason) override;
		
	private:
		void ConsumeNavdata(Navdata *navhdata);
		void SetCooldown(std::chrono::milliseconds duration);
		
		struct Command
		{
			enum class Type
			{
				MoveTo,
				Wait,
				Land
			};
			
			void *data;
		};
		
		struct Waypoint
		{
			Location target;
			float direction;
			uint32_t altitude;
		};
		
		ControlService *_control;
		ATService *_atService;
		
		std::mutex _mutex;
		std::atomic<bool> _wantsRunning;
		
		std::atomic<AutonomyState> _state;
		
		bool _isFlying;
		bool _isCalibrated;
		bool _isTrimmed;
		
		Navdata *_navdata;
		bool _freshNavdata;
		
		std::chrono::steady_clock::time_point _cooldown;
	};
}

#endif /* defined(__libARDrone__ARAutonomousService__) */
