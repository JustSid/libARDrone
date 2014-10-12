//
//  ARControlService.h
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

#ifndef __libARDrone__ARControlService__
#define __libARDrone__ARControlService__

#include <mutex>
#include <string>
#include <sstream>
#include <vector>

#include "ARService.h"
#include "ARATService.h"
#include "ARVector.h"

namespace AR
{
	class Navdata;
	class ControlService : public Service
	{
	public:
		enum class FlyState
		{
			Landed,
			TakingOff,
			Landing,
			Flying
		};
		
		ControlService(Drone *drone);
		~ControlService() override;

		FlyState GetFlyState();
		
		bool HasEmergency();
		bool IsHovering();
		
		void Ftrim();
		void TakeOff();
		void Land();
		void Hover();
		void SetEmergency(bool emergency);
		
		void SetDirection(const Vector3 &direction);
		void SetAngularSpeed(float speed);
		
		Vector3 GetDirection();
		float GetAngularSpeed();
		
	protected:
		void Tick(uint32_t reason) final;
		State ConnectInternal() final;
		void DisconnectInternal() final;
		
	private:
		void ProcessNavdata(Navdata *data);
		void TickNow();
		
		std::mutex _mutex;
		std::chrono::steady_clock::time_point _lastPackage;
		ATService *_atService;
		
		uint32_t _droneState;
		
		bool _hasNavdata;
		bool _wantsTakeOff;
		bool _wantsFtrim;
		bool _emergency;
		
		FlyState _flyState;
		
		Vector3 _direction;
		float _angularSpeed;
		bool _hover;
	};
}

#endif /* defined(__libARDrone__ARControlService__) */
