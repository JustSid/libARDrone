//
//  ARControlService.cpp
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

#include "ARControlService.h"
#include "ARDrone.h"

namespace AR
{
	ControlService::ControlService(Drone *drone) :
		Service(drone, "Control"),
		_wantsTakeOff(false),
		_emergency(false),
		_wantsFtrim(false),
		_hasNavdata(false),
		_hover(true),
		_flyState(FlyState::Landed)
	{}
	
	ControlService::~ControlService()
	{}
	
	
	Service::State ControlService::ConnectInternal()
	{
		_atService = GetDrone()->GetService<ATService>("AT");
		GetDrone()->AddNavdataSubscriber(std::bind(&ControlService::ProcessNavdata, this, std::placeholders::_1), this);
		
		_direction    = Vector3(0.0f);
		_angularSpeed = 0.0f;
		
		return Service::State::Connected;
	}
	void ControlService::DisconnectInternal()
	{
		GetDrone()->RemoveNavdataSubscriber(this);
	}
	
	
	void ControlService::ProcessNavdata(Navdata *data)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		NavdataOptionDemo *options = data->GetOptionWithTag<NavdataOptionDemo>(0);
		if(options)
		{
			_droneState = options->ctrl_state;
			_hasNavdata = true;
			
			switch(_droneState >> 16)
			{
				case ControlStateFlying:
				case ControlStateGotoFix:
				case ControlStateHovering:
				case ControlStateLooping:
					_flyState = FlyState::Flying;
					break;
					
				case ControlStateLanding:
					_flyState = FlyState::Landing;
					break;
					
				case ControlStateTakeOff:
					_flyState = FlyState::TakingOff;
					
				case ControlStateDefault:
				case ControlStateLanded:
				default:
					_flyState = FlyState::Landed;
					break;
			}
		}
		
		if((data->state & ARDRONE_EMERGENCY_MASK) && !_emergency)
		{
			_emergency    = true;
			_wantsTakeOff = false;
			
			TickNow();
		}
		
		Wakeup(WakeupReason::DataAvilable);
	}
	
	void ControlService::Ftrim()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_wantsFtrim = true;
		
		Wakeup(WakeupReason::DataAvilable);
	}
	
	void ControlService::TakeOff()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		if(!_wantsTakeOff && _flyState == FlyState::Landed)
		{
			_wantsTakeOff = true;
			_emergency = false;
			
			_direction = Vector3(0.0f);
			_angularSpeed = 0.0f;
			_hover = true;
			
			TickNow();
			Wakeup(WakeupReason::DataAvilable);
		}
	}
	
	void ControlService::Land()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		if(_wantsTakeOff && _flyState == FlyState::Flying)
		{
			_wantsTakeOff = false;
			Wakeup(WakeupReason::DataAvilable);
		}
	}
	
	void ControlService::Hover()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		_direction = Vector3(0.0f);
		_hover = true;
		
		Wakeup(WakeupReason::DataAvilable);
	}
	
	void ControlService::SetEmergency(bool emergency)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_emergency = emergency;
		
		if(_emergency)
			_wantsTakeOff = false;
		
		Wakeup(WakeupReason::DataAvilable);
	}
	
	void ControlService::SetDirection(const Vector3 &direction)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_direction.x = std::max(-1.0f, std::min(direction.x, 1.0f));
		_direction.y = std::max(-1.0f, std::min(direction.y, 1.0f));
		_direction.z = std::max(-1.0f, std::min(direction.z, 1.0f));
		
		if(fabsf(_direction.x) > 0.01 || fabsf(_direction.z) > 0.01)
			_hover = false;
		
		Wakeup(WakeupReason::DataAvilable);
	}
	
	void ControlService::SetAngularSpeed(float speed)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		_angularSpeed = std::max(-1.0f, std::min(speed, 1.0f));
		
		Wakeup(WakeupReason::DataAvilable);
	}
	
	
	
	ControlService::FlyState ControlService::GetFlyState()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _flyState;
	}
	
	
	bool ControlService::HasEmergency()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _emergency;
	}
	
	bool ControlService::IsHovering()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _hover;
	}
	
	Vector3 ControlService::GetDirection()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _direction;
	}
	
	float ControlService::GetAngularSpeed()
	{
		std::lock_guard<std::mutex> lock(_mutex);
		return _angularSpeed;
	}
	
	void ControlService::TickNow()
	{
		{
			uint32_t data = _hover ? 0 : 1;
			_atService->Send(ATCommand("PCMD") << data << _direction.x << _direction.z << _direction.y << _angularSpeed);
		}
		
		{
			uint32_t data = (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24) | (1 << 28);
			
			if(_wantsTakeOff)
				data |= (1 << 9);
			if(_emergency)
				data |= (1 << 8);
			
			_atService->Send(ATCommand("REF") << data);
		}
	}
	
	void ControlService::Tick(uint32_t reason)
	{
		std::lock_guard<std::mutex> lock(_mutex);
		
		if(!_hasNavdata)
		{
			// Keep the Drone entertained while we are waiting for Navdata
			{
				_atService->Send(ATCommand("PCMD") << 0 << 0.0f << 0.0f << 0.0f << 0.0f);
			}
			
			{
				uint32_t data = (1 << 18) | (1 << 20) | (1 << 22) | (1 << 24) | (1 << 28);
				_atService->Send(ATCommand("REF") << data);
			}
			
			return;
		}
		
		if(_wantsFtrim)
		{
			if(_flyState == FlyState::Landed)
				_atService->Send(ATCommand("FTRIM"));
			
			_wantsFtrim = false;
		}
		
		
		auto now = std::chrono::steady_clock::now();
		auto time = std::chrono::duration_cast<std::chrono::milliseconds>(now - _lastPackage).count();
		
		if(time >= 28)
		{
			TickNow();
			_lastPackage = now;
		}
	}
}
