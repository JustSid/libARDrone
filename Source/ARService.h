//
//  ARService.h
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

#ifndef __libARDrone__ARService__
#define __libARDrone__ARService__

#include <string>
#include <atomic>
#include <mutex>
#include <thread>
#include <condition_variable>

namespace AR
{
	class Drone;
	class Service
	{
	public:
		friend class Drone;
		
		enum class State
		{
			Disconnected,
			Connected,
			Connecting,
			Disconnecting
		};
		
		enum WakeupReason
		{
			DataAvilable = (1 << 0),
			Shutdown = (1 << 1)
		};
		
		Service(Drone *drone, const std::string &name);
		virtual ~Service();
		
		void Connect();
		void Disconnect();
		
		void Wakeup(WakeupReason reason);
		
		State GetState() const { return _state.load(); }
		const std::string &GetName() const { return _name; }
		uint32_t GetNavdataOptions() const { return _navdataOptions; }
		
	protected:
		void SetState(State state);
		Drone *GetDrone() const { return _drone; }
		
		virtual void Tick(uint32_t reason) = 0;
		
		virtual State ConnectInternal();
		virtual void DisconnectInternal();
		
		virtual void Update();
		
		void SetCanSleep(bool value);
		void SetCanTick();
		void UpdateNavdataOptions(uint32_t options);
		
	private:
		void ThreadHandler();
		
		Drone *_drone;
		
		std::atomic<State> _state;
		std::atomic<bool> _canTick;
		std::string _name;
		
		std::mutex _mutex;
		std::condition_variable _signal;
		std::thread _thread;
		
		bool _wakeup;
		std::atomic<bool> _canSleep;
		std::atomic<uint32_t> _reason;
		
		std::atomic<uint32_t> _navdataOptions;
	};
}

#endif /* defined(__libARDrone__ARService__) */
