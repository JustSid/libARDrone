//
//  ARATService.h
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

#ifndef __libARDrone__ARATService__
#define __libARDrone__ARATService__

#include <mutex>
#include <string>
#include <sstream>
#include <vector>

#include "ARService.h"
#include "ARSocket.h"

namespace AR
{
	class ATCommand
	{
	public:
		ATCommand(const std::string &command);
		
		std::string GetCommand(uint32_t sequence) const;
		
		ATCommand &operator << (const std::string &val) { _stream << ",\"" << val <<  "\""; return *this; }
		ATCommand &operator << (const char *val) { _stream << ",\"" << val <<  "\""; return *this; }
		ATCommand &operator << (bool val) { _stream << "," << val; return *this; }
		ATCommand &operator << (short val) { _stream << "," << val; return *this; }
		ATCommand &operator << (unsigned short val) { _stream << "," << val; return *this; }
		ATCommand &operator << (int val) { _stream << "," << val; return *this; }
		ATCommand &operator << (unsigned int val) { _stream << "," << val; return *this; }
		ATCommand &operator << (long val) { _stream << "," << val; return *this; }
		ATCommand &operator << (unsigned long val) { _stream << "," << val; return *this; }
		ATCommand &operator << (long long val) { _stream << "," << val; return *this; }
		ATCommand &operator << (unsigned long long val) { _stream << "," << val; return *this; }
		ATCommand &operator << (float val)
		{
			union
			{
				float f;
				int i;
			} t;
			
			t.f = val;
			
			_stream << "," << t.i;
			return *this;
		}
		
	private:
		std::stringstream _stream;
	};
	
	class ATService : public Service
	{
	public:
		ATService(Drone *drone, const std::string &address);
		~ATService() override;
		
		void Send(const ATCommand &command);
		
	protected:
		void Tick(uint32_t reason) final;
		
		State ConnectInternal() final;
		void DisconnectInternal() final;
		
		Socket *_socket;
		std::mutex _mutex;
		uint32_t _sequence;
		std::vector<std::string> _queue;
	};
}

#endif /* defined(__libARDrone__ARATService__) */
