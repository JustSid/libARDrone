//
//  ARLocation.h
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

#ifndef __libARDrone__ARLocation__
#define __libARDrone__ARLocation__

#include <math.h>

namespace AR
{
	class Location
	{
	public:
		Location(double tlatitude, double tlongitude) :
			latitude(tlatitude),
			longitude(tlongitude)
		{}
		
		Location(const Location &other) = default;
		Location &operator = (const Location &other) = default;
		
		double GetDistance(const Location &other) const
		{
			static double degreesToRadian = 3.14159265358979323846264338327950288 / 180.0;
			
			double dlong = (other.longitude - longitude) * degreesToRadian;
			double dlat  = (other.latitude - latitude) * degreesToRadian;
			
			double a = pow(sin(dlat / 2.0), 2) + cos(latitude * degreesToRadian) * cos(other.latitude * degreesToRadian) * pow(sin(dlong / 2.0), 2);
			double c = 2 * atan2(sqrt(a), sqrt(1 - a));
			
			return (c * 6367) * 1000.0;
		}
		
		double latitude;
		double longitude;
	};
}

#endif /* defined(__libARDrone__ARLocation__) */
