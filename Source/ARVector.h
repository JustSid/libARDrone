//
//  ARVector.h
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

#ifndef __libARDrone__ARVector__
#define __libARDrone__ARVector__

#include <stdio.h>
#include <math.h>
#include <algorithm>
#include <type_traits>

namespace AR
{
	class Vector3
	{
	public:
		Vector3() = default;
		
		Vector3(const float n)
		{
			x = y = z = n;
		}
		
		Vector3(const float _x, const float _y, const float _z)
		{
			x = _x;
			y = _y;
			z = _z;
		}
		
		bool operator== (const Vector3 &other) const
		{
			if(fabs(x - other.x) > 0.001)
				return false;
			
			if(fabs(y - other.y) > 0.001)
				return false;
			
			if(fabs(z - other.z) > 0.001)
				return false;
			
			return true;
		}
		
		bool operator!= (const Vector3 &other) const
		{
			if(fabs(x - other.x) <= 0.001 && fabs(y - other.y) <= 0.001 && fabs(z - other.z) <= 0.001)
				return false;
			
			return true;
		}
		
		Vector3 operator- () const
		{
			return Vector3(-x, -y, -z);
		}
		
		Vector3 operator+ (const Vector3 &other) const
		{
			return Vector3(x + other.x, y + other.y, z + other.z);
		}
		Vector3 operator- (const Vector3 &other) const
		{
			return Vector3(x - other.x, y - other.y, z - other.z);
		}
		Vector3 operator* (const Vector3 &other) const
		{
			return Vector3(x * other.x, y * other.y, z * other.z);
		}
		Vector3 operator/ (const Vector3 &other) const
		{
			return Vector3(x / other.x, y / other.y, z / other.z);
		}
		Vector3 operator* (const float n) const
		{
			return Vector3(x * n, y * n, z * n);
		}
		Vector3 operator/ (const float n) const
		{
			return Vector3(x / n, y / n, z / n);
		}
		
		Vector3 &operator+= (const Vector3 &other)
		{
			x += other.x;
			y += other.y;
			z += other.z;
			
			return *this;
		}
		Vector3 &operator-= (const Vector3 &other)
		{
			x -= other.x;
			y -= other.y;
			z -= other.z;
			
			return *this;
		}
		Vector3 &operator*= (const Vector3 &other)
		{
			x *= other.x;
			y *= other.y;
			z *= other.z;
			
			return *this;
		}
		Vector3 &operator/= (const Vector3 &other)
		{
			x /= other.x;
			y /= other.y;
			z /= other.z;
			
			return *this;
		}
		
		float GetLength() const
		{
			return sqrtf(x * x + y * y + z * z);
		}
		
		float GetMax() const
		{
			return std::max(std::max(x, y), z);
		}
		
		float GetMin() const
		{
			return std::min(std::min(x, y), z);
		}
		
		float GetDotProduct(const Vector3 &other) const
		{
			return (x * other.x + y * other.y + z * other.z);
		}
		
		Vector3 GetCrossProduct(const Vector3 &other) const
		{
			Vector3 result;
			
			result.x = y * other.z - z * other.y;
			result.y = z * other.x - x * other.z;
			result.z = x * other.y - y * other.x;
			
			return result;
		}
		
		bool IsEqual(const Vector3 &other, float epsilon) const
		{
			if(fabs(x - other.x) > epsilon)
				return false;
			
			if(fabs(y - other.y) > epsilon)
				return false;
			
			if(fabs(z - other.z) > epsilon)
				return false;
			
			return true;
		}
		
		Vector3 &Normalize(const float n = 1.0f)
		{
			float length = GetLength();
			
			if(length != 0)
			{
				x /= length;
				y /= length;
				z /= length;
			}
			
			x *= n;
			y *= n;
			z *= n;
			
			return *this;
		}
		
		Vector3 GetNormalized(const float n = 1.0f) const
		{
			return Vector3(*this).Normalize(n);
		}
		
		float GetDistance(const Vector3 &other) const
		{
			Vector3 difference = *this - other;
			return difference.GetLength();
		}
		
		float GetSquaredDistance(const Vector3 &other) const
		{
			Vector3 difference = *this - other;
			return difference.GetDotProduct(difference);
		}
		
		Vector3 GetLerp(const Vector3 &other, float factor) const
		{
			return *this*(1.0f-factor)+other*factor;
		}
		
		struct
		{
			float x;
			float y;
			float z;
		};
	};
	
	static_assert(std::is_trivial<Vector3>::value, "Location must be a trivial type!");
}

#endif /* defined(__libARDrone__ARVector__) */
