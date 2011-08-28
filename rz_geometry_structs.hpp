/** @file rz_geometry_structs.hpp */
// classes: rz_point_2d, rz_point_3d, rz_tri, rz_aabb, rz_line
// description: geometry objects and related operators
// last updated: aug.28.2011

// Copyright (C) 2011 Rim Zaidullin <tinybit@yandex.ru>

// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation; either version 2
// of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#ifndef _RZ_GEOMETRY_STRUCTS_HPP_INCLUDED_
#define _RZ_GEOMETRY_STRUCTS_HPP_INCLUDED_

#include <math.h>

namespace rimz {

static const double EPS = 0.0000000001;
static const double MINF = -999999999.0;
static const double MAXF = 999999999.0;

template <typename T> // T can be double or float only!
class rz_point_2d {
public:
	// initialization
	rz_point_2d() : x(0.0f), y(0.0f) {};
	rz_point_2d(T x_, T y_) : x(x_), y(y_) {};
	rz_point_2d(const rz_point_2d<T>& rhs) : x(rhs.x), y(rhs.y) {};
	rz_point_2d<T>& operator = (const rz_point_2d<T>& rhs) {
		if (this != &rhs) {
			x = rhs.x;
			y = rhs.y;
		}
		
		return (*this);
	}
	
	// comparison
	inline bool operator == (const rz_point_2d<T>& rhs) const {
		 return ((fabs(x - rhs.x) < EPS) && (fabs(y - rhs.y) < EPS));
	}
	
	inline bool operator != (const rz_point_2d<T>& rhs) const {
		return (!(*this == rhs));
	}
	
	//math
	inline rz_point_2d<T> operator - () const {
		return rz_point_2d<T>(-x, -y);
	}

	inline rz_point_2d<T> operator - (const rz_point_2d<T>& rhs) {
		return rz_point_2d<T>(x - rhs.x, y - rhs.y);
	}
	
	inline rz_point_2d<T> operator + (const rz_point_2d<T>& rhs) {
		return rz_point_2d<T>(x + rhs.x, y + rhs.y);
	}
		
	inline void operator += (const rz_point_2d<T>& rhs) {
		x += rhs.x;
		y += rhs.y;
	}
	
	inline void operator -= (const rz_point_2d<T>& rhs) {
		x -= rhs.x;
		y -= rhs.y;
	}
	
	inline void operator *= (T value) {
		x *= value;
		y *= value;
	}
	
	inline void operator /= (T value) {
		x /= value;
		y /= value;
	}
	
	inline double length() const {
		return sqrt(x * x + y * y);
	}
	
	inline double normalize() {
		double len = length();
		x /= len;
		y /= len;
		return len;
	}
	
	// data
	T x;
	T y;
};

template <typename T>
inline rz_point_2d<T> operator - (const rz_point_2d<T>& lhs, const rz_point_2d<T>& rhs) {
	return rz_point_2d<T>(lhs.x - rhs.x, lhs.y - rhs.y);
}

template <typename T>
inline rz_point_2d<T> operator + (const rz_point_2d<T>& lhs, const rz_point_2d<T>& rhs) {
	return rz_point_2d<T>(lhs.x + rhs.x, lhs.y + rhs.y);
}

template <typename T> // T can be double or float only!
class rz_point_3d {
public:
	// initialization
	rz_point_3d() : x(0.0), y(0.0), z(0.0) {};
	rz_point_3d(T x_, T y_, T z_) : x(x_), y(y_), z(z_) {};
	rz_point_3d(const rz_point_3d<T>& rhs) : x(rhs.x), y(rhs.y), z(rhs.z) {};
	rz_point_3d<T>& operator = (const rz_point_3d<T>& rhs) {
		if (this != &rhs) {
			x = rhs.x;
			y = rhs.y;
			z = rhs.z;
		}
		
		return (*this);
	}
	
	// comparison
	inline bool operator == (const rz_point_3d<T>& rhs) const {
		return ((fabs(x - rhs.x) < EPS) && (fabs(y - rhs.y) < EPS) && (fabs(z - rhs.z) < EPS));
	}
	
	inline bool operator != (const rz_point_3d<T>& rhs) const {
		return (!(*this == rhs));
	}
	
	//math
	inline rz_point_3d<T> operator - () const {
		return rz_point_3d<T>(-x, -y, -z);
	}
	
	inline rz_point_3d<T> operator - (const rz_point_3d<T>& rhs) {
		return rz_point_3d<T>(x - rhs.x, y - rhs.y, z - rhs.z);
	}
	
	inline rz_point_3d<T> operator + (const rz_point_3d<T>& rhs) {
		return rz_point_3d<T>(x + rhs.x, y + rhs.y, z + rhs.z);
	}
	
	inline void operator += (const rz_point_3d<T>& rhs) {
		x += rhs.x;
		y += rhs.y;
		z += rhs.z;
	}
	
	inline void operator -= (const rz_point_3d<T>& rhs) {
		x -= rhs.x;
		y -= rhs.y;
		z -= rhs.z;
	}
	
	inline void operator *= (T value) {
		x *= value;
		y *= value;
		z *= value;
	}
	
	inline void operator /= (T value) {
		x /= value;
		y /= value;
		z /= value;
	}
	
	inline double length() const {
		return sqrt(x * x + y * y + z * z);
	}
	
	inline double normalize() {
		double len = length();
		x /= len;
		y /= len;
		z /= len;
		return len;
	}
	
	// data
	T x;
	T y;
	T z;
};

template <typename T>
inline rz_point_3d<T> operator - (const rz_point_3d<T>& lhs, const rz_point_3d<T>& rhs) {
	return rz_point_3d<T>(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z);
}

template <typename T>
inline rz_point_3d<T> operator + (const rz_point_3d<T>& lhs, const rz_point_3d<T>& rhs) {
	return rz_point_3d<T>(lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z);
}
	
// triangle, where each component represents index of a point in polygon
class tri_indexed {
public:
	tri_indexed() {
		memset(point, 3, sizeof(point));
	};

	// comparison
	inline bool operator == (const tri_indexed& rhs) const {
		return (point[0] == rhs.point[0] && point[1] == rhs.point[1] && point[2] == rhs.point[2]);
	}
	
	inline bool operator != (const tri_indexed& rhs) const {
		return (!(*this == rhs));
	}

	size_t point[3];
};

// aabb
template <typename T>
class rz_aabb {
public:
	rz_aabb() {}
	rz_aabb(const T& min_, const T& max_) : min(min_), max(max_) {}
	rz_aabb(const rz_aabb& aabb) : min(aabb.min), max(aabb.min) {}

	inline rz_aabb<T> offset(double x, double y) {
		min.x -= x;
		max.x -= x;
		min.y -= y;
		max.y -= y;
		return *this;
	}

	inline rz_aabb<T> offset_tmp(double x, double y) const {
		rz_aabb<T> tmp = *this;
		return tmp.offset(x, y);
	}
	
	inline double width() const {
		return fabs(max.x - min.x);
	}
	
	inline double height() const {
		return fabs(max.y - min.y);
	}
	
	// comparison
	inline bool operator == (const rz_aabb<T>& rhs) const {
		return (min == rhs.min && max == rhs.max);
	}
	
	inline bool operator != (const rz_aabb<T>& rhs) const {
		return (!(*this == rhs));
	}

	typedef T point_type;
	T min;
	T max;
};

// tri with data
template <typename T>
class rz_tri {
public:
	rz_tri() {};
	rz_tri(const T& a, const T& b, const T& c) {
		point[0] = a;
		point[1] = b;
		point[2] = c;
	};
	
	inline rz_aabb<T> aabb() const {
		rz_aabb<T> aabb_tmp(T(MAXF, MAXF), T(MINF, MINF));

		for (int i = 0; i < 3; ++i) {
			if (point[i].x < aabb_tmp.min.x) {
				aabb_tmp.min.x = point[i].x;
			}
			
			if (point[i].y < aabb_tmp.min.y) {
				aabb_tmp.min.y = point[i].y;
			}
			
			if (point[i].x >= aabb_tmp.max.x) {
				aabb_tmp.max.x = point[i].x;
			}
			
			if (point[i].y >= aabb_tmp.max.y) {
				aabb_tmp.max.y = point[i].y;
			}
		}
		
		return aabb_tmp;
	}
	
	// comparison
	inline bool operator == (const rz_tri<T>& rhs) const {
		return (point[0] == rhs.point[0] && point[1] == rhs.point[1] && point[2] == rhs.point[2]);
	}
	
	inline bool operator != (const rz_tri<T>& rhs) const {
		return (!(*this == rhs));
	}
	
	typedef T point_type;
	T point[3];
};

// line
template <typename T>
class rz_line {
public:
	rz_line() {}
	rz_line(const T& begin_, const T& end_) : begin(begin_), end(end_) {}
	rz_line(const rz_line& line) : begin(line.begin), end(line.end) {}
	
	// comparison
	inline bool operator == (const rz_line<T>& rhs) const {
		return (begin == rhs.begin && end == rhs.end);
	}
	
	inline bool operator != (const rz_line<T>& rhs) const {
		return (!(*this == rhs));
	}
	
	typedef T point_type;
	T begin;
	T end;
};
	
} // namespace rimz

#endif // _RZ_GEOMETRY_STRUCTS_HPP_INCLUDED_
