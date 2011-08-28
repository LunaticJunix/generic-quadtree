/** @file rz_geometry_math.hpp */
// class: none
// description: set of math functions, mostly object intersections
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

#ifndef _RZ_GEOMETRY_MATH_HPP_INCLUDED_
#define _RZ_GEOMETRY_MATH_HPP_INCLUDED_

#include <iostream>
#include <math.h>

#include "rz_geometry_structs.hpp"

namespace rimz {
	
template <typename T>
inline bool intersect_2d(const rz_line<T>& line_a, const rz_line<T>& line_b) {
	double mua, mub;
	
	T a1 = line_a.begin;
	T a2 = line_a.end;
	T b1 = line_b.begin;
	T b2 = line_b.end;
	
	double denom  = (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y);
	double numera = (b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x);
	double numerb = (a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x);
	
	// Are the line coincident?
	if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS) {
		return true;
	}
	
	// Are the line parallel
	if (fabs(denom) < EPS) {
		return false;
	}
	
	// is the intersection along the the segments
	mua = numera / denom;
	mub = numerb / denom;
	if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
		return false;
	}
	
	return true;
}
	
template <typename T>
inline bool intersect_2d(const rz_line<T>& line_a, const rz_line<T>& line_b, T& isect_point) {
	double mua, mub;
	
	T a1 = line_a.begin;
	T a2 = line_a.end;
	T b1 = line_a.begin;
	T b2 = line_a.end;
	
	double denom  = (b2.y - b1.y) * (a2.x - a1.x) - (b2.x - b1.x) * (a2.y - a1.y);
	double numera = (b2.x - b1.x) * (a1.y - b1.y) - (b2.y - b1.y) * (a1.x - b1.x);
	double numerb = (a2.x - a1.x) * (a1.y - b1.y) - (a2.y - a1.y) * (a1.x - b1.x);
	
	// are the line coincident?
	if (fabs(numera) < EPS && fabs(numerb) < EPS && fabs(denom) < EPS) {
		isect_point.x = (a1.x + a2.x) / 2.0;
		isect_point.y = (a1.y + a2.y) / 2.0;
		return true;
	}
	
	// are the line parallel
	if (fabs(denom) < EPS) {
		isect_point.x = 0;
		isect_point.y = 0;
		return false;
	}
	
	// is the intersection along the the segments
	mua = numera / denom;
	mub = numerb / denom;
	if (mua < 0 || mua > 1 || mub < 0 || mub > 1) {
		isect_point.x = 0;
		isect_point.y = 0;
		return false;
	}
	
	isect_point.x = a1.x + mua * (a2.x - a1.x);
	isect_point.y = a1.y + mua * (a2.y - a1.y);
	return true;
}
	
template <typename T>
inline bool intersect_2d(const rz_tri<T>& tri, const T& pt) {
	T v0 = tri.point[2] - tri.point[0];
	T v1 = tri.point[1] - tri.point[0];
	T v2 = pt - tri.point[0];
	
	double dot00 = dot_product(v0, v0);
	double dot01 = dot_product(v0, v1);
	double dot02 = dot_product(v0, v2);
	double dot11 = dot_product(v1, v1);
	double dot12 = dot_product(v1, v2);
	
	// compute barycentric coordinates
	double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
	double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
	double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
	
	// check if point is in triangle
	return (u > 0.0) && (v > 0.0) && (u + v < 1.0);
}

template <typename T>
inline bool intersect_2d(const rz_aabb<T>& aabb, const rz_tri<T>& tri) {
	if (intersect_2d(tri.aabb(), aabb) == false) {
		return false;
	}
	
	// check whether any of the tri vertex is inside aabb
	for (int i = 0; i < 3; ++i) {
		if (tri.point[i].x > aabb.min.x && tri.point[i].x < aabb.max.x &&
			tri.point[i].y > aabb.min.y && tri.point[i].y < aabb.max.y) {
			return true;
		}
	}
	
	// check whether any of the aabb vertex is inside tri
	if (intersect_2d(tri, aabb.min)) {
		return true;
	}
	
	if (intersect_2d(tri, aabb.max)) {
		return true;
	}
	
	if (intersect_2d(tri, T(aabb.min.x, aabb.max.y))) {
		return true;
	}
	
	if (intersect_2d(tri, T(aabb.max.x, aabb.min.y))) {
		return true;
	}
	
	// intersect tri edges against aabb edges
	T aabb_edges[4];
	aabb_edges[0] = aabb.min;
	aabb_edges[1] = T(aabb.min.x, aabb.max.y);
	aabb_edges[2] = aabb.max;
	aabb_edges[3] = T(aabb.max.x, aabb.min.y);
	
	for (int i = 0; i < 4; ++i) {
		T pt1 = aabb_edges[i];
		T pt2;
		
		if (i == 3) {
			pt2 = aabb_edges[0];
		}
		else {
			pt2 = aabb_edges[i + 1];			
		}
		
		for (int j = 0; j < 3; ++j) {
			T pt3 = tri.point[j];
			T pt4;
			
			if (j == 2) {
				pt4 = tri.point[0];
			}
			else {
				pt4 = tri.point[j + 1];
			}
			
			typedef rz_line<T> line2d;
			if (intersect_2d(line2d(pt1, pt2), line2d(pt3, pt4))) {
				return true;
			}
		}
	}
	
	return false;
}

template <typename T>
inline bool intersect_2d(const rz_aabb<T>& aabb, const rz_aabb<T>& aabb_b) {
	if (aabb == aabb_b) {
		return true;
	}

	double total_boxes_x_len = fabs(fmax(aabb.max.x, aabb_b.max.x) - fmin(aabb.min.x, aabb_b.min.x));
	double total_boxes_y_len = fabs(fmax(aabb.max.y, aabb_b.max.y) - fmin(aabb.min.y, aabb_b.min.y));
	
	if (total_boxes_x_len < (aabb.width() + aabb_b.width()) && total_boxes_y_len < (aabb.height() + aabb_b.height())) {
		return true;
	}
	
	return false;
}

template <typename T>
inline bool intersect_2d(const rz_aabb<T>& aabb, const rz_line<T>& line) {
	// go through aabb vertexes
	if (line.begin.x > aabb.min.x && line.begin.x < aabb.max.x &&
		line.begin.y > aabb.min.y && line.begin.y < aabb.max.y) {
		return true;
	}
	
	if (line.end.x > aabb.min.x && line.end.x < aabb.max.x &&
		line.end.y > aabb.min.y && line.end.y < aabb.max.y) {
		return true;
	}
	
	T aabb_points[4];
	aabb_points[0] = aabb.min;
	aabb_points[1] = T(aabb.min.x, aabb.max.y);
	aabb_points[2] = aabb.max;
	aabb_points[3] = T(aabb.max.x, aabb.min.y);
	
	// go through aabb edges
	for (int i = 0; i < 4; ++i) {
		rz_line<T> aabb_line;
		aabb_line.begin = aabb_points[i];
		
		if (i == 3) {
			aabb_line.end = aabb_points[0];
		}
		else {
			aabb_line.end = aabb_points[i + 1];
		}
		
		if (intersect_2d(line, aabb_line)) {
			return true;
		}
	}
	
	return false;
}

template <typename T>
inline bool intersect_2d(const rz_aabb<T>& aabb, const T& pt) {
	return (pt.x > aabb.min.x && pt.x <= aabb.max.x && pt.y > aabb.min.y && pt.y <= aabb.max.y);
}

template <typename T>
inline T min_2d(const rz_line<T>& line) {
	T ret_val;
	
	ret_val.x = line.begin.x < line.end.x ? line.begin.x : line.end.x;
	ret_val.y = line.begin.y < line.end.y ? line.begin.y : line.end.y;
	
	return ret_val;
}

template <typename T>
inline T max_2d(const rz_line<T>& line) {
	T ret_val;
	
	ret_val.x = line.begin.x > line.end.x ? line.begin.x : line.end.x;
	ret_val.y = line.begin.y > line.end.y ? line.begin.y : line.end.y;
	
	return ret_val;
}

template <typename T>
inline T min_2d(const rz_tri<T>& tri) {
	T ret_val;
	
	ret_val.x = tri.point[0].x < tri.point[1].x ? tri.point[0].x : tri.point[1].x;
	ret_val.x = ret_val.x < tri.point[2].x ? ret_val.x : tri.point[2].x;
	
	ret_val.y = tri.point[0].y < tri.point[1].y ? tri.point[0].y : tri.point[1].y;
	ret_val.y = ret_val.y < tri.point[2].y ? ret_val.y : tri.point[2].y;

	return ret_val;
}

template <typename T>
inline T max_2d(const rz_tri<T>& tri) {
	T ret_val;
	
	ret_val.x = tri.point[0].x > tri.point[1].x ? tri.point[0].x : tri.point[1].x;
	ret_val.x = ret_val.x > tri.point[2].x ? ret_val.x : tri.point[2].x;
	
	ret_val.y = tri.point[0].y > tri.point[1].y ? tri.point[0].y : tri.point[1].y;
	ret_val.y = ret_val.y > tri.point[2].y ? ret_val.y : tri.point[2].y;
	
	return ret_val;
}

template <typename T>
inline T min_2d(const rz_aabb<T>& aabb) {	
	return aabb.min;
}

template <typename T>
inline T max_2d(const rz_aabb<T>& aabb) {
	return aabb.max;
}

template <typename T>
inline double dot_product(const T& a, const T& b) {
	return a.x * b.x + a.y * b.y;
}

} // namespace rimz

#endif // _RZ_GEOMETRY_MATH_HPP_INCLUDED_
