/** @file rz_quadtree.hpp */
// class: rz_quadtree
// description: generic quadtree, by default can be used with triangles,
// lines or axis-aligned boxes. objects lookup can be done by point or
// axis-aligned box. this class can be easily extended to store any 2d
// primitive, you just have to specify min_2d,max_2d and intersect_2d
// methods (intersection of aabb with your object)
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

#ifndef _RZ_QUADTREE_HPP_INCLUDED_
#define _RZ_QUADTREE_HPP_INCLUDED_

#include <cmath>
#include <memory>
#include <iomanip>
#include <stdexcept>

#include "rz_quadtree_node.hpp"
#include "rz_geometry_structs.hpp"
#include "rz_geometry_math.hpp"
		
namespace rimz {

template <typename T>
class rz_quadtree {
public:
	typedef std::vector<T> o_vector;
	typedef rz_quadtree_node<T> q_node;
	typedef typename T::point_type point2d;
	typedef rz_aabb<point2d> aabb2d;
	
	rz_quadtree(const o_vector& objects_list);
	rz_quadtree(const o_vector& objects_list, size_t objects_threshold, size_t depth_threshold);
	virtual ~rz_quadtree();

	void get_objects_from_point(const point2d& pt, o_vector& objects);
	void get_objects_from_aabb(const aabb2d& pt, o_vector& objects);

private:
	void build_tree(const o_vector& objects_list);
	void build_sub_tree(q_node* node, const o_vector& objects_list, const point2d& box_origin, double box_size, size_t depth);
	void get_min_max(const o_vector& objects_list, point2d& min, point2d& max);
	void intersect_objects_with_cell(const o_vector& objects_list, const point2d& box_origin, double box_size, o_vector& intersected_objects_list);
	
	void intersect_tree_with_point(const point2d& pt, o_vector& objects, q_node* node);
	bool intersect_node_with_point(const point2d& pt, q_node* node);
	
	void intersect_tree_with_aabb(const aabb2d& pt, o_vector& objects, q_node* node);
	bool intersect_node_with_aabb(const aabb2d& aabb, q_node* node);

	point2d min_;		// actual data min (also used as root node coords origin)
	point2d max_;		// actual data max
	double box_size_;	// aligned root node size

	std::auto_ptr<q_node> root_;
	size_t objects_threshold_;
	size_t depth_threshold_;
};

template <typename T> inline
rz_quadtree<T>::rz_quadtree(const o_vector& objects_list) :
objects_threshold_(10), depth_threshold_(12) {
	build_tree(objects_list);
}

template <typename T> inline
rz_quadtree<T>::rz_quadtree(const o_vector& objects_list, size_t objects_threshold, size_t depth_threshold) :
objects_threshold_(objects_threshold), depth_threshold_(depth_threshold) {
	build_tree(objects_list);
}
	
template <typename T> inline
rz_quadtree<T>::~rz_quadtree() {
}

template <typename T> inline void
rz_quadtree<T>::get_min_max(const o_vector& objects_list, point2d& min, point2d& max) {
	point2d common_min;
	point2d common_max;

	common_min.x = MAXF;
	common_min.y = MAXF;
	common_max.x = MINF;
	common_max.y = MINF;

	for (size_t i = 0; i < objects_list.size(); ++i) {
		point2d obj_min = min_2d(objects_list[i]);
		point2d obj_max = max_2d(objects_list[i]);

		if (obj_min.x < common_min.x) {
			common_min.x = obj_min.x;
		}

		if (obj_min.y < common_min.y) {
			common_min.y = obj_min.y;
		}

		if (obj_max.x > common_max.x) {
			common_max.x = obj_max.x;
		}

		if (obj_max.y > common_max.y) {
			common_max.y = obj_max.y;
		}
	}

	min = common_min;
	max = common_max;
}

template <typename T> inline void
rz_quadtree<T>::build_tree(const o_vector& objects_list) {
	// calc objects min/max
	get_min_max(objects_list, min_, max_);

	// calc max align size
	double size_x = fabs(max_.x - min_.x);
	double size_y = fabs(max_.y - min_.y);

	if (size_x > size_y) {
		box_size_ = size_x;
	}
	else {
		box_size_ = size_y;
	}

	// build tree!
	root_.reset(new rz_quadtree_node<T>());
	root_->set_parent(NULL);
	root_->set_dimentions(min_, box_size_);

	build_sub_tree(root_.get(), objects_list, min_, box_size_, 0);
}

template <typename T> inline void
rz_quadtree<T>::build_sub_tree(q_node* node, const o_vector& objects_list, const point2d& box_origin, double box_size, size_t depth) {
	if (!node) {
		throw std::runtime_error("rz_quadtree build_sub_tree received null node!");
	}

	// check whether node box intersects with actual data
	point2d box_max(box_origin.x + box_size, box_origin.y + box_size);
	aabb2d cell_box(box_origin, box_max);
	aabb2d data_box(min_, max_);

	if (false == intersect_2d(cell_box, data_box)) {
		node->set_leaf(true);
		return;
	}

	// store objects
	o_vector& node_obj_list = node->objects_list();
	node_obj_list.assign(objects_list.begin(), objects_list.end());

	// check thresholds
	if (objects_list.size() <= objects_threshold_ ) {
		node->set_leaf(true);
		return;
	}

	if (depth >= depth_threshold_) {
		node->set_leaf(true);
		return;
	}

	// create children nodes
	node->create_children();

	double sub_box_size = box_size / 2.0;

	o_vector intersected_objects_list;
	q_node* sub_node = NULL;

	// sub-box A
	point2d sub_box_origin_a(box_origin.x, box_origin.y + sub_box_size);
	intersect_objects_with_cell(objects_list, sub_box_origin_a, sub_box_size, intersected_objects_list);
	sub_node = node->child_a();
	sub_node->set_parent(node);
	sub_node->set_dimentions(sub_box_origin_a, sub_box_size);
	build_sub_tree(sub_node, intersected_objects_list, sub_box_origin_a, sub_box_size, depth + 1);

	// sub-box B
	point2d sub_box_origin_b(box_origin.x + sub_box_size, box_origin.y + sub_box_size);
	intersect_objects_with_cell(objects_list, sub_box_origin_b, sub_box_size, intersected_objects_list);
	sub_node = node->child_b();
	sub_node->set_parent(node);
	sub_node->set_dimentions(sub_box_origin_b, sub_box_size);
	build_sub_tree(sub_node, intersected_objects_list, sub_box_origin_b, sub_box_size, depth + 1);

	// sub-box C
	point2d sub_box_origin_c(box_origin.x, box_origin.y);
	intersect_objects_with_cell(objects_list, sub_box_origin_c, sub_box_size, intersected_objects_list);
	sub_node = node->child_c();
	sub_node->set_parent(node);
	sub_node->set_dimentions(sub_box_origin_c, sub_box_size);
	build_sub_tree(sub_node, intersected_objects_list, sub_box_origin_c, sub_box_size, depth + 1);

	// sub-box D
	point2d sub_box_origin_d(box_origin.x + sub_box_size, box_origin.y);
	intersect_objects_with_cell(objects_list, sub_box_origin_d, sub_box_size, intersected_objects_list);
	sub_node = node->child_d();
	sub_node->set_parent(node);
	sub_node->set_dimentions(sub_box_origin_d, sub_box_size);
	build_sub_tree(sub_node, intersected_objects_list, sub_box_origin_d, sub_box_size, depth + 1);
}

template <typename T> inline void
rz_quadtree<T>::intersect_objects_with_cell(const o_vector& objects_list, const point2d& box_origin, double box_size, o_vector& intersected_objects_list) {
	point2d box_max(box_origin.x + box_size, box_origin.y + box_size);
	aabb2d cell_box(box_origin, box_max);

	intersected_objects_list.clear();

	for (size_t i = 0; i < objects_list.size(); ++i) {
		if (intersect_2d(cell_box, objects_list[i])) {
			intersected_objects_list.push_back(objects_list[i]);
		}
	}
}

template <typename T> inline void
rz_quadtree<T>::get_objects_from_point(const point2d& pt, o_vector& objects) {
	intersect_tree_with_point(pt, objects, root_.get());
}

template <typename T> inline void
rz_quadtree<T>::get_objects_from_aabb(const aabb2d& aabb, o_vector& objects) {
	intersect_tree_with_aabb(aabb, objects, root_.get());
}

template <typename T> inline void
rz_quadtree<T>::intersect_tree_with_point(const point2d& pt, o_vector& objects, q_node* node) {
	if (!node) {
		throw std::runtime_error("rz_quadtree get_objects_from_point received null node!");
	}

	// check wether we hit actual data bbox
	aabb2d box(min_, max_);
	if (false == intersect_2d(box, pt)) {
		return;
	}

	// intersect with currect node
	if (intersect_node_with_point(pt, node)) {
		if (node->is_leaf()) {
			o_vector& node_objects = node->objects_list();
			objects.assign(node_objects.begin(), node_objects.end());
			return;
		}
		else {
			if (intersect_node_with_point(pt, node->child_a())) {
				intersect_tree_with_point(pt, objects, node->child_a());
				return;
			}

			if (intersect_node_with_point(pt, node->child_b())) {
				intersect_tree_with_point(pt, objects, node->child_b());
				return;
			}

			if (intersect_node_with_point(pt, node->child_c())) {
				intersect_tree_with_point(pt, objects, node->child_c());
				return;
			}

			if (intersect_node_with_point(pt, node->child_d())) {
				intersect_tree_with_point(pt, objects, node->child_d());
				return;
			}
		}
	}
	else {
		return;
	}
}

template <typename T> inline void
rz_quadtree<T>::intersect_tree_with_aabb(const aabb2d& aabb, o_vector& objects, q_node* node) {
	if (!node) {
		throw std::runtime_error("rz_quadtree get_objects_from_aabb received null node!");
	}
	
	// check wether we hit actual data bbox
	aabb2d box(min_, max_);
	if (false == intersect_2d(box, aabb)) {
		return;
	}
	
	// intersect with currect node
	if (intersect_node_with_aabb(aabb, node)) {
		if (node->is_leaf()) {
			o_vector& node_objects = node->objects_list();
			objects.insert(objects.end(), node_objects.begin(), node_objects.end());
			return;
		}
		else {
			if (intersect_node_with_aabb(aabb, node->child_a())) {
				intersect_tree_with_aabb(aabb, objects, node->child_a());
			}
			
			if (intersect_node_with_aabb(aabb, node->child_b())) {
				intersect_tree_with_aabb(aabb, objects, node->child_b());
			}
			
			if (intersect_node_with_aabb(aabb, node->child_c())) {
				intersect_tree_with_aabb(aabb, objects, node->child_c());
			}
			
			if (intersect_node_with_aabb(aabb, node->child_d())) {
				intersect_tree_with_aabb(aabb, objects, node->child_d());
			}
		}
	}
	else {
		return;
	}
}
	
template <typename T> inline bool
rz_quadtree<T>::intersect_node_with_point(const point2d& pt, q_node* node) {
	if (!node) {
		throw std::runtime_error("rz_quadtree intersect_node_with_point received null node!");
	}

	point2d origin;
	double size;
	node->get_dimentions(origin, size);

	aabb2d box(origin, point2d(origin.x + size, origin.y + size));
	return intersect_2d(box, pt);
}

template <typename T> inline bool
rz_quadtree<T>::intersect_node_with_aabb(const aabb2d& aabb, q_node* node) {
	if (!node) {
		throw std::runtime_error("rz_quadtree intersect_node_with_aabb received null node!");
	}
	
	point2d origin;
	double size;
	node->get_dimentions(origin, size);
	
	aabb2d box(origin, point2d(origin.x + size, origin.y + size));
	return intersect_2d(box, aabb);
}

} // namespace rimz

#endif // _RZ_QUADTREE_HPP_INCLUDED_
