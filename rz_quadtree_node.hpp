/** @file rz_quadtree_node.hpp */
// class: rz_quadtree_node
// description: class used as quadtree node (KO), can store list
// of any objects that you specify
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

#ifndef _RZ_QUADTREE_NODE_HPP_INCLUDED_
#define _RZ_QUADTREE_NODE_HPP_INCLUDED_

#include <cmath>
#include <memory>
#include <vector>

#include "rz_geometry_structs.hpp"

namespace rimz {

template <typename T>
class rz_quadtree_node {
public:
	typedef typename T::point_type point2d;
	rz_quadtree_node() : is_leaf_(false), parent_(NULL) {
	};

	virtual ~rz_quadtree_node() {}

	bool empty() {
		return objects_list_.empty();
	}

	std::vector<T>& objects_list() {
		return objects_list_;
	}

	void set_objects_list(const std::vector<T>& objects_list) {
		objects_list_.assign(objects_list.begin(), objects_list.end());
	}

	bool is_leaf() {
		return is_leaf_;
	}

	void set_leaf(bool value) {
		is_leaf_ = value;
	}

	const rz_quadtree_node* parent() {
		return parent_;
	}

	void set_parent(rz_quadtree_node* parent) {
		parent_ = parent;
	}

	void create_children() {
		child_a_.reset(new rz_quadtree_node<T>());
		child_b_.reset(new rz_quadtree_node<T>());
		child_c_.reset(new rz_quadtree_node<T>());
		child_d_.reset(new rz_quadtree_node<T>());
	}

	rz_quadtree_node<T>* child_a() {
		return child_a_.get();
	}

	rz_quadtree_node<T>* child_b() {
		return child_b_.get();
	}

	rz_quadtree_node<T>* child_c() {
		return child_c_.get();
	}

	rz_quadtree_node<T>* child_d() {
		return child_d_.get();
	}

	void set_dimentions(const point2d& origin, double& size) {
		origin_ = origin;
		size_ = size;
	}

	void get_dimentions(point2d& origin, double& size) {
		origin = origin_;
		size = size_;
	}

private:
	std::vector<T> objects_list_;
	bool is_leaf_;
	rz_quadtree_node<T>* parent_;

	std::auto_ptr<rz_quadtree_node<T> > child_a_;
	std::auto_ptr<rz_quadtree_node<T> > child_b_;
	std::auto_ptr<rz_quadtree_node<T> > child_c_;
	std::auto_ptr<rz_quadtree_node<T> > child_d_;

	point2d origin_;
	double size_;
};

} // namespace rimz

#endif // _RZ_QUADTREE_NODE_HPP_INCLUDED_
