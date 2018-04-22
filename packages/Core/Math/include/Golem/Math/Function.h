/** @file Function.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_MATH_FUNCTION_H_
#define _GOLEM_MATH_FUNCTION_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Math.h>

namespace golem {

//------------------------------------------------------------------------------

/** Closed finite real interval */
template <typename REAL>
class _Interval {
public:
	typedef shared_ptr< _Interval<REAL> > Ptr;

	REAL left, right;

	_Interval(REAL left, REAL right) : left(left), right(right) {
	}
	_Interval(const _Interval<REAL> &interval) : left(interval.left), right(interval.right) {
	}

	void setLeft(REAL left) {
		this->left = left;
	}
	REAL getLeft() {
		return left;
	}
	const REAL getLeft() const {
		return left;
	}

	void setRight(REAL right) {
		this->right = right;
	}
	REAL getRight() {
		return right;
	}
	const REAL getRight() const {
		return right;
	}

	void set(REAL left, REAL right) {
		this->left = left;
		this->right = right;
	}
	REAL get() const {
		return right - left;
	}
	REAL min() const {
		return left < right ? left : right;
	}
	REAL max() const {
		return left < right ? right : left;
	}

	bool contains(REAL x) const {
		return x >= left && x <= right;
	}
	bool increasing() const {
		return left < right;
	}
	bool nondecreasing() const {
		return left <= right;
	}
	bool decreasing() const {
		return left > right;
	}
	bool nonincreasing() const {
		return left >= right;
	}

	void swap() {
		std::swap(left, right);
	}

	const _Interval<REAL>& operator = (const _Interval<REAL> &interval) {
		left = interval.left;
		right = interval.right;
		return *this;
	}
};

//------------------------------------------------------------------------------

/** Single variable function on real interval */
template <typename REAL>
class _Function {
public:
	typedef shared_ptr< _Function<REAL> > Ptr;

	virtual ~_Function() {}
	
	virtual REAL get(REAL x) const = 0;
};

//------------------------------------------------------------------------------

template <typename REAL>
class _StdFunc : public _Function<REAL> {
public:
	typedef REAL (*FuncPtr)(REAL);

protected:
	FuncPtr func;

public:
	_StdFunc(FuncPtr func) : func(func) {
	}
	
	virtual REAL get(REAL x) const {
		return func(x);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_FUNCTION_H_*/
