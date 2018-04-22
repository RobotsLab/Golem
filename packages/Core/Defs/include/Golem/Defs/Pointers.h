/** @file Pointers.h
 * 
 * Implements smart pointers with reference counting.
 * TODO use BOOST implementation instead
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
#ifndef _GOLEM_DEFS_POINTERS_H_
#define _GOLEM_DEFS_POINTERS_H_

#include <stddef.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reference counter base */
template <typename _Type> class reference_cnt_base {
public:
	typedef _Type Type;

	reference_cnt_base(Type* ptr = NULL) : ptr(ptr), cnt(ptr != NULL ? 1 : 0) {
	}
	
	Type* get() {
		return ptr;
	}
	const Type* get() const {
		return ptr;
	}
	
	void inc() {
		++cnt;
	}
	bool dec() {
		return cnt > 0 && --cnt > 0;
	}

protected:
	Type* ptr;
	size_t cnt;
};

/** Reference counter virtual base */
template <typename _Type> class reference_cnt_vt_base : public reference_cnt_base<_Type> {
public:
	reference_cnt_vt_base(_Type* ptr = NULL) : reference_cnt_base<_Type>(ptr) {
	}
	virtual ~reference_cnt_vt_base() {
	}
};

//------------------------------------------------------------------------------

/** Object pointer release */
struct release_obj {
	template <typename _Type> static void release(_Type* ptr) {
		delete ptr; // safe if null
	}
};

/** Array pointer release */
struct release_arr {
	template <typename _Type> static void release(_Type* ptr) {
		delete [] ptr; // safe if null
	}
};

//------------------------------------------------------------------------------

/** Reference counter */
template <typename _Type, typename _Rel = release_obj> class reference_cnt : public reference_cnt_base<_Type> {
public:
	typedef _Rel Rel;

	reference_cnt(_Type* ptr = NULL) : reference_cnt_base<_Type>(ptr) {
	}
	~reference_cnt() {
		_Rel::release(reference_cnt_base<_Type>::ptr);
	}
};

/** Reference counter virtual */
template <typename _Type, typename _Rel = release_obj> class reference_cnt_vt : public reference_cnt_vt_base<_Type> {
public:
	typedef _Rel Rel;

	reference_cnt_vt(_Type* ptr = NULL) : reference_cnt_vt_base<_Type>(ptr) {
	}
	~reference_cnt_vt() {
		_Rel::release(reference_cnt_base<_Type>::ptr);
	}
};

//------------------------------------------------------------------------------

/** Object pointer wrapper with reference counting. */
template <typename _Type, typename _Cnt = reference_cnt<_Type> > class shared_ptr {
public:
	typedef _Type Type;
	typedef _Cnt Cnt;

	/** Explicitly constructs from object pointer.
	*
	* Constructor cannot be used for implicit object construction.
	*
	* @param ptr	object pointer
	*/
	explicit shared_ptr(_Type *ptr = NULL) : cnt(NULL) {
		create(ptr);
	}

	/** Explicitly constructs from object pointer counter.
	*
	* Constructor cannot be used for implicit object construction.
	*
	* @param ptr	object pointer counter
	*/
	explicit shared_ptr(_Cnt *cnt) : cnt(cnt) {
	}

	/** Constructs assuming pointer from ptr shared_ptr
	 * 
	 * @param ptr	object reference
	 */
	shared_ptr(const shared_ptr<_Type, _Cnt>& ptr) {
		acquire(ptr.cnt);
	}

	/** Destroys the object.
	 */
	~shared_ptr() {
		release();
	}

	/** Assigns ptr.
	 */
	shared_ptr<_Type, _Cnt>& operator = (const shared_ptr<_Type, _Cnt>& ptr) {
		if (this != &ptr) {
			release();
			acquire(ptr.cnt);
		}
		return *this;
	}

	/** Returns array object reference.
	 */
	_Type& operator [] (size_t index) const {
		return cnt->get()[index];
	}

	/** Returns designated value.
	 */
	_Type &operator * () const {
		return *cnt->get();
	}

	/** Returns pointer to class object.
	 */
	_Type *operator -> () const {
		return cnt->get();
	}
	
	/** Returns wrapped pointer.
	 */
	_Type* get() const {
		return cnt != NULL ? cnt->get() : NULL;
	}

	/** Releases designated object and store a new pointer.
	 */
	void reset(_Type* ptr = NULL) {
		release();
		create(ptr);
	}
	
	/** Releases designated object and store a new pointer counter.
	*/
	void reset(_Cnt* cnt) {
		release();
		this->cnt = cnt;
	}

	/** Releases and decrement pointer counter.
	 */
	void release() {
		if (cnt != NULL) {
			if (!cnt->dec())
				delete cnt;
			cnt = NULL;
		}
	}

	/** Performs dynamic_cast pointer conversion.
	 * 
	 * @param r	converted pointer
	 */
	template <typename _Left, typename _Right> friend _Left dynamic_pointer_cast(const _Right& r);

	/** Performs static_cast pointer conversion.
	 * 
	 * @param r	converted pointer
	 */
	template <typename _Left, typename _Right> friend _Left static_pointer_cast(const _Right& r);

	/** Performs const_cast pointer conversion.
	 * 
	 * @param r	converted pointer
	 */
	template <typename _Left, typename _Right> friend _Left const_pointer_cast(const _Right& r);

private:
	/** Object pointer counter */
	_Cnt* cnt;
	
	/** Create a new pointer counter.
	 * 
	 * @param ptr	object pointer
	 */
	void create(_Type* ptr) {
		if (ptr != NULL)
			cnt = new _Cnt(ptr);
	}
	
	/** Acquire and increment a new pointer counter.
	 * 
	 * @param cnt	acquired pointer counter
	 */
	void acquire(_Cnt* cnt) {
		this->cnt = cnt;
		if (cnt != NULL)
			cnt->inc();
	}
};

template <typename _Left, typename _Right> _Left dynamic_pointer_cast(const _Right& r) {
	_Left ptr;

	if (dynamic_cast<typename _Left::Type*>(r.get()) != NULL) {
		ptr.cnt = (typename _Left::Cnt*)r.cnt;
		ptr.cnt->inc();
	}

	return ptr;
}

template <typename _Left, typename _Right> _Left static_pointer_cast(const _Right& r) {
	_Left ptr;

	if (static_cast<typename _Left::Type*>(r.get()) != NULL) {
		ptr.cnt = (typename _Left::Cnt*)r.cnt;
		ptr.cnt->inc();
	}

	return ptr;
}

template <typename _Left, typename _Right> _Left const_pointer_cast(const _Right& r) {
	_Left ptr;

	if (const_cast<typename _Left::Type*>(r.get()) != NULL) {
		ptr.cnt = (typename _Left::Cnt*)r.cnt;
		ptr.cnt->inc();
	}

	return ptr;
}

template <typename _Left, typename _LeftCnt, typename _Right, typename _RightCnt> bool operator == (const shared_ptr<_Left, _LeftCnt>& l, const shared_ptr<_Right, _RightCnt>& r) {
	return l.get() == r.get();
}

template <typename _Type, typename _Cnt> bool operator == (const shared_ptr<_Type, _Cnt>& l, const void* r) {
	return l.get() == r;
}

template <typename _Type, typename _Cnt> bool operator == (const void* l, const shared_ptr<_Type, _Cnt>& r) {
	return l == r.get();
}

template <typename _Left, typename _LeftCnt, typename _Right, typename _RightCnt> bool operator != (const shared_ptr<_Left, _LeftCnt>& l, const shared_ptr<_Right, _RightCnt>& r) {
	return l.get() != r.get();
}

template <typename _Type, typename _Cnt> bool operator != (const shared_ptr<_Type, _Cnt>& l, const void* r) {
	return l.get() != r;
}

template <typename _Type, typename _Cnt> bool operator != (const void* l, const shared_ptr<_Type, _Cnt>& r) {
	return l != r.get();
}

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_DEFS_POINTERS_H_*/
