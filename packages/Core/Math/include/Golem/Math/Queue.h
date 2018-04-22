/** @file Queue.h
 * 
 * STL-style cyclic buffer with random access iterators.
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
#ifndef _GOLEM_MATH_QUEUE_H_
#define _GOLEM_MATH_QUEUE_H_

#include <vector>
#include <memory.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

template <typename _seq, typename _ptr> void queue_inc(_seq& seq, _ptr& ptr) {
	if (ptr == --seq.end())
		ptr = seq.begin();
	else
		++ptr;
}
template <typename _seq, typename _ptr> void queue_dec(_seq& seq, _ptr& ptr) {
	if (ptr == seq.begin())
		ptr = --seq.end();
	else
		--ptr;
}
template <typename _seq, typename _ptr, typename _diff> _diff queue_inc(_seq& seq, _ptr& ptr, _diff n) {
	const _diff size = seq.size();
	return (n + _diff(ptr - seq.begin()))%size;
}
template <typename _seq, typename _ptr, typename _diff> _diff queue_dec(_seq& seq, _ptr& ptr, _diff n) {
	const _diff size = seq.size();
	return size - 1 - (n + size - 1 - _diff(ptr - seq.begin()))%size;
}


template <typename _Ty> class queue;

template <typename _value_type>
class queue_iterator_base {
public:
	typedef _value_type value_type;
	typedef std::random_access_iterator_tag iterator_category;
	typedef ptrdiff_t difference_type;
	typedef ptrdiff_t distance_type;

	template <typename _Ty> friend class queue;
	
protected:
	typedef std::vector<value_type> seq;
	typedef typename seq::iterator seq_iterator;
	typedef typename seq::const_iterator seq_const_iterator;

	seq* _pseq;
	seq_iterator _begin;
	seq_iterator _ptr;

	inline queue_iterator_base() {
	}
	inline queue_iterator_base(const queue_iterator_base& iterator) : _pseq(iterator._pseq), _begin(iterator._begin), _ptr(iterator._ptr) {
	}
	inline queue_iterator_base(seq* pseq, const seq_iterator& begin, const seq_iterator& ptr) : _pseq(pseq), _begin(begin), _ptr(ptr) {
	}

public:
	inline queue_iterator_base& operator = (const queue_iterator_base& iterator) {
		_pseq = iterator._pseq;
		_begin = iterator._begin;
		_ptr = iterator._ptr;
		return *this;
	}
	inline friend difference_type operator - (const queue_iterator_base &l, const queue_iterator_base &r) {
		return	r._ptr < r._begin && l._begin <= l._ptr ? difference_type(l._ptr - r._ptr) - difference_type(l._pseq->size()) :
				l._ptr < l._begin && r._begin <= r._ptr ? difference_type(l._pseq->size()) - difference_type(r._ptr - l._ptr) :
				difference_type(l._ptr - r._ptr);
	}
	inline friend bool operator < (const queue_iterator_base &l, const queue_iterator_base &r) {
		return	r._ptr < r._begin && l._begin <= l._ptr ? true : l._ptr < l._begin && r._begin <= r._ptr ? false : l._ptr < r._ptr;
	}
	inline friend bool operator == (const queue_iterator_base &l, const queue_iterator_base &r) {
		return l._ptr == r._ptr;
	}
	inline friend bool operator != (const queue_iterator_base &l, const queue_iterator_base &r) {
		return l._ptr != r._ptr;
	}
};

template <typename _value_type>
class queue_const_iterator : public queue_iterator_base<_value_type> {
public:
	typedef _value_type value_type;
	typedef typename queue_iterator_base<value_type>::difference_type difference_type;
	typedef const value_type* pointer;
	typedef const value_type& reference;

	template <typename _Ty> friend class queue;
	
protected:
	typedef typename queue_iterator_base<value_type>::seq seq;
	typedef typename queue_iterator_base<value_type>::seq_iterator seq_iterator;
	typedef typename queue_iterator_base<value_type>::seq_const_iterator seq_const_iterator;

	inline queue_const_iterator(const seq* pseq, const seq_iterator& begin, const seq_iterator& ptr) : queue_iterator_base<_value_type>(const_cast<seq*>(pseq), begin, ptr) {
	}

public:
	inline queue_const_iterator() {
	}
	inline queue_const_iterator(const queue_iterator_base<_value_type>& iterator) : queue_iterator_base<_value_type>(iterator) {
	}
	inline queue_const_iterator& operator = (const queue_iterator_base<_value_type>& iterator) {
		queue_iterator_base<_value_type>::operator = (iterator);
		return *this;
	}
	inline reference operator * () const {
		return *this->_ptr;
	}
	inline pointer operator -> () const {
		return (&**this);
	}
	inline queue_const_iterator& operator ++ () {
		queue_inc(*this->_pseq, this->_ptr);
		return *this;
	}
	inline queue_const_iterator operator ++ (int) {
		queue_const_iterator p(*this);
		++*this;
		return p;
	}
	inline queue_const_iterator& operator -- () {
		queue_dec(*this->_pseq, this->_ptr);
		return *this;
	}
	inline queue_const_iterator operator -- (int) {
		queue_const_iterator p(*this);
		--*this;
		return p;
	}
	inline queue_const_iterator& operator += (difference_type n) {
		this->_ptr = this->_pseq->begin() + queue_inc(*this->_pseq, this->_ptr, n);
		return *this;
	}
	inline friend queue_const_iterator operator + (const queue_const_iterator &p, difference_type n) {
		return queue_const_iterator(p._pseq, p._begin, p._pseq->begin() + queue_inc(*p._pseq, p._ptr, n));
	}
	inline queue_const_iterator& operator -= (difference_type n) {
		this->_ptr = this->_pseq->begin() + queue_dec(*this->_pseq, this->_ptr, n);
		return *this;
	}
	inline friend queue_const_iterator operator - (const queue_const_iterator &p, difference_type n) {
		return queue_const_iterator(p._pseq, p._begin, p._pseq->begin() + queue_dec(*p._pseq, p._ptr, n));
	}
};

template <typename _value_type>
class queue_iterator : public queue_iterator_base<_value_type> {
public:
	typedef _value_type value_type;
	typedef typename queue_iterator_base<value_type>::difference_type difference_type;
	typedef value_type* pointer;
	typedef value_type& reference;

	template <typename _Ty> friend class queue;
	
protected:
	typedef typename queue_iterator_base<value_type>::seq seq;
	typedef typename queue_iterator_base<value_type>::seq_iterator seq_iterator;
	typedef typename queue_iterator_base<value_type>::seq_const_iterator seq_const_iterator;

	inline queue_iterator(seq* pseq, const seq_iterator& begin, const seq_iterator& ptr) : queue_iterator_base<_value_type>(pseq, begin, ptr) {
	}

public:
	inline queue_iterator() {
	}
	inline queue_iterator(const queue_iterator_base<_value_type>& iterator) : queue_iterator_base<_value_type>(iterator) {
	}
	inline queue_iterator& operator = (const queue_iterator_base<_value_type>& iterator) {
		queue_iterator_base<_value_type>::operator = (iterator);
		return *this;
	}
	inline reference operator * () const {
		return *this->_ptr;
	}
	inline pointer operator -> () const {
		return (&**this);
	}
	inline queue_iterator& operator ++ () {
		queue_inc(*this->_pseq, this->_ptr);
		return *this;
	}
	inline queue_iterator operator ++ (int) {
		queue_iterator p(*this);
		++*this;
		return p;
	}
	inline queue_iterator& operator -- () {
		queue_dec(*this->_pseq, this->_ptr);
		return *this;
	}
	inline queue_iterator operator -- (int) {
		queue_iterator p(*this);
		--*this;
		return p;
	}
	inline queue_iterator& operator += (difference_type n) {
		this->_ptr = this->_pseq->begin() + queue_inc(*this->_pseq, this->_ptr, n);
		return *this;
	}
	inline friend queue_iterator operator + (const queue_iterator &p, difference_type n) {
		return queue_iterator(p._pseq, p._begin, p._pseq->begin() + queue_inc(*p._pseq, p._ptr, n));
	}
	inline queue_iterator& operator -= (difference_type n) {
		this->_ptr = this->_pseq->begin() + queue_dec(*this->_pseq, this->_ptr, n);
		return *this;
	}
	inline friend queue_iterator operator - (const queue_iterator &p, difference_type n) {
		return queue_iterator(p._pseq, p._begin, p._pseq->begin() + queue_dec(*p._pseq, p._ptr, n));
	}
};

template <typename _value_type>
class queue {
public:
	typedef _value_type value_type;
	
	typedef typename queue_iterator_base<value_type>::iterator_category iterator_category;
	typedef typename queue_iterator_base<value_type>::difference_type difference_type;
	typedef typename queue_iterator_base<value_type>::distance_type distance_type;

	typedef queue_iterator<value_type> iterator;
	typedef queue_const_iterator<value_type> const_iterator;
	
	typedef typename iterator::pointer pointer;
	typedef typename const_iterator::pointer const_pointer;
	typedef typename iterator::reference reference;
	typedef typename const_iterator::reference const_reference;

protected:
	typedef typename queue_iterator_base<value_type>::seq seq;
	typedef typename queue_iterator_base<value_type>::seq_iterator seq_iterator;
	typedef typename queue_iterator_base<value_type>::seq_const_iterator seq_const_iterator;
	
	size_t _size;
	seq _seq;
	seq_iterator _begin, _end;

public:
	inline queue(size_t size = 0, const_reference val = value_type()) : _size(0), _seq(size + 1, val), _begin(_seq.begin()), _end(_seq.begin()) {
	}

	inline void reserve(size_t size, const_reference val = value_type()) {
		_seq.resize(size + 1, val);
		clear();
	}
	inline void clear() {
		_begin = _seq.begin();
		_end = _seq.begin();
		_size = 0;
	}
	inline size_t capacity() const {
		return _seq.size() - 1;
	}
	inline size_t size() const {
		return _size;
	}
	inline bool empty() const {
		return _size == 0; //begin == end;
	}
	inline bool full() const {
		return size() == capacity();
	}

	inline void push_front(const_reference val) {
		if (full()) pop_back();
		queue_dec(_seq, _begin);
		*_begin = val;
		++_size;
	}
	inline void pop_front() {
		queue_inc(_seq, _begin);
		--_size;
	}
	inline void push_back(const_reference val) {
		if (full()) pop_front();
		*_end = val;
		queue_inc(_seq, _end);
		++_size;
	}
	inline void pop_back() {
		queue_dec(_seq, _end);
		--_size;
	}

	inline void erase(iterator begin, iterator end) {
		iterator i = begin, j = end, __end = this->end();
		difference_type delta = end - begin;

		for (; j != __end; ++i, ++j)
			*i = *j;
		
		// TODO do proper decrement
		while (delta-- > 0) {
			queue_dec(_seq, _end);
			--_size;
		}
	}

	inline reference front() {
		return *_begin;
	}
	inline const_reference front() const {
		return *_begin;
	}
	inline reference back() {
		seq_iterator _back = _end;
		queue_dec(_seq, _back);
		return *_back;
	}
	inline const_reference back() const {
		seq_const_iterator _back = _end;
		queue_dec(_seq, _back);
		return *_back;
	}

	inline iterator begin() {
		return iterator(&_seq, _begin, _begin);
	}
	inline const_iterator begin() const {
		return const_iterator(&_seq, _begin, _begin);
	}
	inline iterator end() {
		return iterator(&_seq, _begin, _end);
	}
	inline const_iterator end() const {
		return const_iterator(&_seq, _begin, _end);
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_QUEUE_H_*/
