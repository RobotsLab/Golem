/** @file Collection.h
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
#ifndef _GOLEM_MATH_COLLECTION_H_
#define _GOLEM_MATH_COLLECTION_H_

//------------------------------------------------------------------------------

#include <vector>
#include <list>
#include <map>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** List-like object container (preserving the order of inserted elements) with fast elament access and open access to values. */
template <typename _Key, typename _Val>
class PublicList {
public:
	typedef _Key Key;
	typedef _Val Val;
	typedef std::pair<Key, Val> Pair;
	
	typedef std::list<Pair> List; // must not invalidate iterators after random removal
	typedef typename List::iterator iterator;
	typedef typename List::const_iterator const_iterator;

protected:
	typedef std::pair<Key, iterator> MapPair;
	typedef std::map<Key, iterator> Map;
	
	List list;
	Map map;

public:
	/** Default constructor does not do any initialisation. */
	PublicList() {}

	/**	Copy constructor. */
	PublicList(const PublicList &list) {
		*this = list;
	}

	~PublicList() {
		clear();
	}

	/** Finds the key on the list. */
	inline const_iterator find(const Key& key) const {
		typename Map::const_iterator pos = map.find(key);
		return pos != map.end() ? pos->second : end();
	}

	/** Finds the key on the list. */
	inline iterator find(const Key& key) {
		typename Map::iterator pos = map.find(key);
		return pos != map.end() ? pos->second : end();
	}

	/** Checks if the key is on the list. */
	inline bool contains(const Key& key) const {
		return map.find(key) != map.end();
	}

	/** Adds data at the front. */
	inline void push_front(const Pair &pair) {
		if (!contains(pair.first)) {
			list.push_front(pair);
			map.insert(MapPair(pair.first, list.begin()));
		}
	}
	/** Adds data at the back. */
	inline void push_back(const Pair &pair) {
		if (!contains(pair.first)) {
			list.push_back(pair);
			map.insert(MapPair(pair.first, --list.end()));
		}
	}

	/** erases the key. */
	inline void erase(const Key& key) {
		typename Map::iterator pos = map.find(key);
		if (pos != map.end()) {
			map.erase(pos);
			list.erase(pos->second);
		}
	}
	/** erases the pointer. */
	inline void erase(iterator pos) {
		if (pos != list.end()) {
			map.erase(pos->first);
			list.erase(pos);
		}
	}
	
	/** Clears the list. */
	inline void clear() {
		map.clear();
		list.clear();
	}
	
	/** Number of items. */
	inline size_t size() const {
		return list.size();
	}

	/** Checks if empty. */
	inline bool empty() const {
		return list.empty();
	}
	
	/** PublicList begin */
	inline const_iterator begin() const {
		return list.begin();
	}
	/** PublicList begin */
	inline iterator begin() {
		return list.begin();
	}
	/** PublicList end */
	inline const_iterator end() const {
		return list.end();
	}
	/** PublicList end */
	inline iterator end() {
		return list.end();
	}
	
	/** PublicList front element */
	inline Key front() const {
		return list.front();
	}
	/** PublicList front element */
	inline Key front() {
		return list.front();
	}
	/** PublicList back element */
	inline Key back() const {
		return list.back();
	}
	/** PublicList back element */
	inline Key back() {
		return list.back();
	}

	/**	Assignment operator. */
	inline PublicList& operator = (const PublicList<Key, Val>& list) {
		// clear data
		clear();

		for (typename PublicList<Key, Val>::List::const_iterator i = list.list.begin(); i != list.list.end(); ++i) {
			this->list.push_back(*i);
			this->map.insert(MapPair(i->first, --this->list.end()));
		}
		
		return *this;
	}
};

//------------------------------------------------------------------------------

/** List-like object container (preserving the order of inserted elements) with fast elament access and limited access to values. */
template <typename _Key, typename _Val>
class PrivateList {
public:
	typedef _Key Key;
	typedef _Val Val;
	typedef std::pair<Key, Val> Pair;
	
	typedef std::list<Key> List; // must not invalidate iterators after random removal
	typedef typename List::iterator iterator;
	typedef typename List::const_iterator const_iterator;

protected:
	struct Data {
		Val val;
		iterator pos;

		inline Data() {}
		inline Data(const Val& val, iterator pos) : val(val), pos(pos) {}
	};
	typedef std::pair<Key, Data> MapPair;
	typedef std::map<Key, Data> Map;
	
	List list;
	Map map;

public:
	/** Default constructor does not do any initialisation. */
	PrivateList() {}

	/**	Copy constructor. */
	PrivateList(const PrivateList &list) {
		*this = list;
	}

	~PrivateList() {
		clear();
	}

	/** Finds the key on the list. */
	inline const_iterator find(const Key& key) const {
		typename Map::const_iterator pos = map.find(key);
		return pos != map.end() ? pos->second.pos : end();
	}

	/** Finds the key on the list. */
	inline iterator find(const Key& key) {
		typename Map::iterator pos = map.find(key);
		return pos != map.end() ? pos->second.pos : end();
	}

	/** Checks if the key is on the list. */
	inline bool contains(const Key& key) const {
		return map.find(key) != map.end();
	}

	/** Adds data at the front. */
	inline void push_front(const Pair &pair) {
		if (!contains(pair.first)) {
			list.push_front(pair.first);
			map.insert(MapPair(pair.first, Data(pair.second, list.begin())));
		}
	}
	/** Adds data at the back. */
	inline void push_back(const Pair &pair) {
		if (!contains(pair.first)) {
			list.push_back(pair.first);
			map.insert(MapPair(pair.first, Data(pair.second, --list.end())));
		}
	}
	/** Adds data at the given position. */
	inline void insert(iterator pos, const Pair &pair) {
		if (!contains(pair.first)) {
			list.insert(pos, pair.first);
			map.insert(MapPair(pair.first, Data(pair.second, --list.end())));
		}
	}

	/** erases the key. */
	inline void erase(const Key& key) {
		// remove() can be called from itself (an type calls other types destructors)
		typename Map::iterator pos = map.find(key);
		if (pos != map.end()) {
			list.erase(pos->second.pos);
			// this works only because map first removes the entry and then the type
			// (the other way round can be disasterous)
			map.erase(pos);
		}
	}
	/** erases the pointer. */
	inline void erase(iterator pos) {
		if (pos != list.end())
			erase(*pos);
	}
	
	/** Clears the list. */
	inline void clear() {
		list.clear();
		map.clear();
	}
	
	/** Number of items. */
	inline size_t size() const {
		return list.size();
	}

	/** Checks if empty. */
	inline bool empty() const {
		return list.empty();
	}
	
	/** PrivateList begin */
	inline const_iterator begin() const {
		return list.begin();
	}
	/** PrivateList begin */
	inline iterator begin() {
		return list.begin();
	}
	/** PrivateList end */
	inline const_iterator end() const {
		return list.end();
	}
	/** PrivateList end */
	inline iterator end() {
		return list.end();
	}
	
	/** PrivateList front element */
	inline Key front() const {
		return list.front();
	}
	/** PrivateList front element */
	inline Key front() {
		return list.front();
	}
	/** PrivateList back element */
	inline Key back() const {
		return list.back();
	}
	/** PrivateList back element */
	inline Key back() {
		return list.back();
	}

	/**	Assignment operator. */
	inline PrivateList& operator = (const PrivateList<Key, Val>& list) {
		// clear data
		clear();

		for (typename PrivateList<Key, Val>::Map::const_iterator i = list.map.begin(); i != list.map.end(); ++i) {
			this->list.push_back(i->first);
			this->map.insert(MapPair(i->first, Data(i->second.val, --this->list.end())));
		}
		
		return *this;
	}
};

//------------------------------------------------------------------------------

template <typename Key, typename Type, typename Compare = std::less<Key> >
class Rank : public std::map<Key, Type, Compare> {
public:
	/** Base class */
	typedef std::map<Key, Type, Compare> Base;

protected:
	/** Collection capacity */
	size_t n;

public:
	/** Constructs collection with the specified capacity */
	inline Rank(size_t n = 1) {
		this->n = std::max((size_t)1, n);
	}

	/** Returns collection capacity */
	inline size_t capacity() const {
		return n;
	}
	
	/** Sets collection capacity */
	inline void reserve(size_t n) {
		this->n = std::max((size_t)1, n);
	}

	/** Compacts collection to the default capacity */
	inline void compact() {
		size_t size = Base::size();
		typename Base::iterator end = Base::end();
		while (size-- > n) --end;
		Base::erase(end, Base::end());
	}

	/** Inserts a new element */
	inline std::pair<typename Base::iterator, bool> insert(const typename Base::value_type& item) {
		if (Base::size() < n)
			return Base::insert(item);

		// test if insertion makes sense
		typename Base::iterator last = --Base::end();
		if (Compare()(item.first, last->first)) {
			Base::erase(last);
			return Base::insert(item);
		}

		return std::pair<typename Base::iterator, bool>(Base::end(), false);
	}

	/** Inserts a new element */
	inline std::pair<typename Base::iterator, bool> insert(const typename Base::key_type& key, const typename Base::mapped_type& value) {
		return insert(typename Base::value_type(key, value));
	}
};

//------------------------------------------------------------------------------

/** Limited capacity sorted collection with unique class identifiers */
template <typename Key, typename Class, typename Type, typename Compare = std::less<Key> >
class ClassRank : public std::map<Key, std::pair<Class, Type>, Compare> {
public:
	/** Base class */
	typedef std::map<Key, std::pair<Class, Type>, Compare> Base;
	/** Classes */
	typedef std::map<Class, typename Base::iterator> Classes;

protected:
	/** Collection capacity */
	size_t n;
	/** Classes */
	Classes classes;

	inline std::pair<typename Base::iterator, bool> _insert(const typename Base::value_type& item) {
		std::pair<typename Base::iterator, bool> ret = Base::insert(item);
		if (ret.second)
			(void)classes.insert(typename Classes::value_type(item.second.first, ret.first));
		return ret;
	}

public:
	/** Constructs collection with the specified capacity */
	inline ClassRank(size_t n = 1) {
		reserve(n);
	}

	/** Classes */
	const Classes& getClasses() const {
		return classes;
	}

	/** Returns collection capacity */
	inline size_t capacity() const {
		return n;
	}
	
	/** Sets collection capacity */
	inline void reserve(size_t n) {
		this->n = std::max((size_t)1, n);
	}

	/** Erase */
	inline void erase(typename Base::iterator position) {
		if (position != Base::end()) {
			classes.erase(position->second.first);
			Base::erase(position);
		}
	}

	/** Erase */
	inline void erase(typename Base::iterator first, typename Base::iterator last) {
		while (first != last)  // TODO make it faster
			erase(first++);
	}

	/** Clear */
	inline void clear() {
		Base::clear();
		classes.clear();
	}

	/** Compacts collection to the default capacity */
	inline void compact() {
		size_t size = Base::size();
		while (size-- > n)   // TODO make it faster
			erase(--Base::end());
	}

	/** Inserts a new element */
	inline std::pair<typename Base::iterator, bool> insert(const typename Base::value_type& item) {
		// is the class in the collection
		typename Classes::iterator position = classes.find(item.second.first);
		if (position == classes.end()) {
			// always insert new class
			if (Base::size() < n)
				return _insert(item);
			
			// remove the other class and insert the new one
			typename Base::iterator last = --Base::end();
			if (Compare()(item.first, last->first)) {
				erase(last);
				return _insert(item);
			}
		}
		else {
			// replace stored data within the same class
			if (Compare()(item.first, position->second->first)) {
				erase(position->second);
				return _insert(item);
			}
		}
		
		return std::pair<typename Base::iterator, bool>(Base::end(), false);
	}

	/** Inserts a new element */
	inline std::pair<typename Base::iterator, bool> insert(const typename Base::key_type& key, const typename Base::mapped_type& value) {
		return insert(typename Base::value_type(key, value));
	}

	/** Inserts a new element */
	inline std::pair<typename Base::iterator, bool> insert(const typename Base::key_type& key, const typename Base::mapped_type::first_type& first, const typename Base::mapped_type::second_type& second) {
		return insert(typename Base::value_type(key, typename Base::mapped_type(first, second)));
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_COLLECTION_H_*/
