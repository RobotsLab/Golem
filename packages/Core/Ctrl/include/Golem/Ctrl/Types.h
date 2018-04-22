/** @file Types.h
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
#ifndef _GOLEM_CTRL_TYPES_H_
#define _GOLEM_CTRL_TYPES_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Sys/Timer.h>
#include <Golem/Math/Math.h>
#include <Golem/Math/Twist.h>
#include <Golem/Math/Quat.h>
#include <Golem/Sys/Message.h>
#include <stdint.h>
#include <vector>
#include <algorithm>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgSpace, Message)
MESSAGE_DEF(MsgSpaceIndex, MsgSpace)
MESSAGE_DEF(MsgSpaceRange, MsgSpace)

//------------------------------------------------------------------------------

/* SI Units:
 * - position/angle: meter [m], radian [rad]
 * - time: second [sec]
 * - velocity: [m/sec], [rad/sec]
 * - acceleration: [m/sec*sec], [rad/sec*sec]
 * - weight: kilogram [kg]
 * - force: Newton [N]
 */

//------------------------------------------------------------------------------

/** Type of dimension index */
typedef intptr_t idx_t;
/** Type of dimension index difference */
typedef ptrdiff_t idxdiff_t;

/** Space with a maximum number of dimensions DIM */
template <idx_t _DIM, size_t _HASH = 0> class Space {
public:
	/** Maximum number of dimensions */
	static const idx_t DIM = _DIM;
	/** Hashing parameter */
	static const idx_t HASH = _HASH;

	/** Space name */
	static const char* Name;

	/** Space name */
	static const char* getName() {
		return Name;
	}

	class Range;

	/** Space variable index (iterator bound to a space, NOT a type) */
	class Index {
	protected:
		friend class Range;

		/** index */
		idx_t _index;

	public:
		/** Undefined index */
		static const idx_t UNDEF = golem::numeric_const<idx_t>::MAX;

		/** No initialisation */
		inline explicit Index(idx_t index = UNDEF) {
			set(index);
		}
		/** Index copy constructor */
		inline Index(const Index& index) : _index(index._index) {
		}

		/** Initialisation */
		inline void set(idx_t index = UNDEF) {
			if ((index < 0 || index > DIM) && index != UNDEF)
				throw MsgSpaceIndex(Message::LEVEL_ERROR, "%s::Index::set(): invalid variable index %i", getName(), index);
			_index = index;
		}

		/** Index assignment */
		inline Index& operator = (const Index& index) {
			this->_index = index._index;
			return *this;
		}
		
		/** Retrieves index as integer */
		inline idx_t operator * () const {
			return this->_index;
		}

		inline Index& operator ++ () {
			++this->_index;
			return *this;
		}
		inline Index operator ++ (int) {
			Index index(*this);
			++*this;
			return index;
		}
		inline Index& operator -- () {
			--this->_index;
			return *this;
		}
		inline Index operator -- (int) {
			Index index(*this);
			--*this;
			return index;
		}

		inline friend idxdiff_t operator - (const Index &l, const Index &r) {
			return	idxdiff_t(l._index - r._index);
		}
		
		inline friend Index operator + (const Index &index, idxdiff_t n) {
			return Index(index._index + n);
		}
		inline friend Index operator - (const Index &index, idxdiff_t n) {
			return Index(index._index - n);
		}

		inline friend idxdiff_t operator + (idxdiff_t n, const Index &index) {
			return n + index._index;
		}
		inline friend idxdiff_t operator - (idxdiff_t n, const Index &index) {
			return n - index._index;
		}

		inline Index& operator += (idxdiff_t n) {
			this->_index += n;
			return *this;
		}
		inline Index& operator -= (idxdiff_t n) {
			this->_index -= n;
			return *this;
		}

		inline friend bool operator <  (const Index &l, const Index &r) {
			return	l._index <  r._index;
		}
		inline friend bool operator <  (const Index &l, idx_t r) {
			return	l._index <  r;
		}
		inline friend bool operator <  (idx_t l, const Index &r) {
			return	l <  r._index;
		}
		inline friend bool operator >  (const Index &l, const Index &r) {
			return	l._index >  r._index;
		}
		inline friend bool operator >  (const Index &l, idx_t r) {
			return	l._index >  r;
		}
		inline friend bool operator >  (idx_t l, const Index &r) {
			return	l >  r._index;
		}
		inline friend bool operator <= (const Index &l, const Index &r) {
			return	l._index <= r._index;
		}
		inline friend bool operator <= (const Index &l, idx_t r) {
			return	l._index <= r;
		}
		inline friend bool operator <= (idx_t l, const Index &r) {
			return	l <= r._index;
		}
		inline friend bool operator >= (const Index &l, const Index &r) {
			return	l._index >= r._index;
		}
		inline friend bool operator >= (const Index &l, idx_t r) {
			return	l._index >= r;
		}
		inline friend bool operator >= (idx_t l, const Index &r) {
			return	l >= r._index;
		}
		inline friend bool operator == (const Index &l, const Index &r) {
			return l._index == r._index;
		}
		inline friend bool operator == (const Index &l, idx_t r) {
			return l._index == r;
		}
		inline friend bool operator == (idx_t l, const Index &r) {
			return l == r._index;
		}
		inline friend bool operator != (const Index &l, const Index &r) {
			return l._index != r._index;
		}
		inline friend bool operator != (const Index &l, idx_t r) {
			return l._index != r;
		}
		inline friend bool operator != (idx_t l, const Index &r) {
			return l != r._index;
		}
	};

	/** Space variable range */
	class Range {
	protected:
		/** Range limits */
		Index _begin, _end;
		/** Range size */
		idx_t _size;

	public:
		/** Full range */
		inline Range() : _begin(0), _end(DIM), _size(DIM) {
		}
		/** Initialisation */
		inline Range(idx_t begin, intptr_t size = 0) {
			set(begin, size);
		}
	
		/** Range begin */
		inline const Index& begin() const {
			return _begin;
		}
		/** Range end */
		inline const Index& end() const {
			return _end;
		}
		/** Range size */
		inline idx_t size() const {
			return _size;
		}
		/** Tests if index is within the range */
		inline bool contains(const Index& index) const {
			return _begin <= index && _end > index;
		}

		/** Initialisation */
		inline void set(idx_t begin, idx_t size = 0) {
			if (begin >= DIM || size > DIM - begin)
				throw MsgSpaceRange(Message::LEVEL_ERROR, "%s::Range::set(): variable index too large %i > %i", getName(), begin >= DIM ? begin : begin + size, DIM);
			_begin._index = begin;
			_size = size;
			_end._index = begin + size;
		}
		/** Adds size */
		inline void add(idx_t size) {
			set(_begin._index, _size + size);
		}

		inline friend bool operator == (const Range &l, const Range &r) {
			return l._begin == r._begin && l._end == r._end;
		}
		inline friend bool operator != (const Range &l, const Range &r) {
			return l._begin != r._begin || l._end != r._end;
		}
	};

	/** Space variable map */
	typedef std::vector<Index> Map;

	/** Space type-valued coordinates. */
	// TODO indices specialised to Coord<_Type> derived from generic Space<DIM>::Index
	template <typename _Type> class Coord {
	protected:
		/** Data */
		_Type _data[DIM];

	public:
		/** Type */
		typedef _Type Type;

		/**	Data pointer. */
		inline _Type* data() {
			return _data;
		}
		/**	Data pointer. */
		inline const _Type* data() const {
			return _data;
		}

		/**	Get from range */
		template <typename _Ptr> void get(_Ptr begin, _Ptr end, idx_t offset = 0) const {
			for (_Ptr i = begin; i < end && offset < DIM; ++i)
				*i = _data[offset++];
		}
		/**	Get from range */
		void get(const Index& begin, const Index& end, Coord& coord) const {
			for (Index i = begin; i < end; ++i)
				coord._data[*i] = _data[*i];
		}
		/**	Get from range */
		void get(const Range& range, Coord& coord) const {
			get(range.begin(), range.end(), coord);
		}
		/**	Set from range */
		template <typename _Ptr> void set(_Ptr begin, _Ptr end, idx_t offset = 0) {
			for (_Ptr i = begin; i < end && offset < DIM; ++i)
				_data[offset++] = *i;
		}
		/**	Set from range */
		void set(const Index& begin, const Index& end, const Coord& coord) {
			for (Index i = begin; i < end; ++i)
				_data[*i] = coord._data[*i];
		}
		/**	Set from range */
		void set(const Range& range, const Coord& coord) {
			set(range.begin(), range.end(), coord);
		}

		/**	Fill entire range */
		template <typename _Ty> void fill(const _Ty& val) {
			std::fill(_data, _data + DIM, val);
		}
		/**	Fill from range */
		template <typename _Ty> void fill(const Index& begin, const Index& end, const _Ty& val) {
			std::fill(_data + *begin, _data + *end, val);
		}
		/**	Fill from range */
		template <typename _Ty> void fill(const Range& range, const _Ty& val) {
			fill(range.begin(), range.end(), val);
		}

		/**	Array subscript operator. */
		inline _Type &operator [] (const Index& index) {
			return _data[*index];
		}
		/**	Array subscript operator. */
		inline const _Type &operator [] (const Index& index) const {
			return _data[*index];
		}
	};
};

//------------------------------------------------------------------------------

/** Chain space consists of kinematic chains - default value: 18 = 2 x (1 arm + 5 fingers' hand) + 3 head + 2 x (1 leg) + 1 torso */
typedef Space<18> Chainspace;

/** Configuration space consists of coordinates of kinematic chains - default max value: 70 = 2 x (7-DOF arm + 5 fingers x 4-DOF each) + 7 head + 2 x (4-DOF leg) + 1 torso */
typedef Space<70> Configspace;

/** Reserved space size in bytes */
typedef Space<sizeof(Real)*(5*Chainspace::DIM + 5*Configspace::DIM) + 100> Reservedspace;

//------------------------------------------------------------------------------

/** Scalar coordinates. */
template <typename Type, typename Space> class ScalarCoord : public Space::template Coord<Type> {
public:
	/** This */
	typedef ScalarCoord<Type, Space> This;
	/** Sequence of coordinates */
	typedef std::vector<This> Seq;
	/** Space range */
	typedef typename Space::Range Range;
	/** Space index */
	typedef typename Space::Index Index;
	/** Space map */
	typedef typename Space::Map Map;

	/**	Sets coordinates to the specified value. */
	void setToDefault(Index begin, Index end) {
		Space::template Coord<Type>::fill(begin, end, numeric_const<Type>::ZERO);
	}
	/**	Sets coordinates to the specified value. */
	void setToDefault(const Range& range) {
		setToDefault(range.begin(), range.end());
	}
};

/** Real-valued coordinates. */
template <typename Space> class RealCoord : public ScalarCoord<Real, Space> {
public:
	/** Base */
	typedef ScalarCoord<Real, Space> Base;
	/** This */
	typedef RealCoord<Space> This;
	/** Sequence of coordinates */
	typedef std::vector<This> Seq;
	/** Space range */
	typedef typename Space::Range Range;
	/** Space index */
	typedef typename Space::Index Index;
	/** Space map */
	typedef typename Space::Map Map;

	/** tests for valid object */
	bool isValid(Index begin, Index end) const {
		for (Index i = begin; i < end; ++i)
			if (!Math::isFinite(Base::operator [] (i)))
				return false;
		return true;
	}
	/** tests for valid object */
	bool isValid(const Range& range) const {
		return isValid(range.begin(), range.end());
	}

	/** Comparison */
	bool equals(const This& config, const Range& range, Real eps = numeric_const<Real>::EPS) const {
		for (Index i = range.begin(); i != range.end(); ++i)
			if (!Math::equals(Base::operator [] (i), config[i], eps))
				return false;
		return true;
	}


	/** this = a - b 	*/
	void subtract(const Map& map, const This& a, const This& b) {
		for (typename Map::const_iterator i = map.begin(); i != map.end(); ++i)
			Base::operator [] (*i) = a[*i] - b[*i];
	}

	/** this = a * b 	*/
	Real dot(const Map& map, const This& a) {
		Real s = numeric_const<Real>::ZERO;
		for (typename Map::const_iterator i = map.begin(); i != map.end(); ++i)
			s += Base::operator [] (*i) * a[*i];
		return s;
	}

	/** Magnitude */
	Real magnitudeSqr(const Map& map) const {
		Real m = numeric_const<Real>::ZERO;
		for (typename Map::const_iterator i = map.begin(); i != map.end(); ++i)
			m += Math::sqr(Base::operator [] (*i));
		return m;
	}
};

/** Object-valued coordinates. */
template <typename Object, typename Space> class ObjectCoord : public Space::template Coord<Object> {
public:
	/** This */
	typedef ObjectCoord<Object, Space> This;
	/** Sequence of coordinates */
	typedef std::vector<This> Seq;
	/** Space range */
	typedef typename Space::Range Range;
	/** Space index */
	typedef typename Space::Index Index;

	/**	Sets space coordinates to the specified value. */
	void setToDefault(Index begin, Index end) {
		for (Index i = begin; i < end; ++i)
			Space::template Coord<Object>::operator [] (i).setToDefault();
	}
	/**	Sets space coordinates to the specified value. */
	void setToDefault(const Range& range) {
		setToDefault(range.begin(), range.end());
	}
	/** tests for valid object */
	bool isValid(Index begin, Index end) const {
		for (Index i = begin; i < end; ++i)
			if (!Space::template Coord<Object>::operator [] (i).isValid())
				return false;
		return true;
	}
	/** tests for valid object */
	bool isValid(const Range& range) const {
		return isValid(range.begin(), range.end());
	}
};

/** Chainspace coordinates. */
typedef RealCoord<Chainspace> ChainspaceCoord;

/** Configspace coordinates. */
typedef RealCoord<Configspace> ConfigspaceCoord;

/** Rigid body coordinates (per chain). */
typedef ObjectCoord<Mat34, Chainspace> WorkspaceChainCoord;

/** Rigid body coordinates (per joint). */
typedef ObjectCoord<Mat34, Configspace> WorkspaceJointCoord;

/** Reserved area coordinates */
typedef ScalarCoord<U8, Reservedspace> ReservedCoord;

//------------------------------------------------------------------------------

/** Coordinates state variable. */
template <typename Coord> class CoordState : public Coord {
public:
	/** This */
	typedef CoordState<Coord> This;
	/** Trajectory */
	typedef std::vector<This> Seq;

	/** Time stamp */
	SecTmReal t;

	/** Initialise with a given stamp */
	CoordState(SecTmReal t = SEC_TM_REAL_ZERO) : t(t) {
	}
	/** Initialise with given parameters */
	CoordState(const Coord& c, SecTmReal t) : Coord(c), t(t) {
	}

	/** Comparison operator */
	bool operator < (const CoordState& s) const {
		return this->t < s.t;
	}
	/** Friend comparison operator */
	friend bool operator < (const CoordState& s, SecTmReal t) {
		return s.t < t;
	}
	/** Friend comparison operator */
	friend bool operator < (SecTmReal t, const CoordState& s) {
		return t < s.t;
	}
	/** Comparison operator */
	bool operator <= (const CoordState& s) const {
		return this->t <= s.t;
	}
	/** Friend comparison operator */
	friend bool operator <= (const CoordState& s, SecTmReal t) {
		return s.t <= t;
	}
	/** Friend comparison operator */
	friend bool operator <= (SecTmReal t, const CoordState& s) {
		return t <= s.t;
	}
	/** Shifts trajectory in time */
	template <typename Ptr> static void shift(SecTmReal timeShift, Ptr begin, Ptr end) {
		for (Ptr i = begin; i != end; ++i)
			i->t += timeShift;
	}

	/* Returns dest pointers to the range of items begin, ..., end such that:
	* *destBegin <= *srcBegin, *(destBegin + 1) > *srcBegin, ..., *(destEnd - 1) <= *srcEnd, *destEnd > *srcEnd
	*/
	template <typename _Ptr, typename _Cmp> static void lookup(_Ptr begin, _Ptr end, _Cmp srcBegin, _Cmp srcEnd, _Ptr& destBegin, _Ptr& destEnd) {
		if (begin == end) {
			destBegin = destEnd = end;
			return;
		}

		destBegin = std::lower_bound(begin, end, srcBegin);
		if (destBegin != begin)
			--destBegin;
		destEnd = std::upper_bound(begin, end, srcEnd);
		if (destEnd != end)
			++destEnd;
	}

	/* Returns dest pointers to the range of items containing src
	*/
	template <typename _Ptr, typename _Cmp> static bool lookup(_Ptr begin, _Ptr end, _Cmp src, _Ptr& destBegin, _Ptr& destEnd) {
		lookup(begin, end, src, src, destBegin, destEnd);
		
		if (destBegin == destEnd)
			return false;

		--destEnd;

		return true;
	}
};

//------------------------------------------------------------------------------

/** Generalised 1D coordinates. */
class GenCoord {
public:
	/** position */
	Real pos;
	/** velocity */
	Real vel;
	/** acceleration */
	Real acc;

	/** Default constructor does not do anything */
	GenCoord() {
	}
	/** Constructs GenCoord with specified parameters */
	GenCoord(Real pos, Real vel, Real acc) {
		set(pos, vel, acc);
	}
	/** sets specified values */
	void set(Real pos = REAL_ZERO, Real vel = REAL_ZERO, Real acc = REAL_ZERO) {
		this->pos = pos;
		this->vel = vel;
		this->acc = acc;
	}
	/** sets default value */
	void setToDefault() {
		set(REAL_ZERO, REAL_ZERO, REAL_ZERO);
	}
	/** tests for valid vector */
	bool isValid() const {
		if (!Math::isFinite(pos) || !Math::isFinite(vel) || !Math::isFinite(acc))
			return false;
		return true;
	}

	inline bool operator < (const GenCoord& c) const {
		return pos < c.pos && vel < c.vel && acc < c.acc;
	}
	inline bool operator <= (const GenCoord& c) const {
		return pos <= c.pos && vel <= c.vel && acc <= c.acc;
	}

	GenCoord operator + (const GenCoord& c) const {
		return GenCoord(pos + c.pos, vel + c.vel, acc + c.acc);
	}
	GenCoord operator - (const GenCoord& c) const {
		return GenCoord(pos - c.pos, vel - c.vel, acc - c.acc);
	}
};

/** Generalised coordinates in configspace. */
typedef ObjectCoord<GenCoord, Configspace> GenCoordConfigspace;

//------------------------------------------------------------------------------

/** Generalised configuration space coordinates. */
class GenConfigspaceCoord {
public:
	/** position */
	ConfigspaceCoord cpos;
	/** velocity */
	ConfigspaceCoord cvel;
	/** acceleration */
	ConfigspaceCoord cacc;
	
	/**	Get from range */
	inline void get(const Configspace::Index& begin, const Configspace::Index& end, GenConfigspaceCoord& coord) const {
		for (Configspace::Index i = begin; i < end; ++i) {
			coord.cpos[i] = cpos[i];
			coord.cvel[i] = cvel[i];
			coord.cacc[i] = cacc[i];
		}
	}
	/**	Set from range */
	inline void set(const Configspace::Index& begin, const Configspace::Index& end, const GenConfigspaceCoord& coord) {
		for (Configspace::Index i = begin; i < end; ++i) {
			cpos[i] = coord.cpos[i];
			cvel[i] = coord.cvel[i];
			cacc[i] = coord.cacc[i];
		}
	}

	/**	Assigns configuration space coordinates. */
	inline void fromGenCoord(const Configspace::Index& i, const GenCoord& gc) {
		cpos[i] = gc.pos;
		cvel[i] = gc.vel;
		cacc[i] = gc.acc;
	}
	/**	Returns configuration space coordinates. */
	inline GenCoord toGenCoord(const Configspace::Index& i) const {
		GenCoord gc(cpos[i], cvel[i], cacc[i]);
		return gc;
	}

	/** sets to the default value */
	void setToDefault(Configspace::Index begin, Configspace::Index end) {
		cpos.setToDefault(begin, end);
		cvel.setToDefault(begin, end);
		cacc.setToDefault(begin, end);
	}
	/** tests for valid object */
	bool isValid(Configspace::Index begin, Configspace::Index end) const {
		if (!cpos.isValid(begin, end) || !cvel.isValid(begin, end) || !cacc.isValid(begin, end))
			return false;
		return true;
	}
};

/** Generalised configuration space coordinates as state variable. */
typedef CoordState<GenConfigspaceCoord> GenConfigspaceState;

//------------------------------------------------------------------------------

/** Generalised Workspace coordinates. */
template <typename WorkspaceCoord> class GenWorkspaceCoord {
public:
	/** Space Index */
	typedef typename WorkspaceCoord::Index Index;

	/** Poses */
	WorkspaceCoord wpos;

	/**	Get from range */
	inline void get(const Index& begin, const Index& end, GenWorkspaceCoord& coord) const {
		for (Index i = begin; i < end; ++i) {
			coord.wpos[i] = wpos[i];
		}
	}
	/**	Set from range */
	inline void set(const Index& begin, const Index& end,const GenWorkspaceCoord& coord) {
		for (Index i = begin; i < end; ++i) {
			wpos[i] = coord.wpos[i];
		}
	}

	/** sets to the default value */
	void setToDefault(Index begin, Index end) {
		wpos.setToDefault(begin, end);
	}
	/** tests for valid object */
	bool isValid(Index begin, Index end) const {
		if (!wpos.isValid(begin, end))
			return false;
		return true;
	}
};

/** Rigid body coordinates (tool frames). */
typedef GenWorkspaceCoord<WorkspaceChainCoord> GenWorkspaceChainCoord;
typedef CoordState<GenWorkspaceChainCoord> GenWorkspaceChainState;

/** Rigid body coordinates (all joints). */
typedef GenWorkspaceCoord<WorkspaceJointCoord> GenWorkspaceJointCoord;
typedef CoordState<GenWorkspaceJointCoord> GenWorkspaceJointState;

//------------------------------------------------------------------------------

/** Exponential coordinates of rigid body transformation (exponential mapping: se(3) -> SE(3)). */
class ExpCoord {
public:
	typedef std::vector<ExpCoord> Seq;

	/** Transformation magnitude */
	Real theta;
	/** Twist coordinates of rigid body transformation (generator of exponential mapping: se(3) -> SE(3)) */
	Twist twist;
	
	/** Axis and position instead of twist */
	bool axis;

	/** sets specified values */
	void set(Real theta = REAL_ZERO, Real v1 = REAL_ZERO, Real v2 = REAL_ZERO, Real v3 = REAL_ZERO, Real w1 = REAL_ZERO, Real w2 = REAL_ZERO, Real w3 = REAL_ZERO, bool axis = true) {
		twist.set(v1, v2, v3, w1, w2, w3);
		this->theta = theta;
		this->axis = axis;
	}

	/** sets to the default value */
	void setToDefault() {
		theta = REAL_ZERO;
		twist.setId();
		axis = true;
	}
	/** tests for valid object */
	bool isValid() const {
		if (!Math::isFinite(theta) || !twist.isFinite())
			return false;
		return true;
	}
};

/** Exponential coordinates of tool frames. */
typedef ObjectCoord<ExpCoord, Chainspace> WorkspaceExpCoord;

/** Manipulator Jacobian (in twist coordinates). */
typedef ObjectCoord<Twist, Configspace> Jacobian;

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_TYPES_H_*/
