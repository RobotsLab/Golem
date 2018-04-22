/** @file Sample.h
 * 
 * Sample class.
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
#ifndef _GOLEM_MATH_SAMPLE_H_
#define _GOLEM_MATH_SAMPLE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>
#include <functional>
#include <cmath>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Single iteration of the Kahan sum reduction algorithm (http://en.wikipedia.org/wiki/Kahan_summation_algorithm)
*	Real sum = 0;
*	Real c = 0;
*	for (i=0..N) kahanSum(sum, c, data[i]);
*/
template <typename _Real> inline void kahanSum(_Real& sum, _Real& c, const _Real& data) {
	_Real y = data - c;
	_Real t = sum + y;
	c = (t - sum) - y;
	sum = t;
}
template <typename _Real> inline void kahanSum(_Real* sum, _Real* c, const _Real* data, size_t N) {
	for (size_t n = 0; n < N; ++n) {
		_Real y = *data - *c;
		_Real t = *sum + y;
		*c = (t - *sum) - y;
		*sum = t;
		++sum;
		++c;
		++data;
	}
}

//------------------------------------------------------------------------------

/** Pointer dereferencing template */
struct Ref1 {
	template <typename _Type, typename Ptr> static inline _Type& get(Ptr& ptr) {
		return ptr; // nothing to do
	}
};
/** Pointer to pointer dereferencing template */
struct Ref2 {
	template <typename _Type, typename Ptr> static inline _Type& get(Ptr& ptr) {
		return *ptr;
	}
};

/** Cumulative density function sample */
template <typename _Real> class Sample {
public:
	/** Cumulative density value comparator */
	template <typename _Ref> struct Cmpr {
		template <typename Ptr> inline bool operator () (Ptr& l, _Real cdf) const {
			return _Ref::template get<const Sample>(l).cdf < cdf;
		}
		template <typename Ptr> inline bool operator () (_Real cdf, Ptr& r) const {
			return cdf < _Ref::template get<const Sample>(r).cdf;
		}
		template <typename Ptr> inline bool operator () (Ptr& l, Ptr& r) const {
			return _Ref::template get<const Sample>(l).cdf < _Ref::template get<const Sample>(r).cdf;
		}
	};

	/** Weight */
	_Real weight;
	/** Cumulative density */
	_Real cdf;

	/** No initialisation */
	inline Sample() {
	}

	/** Sets the sample weight; invalidates cdf if it is not specified */
	inline Sample(_Real weight, _Real cdf = -numeric_const<_Real>::ONE) {
		set(weight, cdf);
	}

	/** Sets the parameters to the default values */
	inline void setToDefault() {
		set(numeric_const<_Real>::ONE);
	}

	/** Sets the sample weight; invalidates cdf if it is not specified */
	inline void set(_Real weight, _Real cdf = -numeric_const<_Real>::ONE) {
		this->weight = weight;
		this->cdf = cdf;
	}

	/** Sets the parameters to the default values */
	template <typename _Ref, typename _Seq> static inline void setToDefault(_Seq& seq) {
		for (typename _Seq::iterator i = seq.begin(); i != seq.end(); ++i)
			_Ref::template get<Sample>(*i).setToDefault();
	}

	/** Sets the sample weights to the specified value; invalidates cdf */
	template <typename _Ref, typename _Seq> static inline void set(_Real weight, _Seq& seq) {
		for (typename _Seq::iterator i = seq.begin(); i != seq.end(); ++i)
			_Ref::template get<Sample>(*i).set(weight);
	}

	/** Checks if sample is valid. */
	bool isValid() const {
		if (weight < numeric_const<_Real>::ZERO || cdf < numeric_const<_Real>::ZERO)
			return false;
		
		return true;
	}
	
	/** Checks if the sample have identical data. */
	template <typename Ref1, typename Ref2> inline bool equals(const Sample& s) const {
		return weight == Ref1::template get<const Sample>(s).weight && cdf == Ref2::template get<const Sample>(s).cdf;
	}
	
	/** Checks if sample sequence is valid. */
	template <typename _Ref, typename _Seq> static inline bool isValid(const _Seq& seq, _Real eps = numeric_const<_Real>::EPS) {
		if (seq.empty() || _Ref::template get<const Sample>(*(seq.end() - 1)).cdf <= eps)
			return false;
		for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i)
			if (!_Ref::template get<const Sample>(*i).isValid())
				return false;
		
		return true;
	}
	
	/** Normalisation constant (CDF) */
	template <typename _Ref, typename _Seq> static inline _Real getNorm(const _Seq& seq, _Real eps = numeric_const<_Real>::EPS) {
		if (seq.empty())
			return numeric_const<_Real>::ZERO;

		const _Real cdf = _Ref::template get<const Sample>(*(seq.end() - 1)).cdf;
		if (cdf <= eps)
			return numeric_const<_Real>::ZERO;
		
		return numeric_const<_Real>::ONE/cdf;
	}

	/** Normalisation constant (weights) */
	template <typename _Ref, typename _Seq> static inline _Real getNormWeight(const _Seq& seq, _Real eps = numeric_const<_Real>::EPS) {
		if (seq.empty())
			return numeric_const<_Real>::ZERO;

		_Real weight = numeric_const<_Real>::ZERO;
		for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i) {
			const _Real w = _Ref::template get<const Sample>(*i).weight;
			if (weight < w)
				weight = w;
		}

		if (weight <= eps)
			return numeric_const<_Real>::ZERO;

		return numeric_const<_Real>::ONE / weight;
	}

	/** Normalises set of samples using samples weights */
	template <typename _Ref, typename _Seq> static inline bool normalise(_Seq& seq, _Real eps = numeric_const<_Real>::EPS) {
		if (seq.empty())
			return false;
		
		_Real sum = numeric_const<_Real>::ZERO, c = numeric_const<_Real>::ZERO;
		for (typename _Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
			Sample& sample = _Ref::template get<Sample>(*i);
			kahanSum(sum, c, sample.weight);
			sample.cdf = sum;
		}

		return sum > eps;
	}

	/** Normalises set of samples to 1 */
	template <typename _Ref, typename _Seq> static inline bool uniform(_Seq& seq) {
		if (seq.empty())
			return false;
		
		const _Real delta = numeric_const<_Real>::ONE/seq.size();
		size_t j = 0;
		for (typename _Seq::iterator i = seq.begin(); i != seq.end(); ++i) {
			Sample& sample = _Ref::template get<Sample>(*i);
			sample.weight = delta;
			sample.cdf = delta*++j;
		}

		return true;
	}

	/** Draws a sample from set of samples */
	template <typename _Ref, typename Ptr, typename _Seq, typename Rand> static inline Ptr sample(_Seq& seq, Rand& rand, _Real eps = numeric_const<_Real>::EPS) {
		if (seq.empty())
			return seq.end();

		const _Real cdf = _Ref::template get<const Sample>(*(seq.end() - 1)).cdf;
		if (cdf <= eps)
			return seq.end();

		const _Real s = rand.template nextUniform<_Real>()*cdf;
		const Ptr ptr = std::lower_bound(seq.begin(), seq.end(), s, Cmpr<_Ref>());
		
		return ptr == seq.end() ? (seq.end() - 1) : ptr;
	}
};

//------------------------------------------------------------------------------

/** Sample property: weighted mean and covariance */
template <typename _Real, typename _Type> class SampleProperty {
public:
	/** Mean value */
	_Type mean;
	/** Covariance */
	_Type covariance;
	/** Square root of covariance */
	_Type covarianceSqrt;
	/** Inverse covariance */
	_Type covarianceInv;
	/** Square root of inverse covariance */
	_Type covarianceInvSqrt;

	template <typename _Ref, typename _TypeRef, typename _Seq> bool create(size_t n, const _Type& scale, _Seq& seq, _Real eps = numeric_const<_Real>::EPS) {
		if (!Sample<_Real>::template normalise<_Ref>(seq, eps))
			return false;

		// initialise variables
		_Type c;
		for (size_t j = 0; j < n; ++j)
			mean[j] = c[j] = numeric_const<_Real>::ZERO;
		const _Real norm = numeric_const<_Real>::ONE / seq.back().cdf;

		// weighted mean
		for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i) {
			for (size_t j = 0; j < n; ++j)
				kahanSum(mean[j], c[j], _Ref::template get<const Sample<_Real> >(*i).weight*_TypeRef::get(*i)[j]);
		}
		for (size_t j = 0; j < n; ++j)
			mean[j] *= norm;

		// weighted covariance
		for (size_t j = 0; j < n; ++j)
			covariance[j] = c[j] = numeric_const<_Real>::ZERO;
		for (typename _Seq::const_iterator i = seq.begin(); i != seq.end(); ++i) {
			for (size_t j = 0; j < n; ++j)
				kahanSum(covariance[j], c[j], _Ref::template get<const Sample<_Real> >(*i).weight*Math::sqr(_TypeRef::get(*i)[j] - mean[j]));
		}
		for (size_t j = 0; j < n; ++j) {
			covariance[j] *= norm*scale[j];
			covarianceSqrt[j] = Math::sqrt(covariance[j]);
			covarianceInv[j] = numeric_const<_Real>::ONE / (covariance[j]);
			covarianceInvSqrt[j] = numeric_const<_Real>::ONE / (covarianceSqrt[j]);
		}

		return true;
	}

	template <typename _Val, typename _Cov> bool create(size_t size, size_t dim, _Val val, _Cov cov) {
		if (size <= 0 || dim <= 0)
			return false;

		// initialise variables
		_Type c;
		for (size_t j = 0; j < dim; ++j)
			mean[j] = c[j] = numeric_const<_Real>::ZERO;
		const _Real norm = numeric_const<_Real>::ONE / size;

		// mean
		for (size_t i = 0; i < size; ++i) {
			for (size_t j = 0; j < dim; ++j)
				kahanSum(mean[j], c[j], val(i, j));
		}
		for (size_t j = 0; j < dim; ++j)
			mean[j] *= norm;

		// covariance
		for (size_t j = 0; j < dim; ++j)
			covariance[j] = c[j] = numeric_const<_Real>::ZERO;
		for (size_t i = 0; i < size; ++i) {
			for (size_t j = 0; j < dim; ++j)
				kahanSum(covariance[j], c[j], Math::sqr(val(i, j) - mean[j]));
		}
		for (size_t j = 0; j < dim; ++j) {
			covariance[j] = cov(j, norm*covariance[j]);
			covarianceSqrt[j] = Math::sqrt(covariance[j]);
			covarianceInv[j] = numeric_const<_Real>::ONE / (covariance[j]);
			covarianceInvSqrt[j] = numeric_const<_Real>::ONE / (covarianceSqrt[j]);
		}

		return true;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_SAMPLE_H_*/
