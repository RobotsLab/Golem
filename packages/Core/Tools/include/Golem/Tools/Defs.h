/** @file Defs.h
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
#ifndef _GOLEM_TOOLS_DEFS_H_
#define _GOLEM_TOOLS_DEFS_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Defs.h>
#include <Golem/Math/Vec3.h>
#include <bitset>
#include <vector>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

typedef golem::_Vec3<golem::F32> F32Vec3;
typedef std::vector<F32Vec3> F32Vec3Seq;

//------------------------------------------------------------------------------

/** Power scaling with sign preservation */
template <typename _Real> inline _Real powerScale(_Real value, _Real pow = golem::numeric_const<_Real>::ONE) {
	return golem::Math::sign(golem::Math::pow(golem::Math::abs(value), pow), value);
}

//------------------------------------------------------------------------------

/** std::bitset comparison operator */
struct bitset_compare {
	template <size_t _Size> inline bool operator() (const std::bitset<_Size>& left, const std::bitset<_Size>& right) const {
#ifdef WIN32 // VC++
		// fast implementation (VC++ only)
		typedef typename std::bitset<_Size>::_Ty _Word;
		static const size_t _Words = _Size > 1 ? (_Size - 1) / (8 * sizeof(_Word)) + 1 : 1;
		for (size_t word = _Words; word-- > 0;) {
			const _Word wl = left._Getword(word);
			const _Word wr = right._Getword(word);
			if (wl < wr)
				return true;
			if (wl > wr)
				return false;
		}
#else // gcc
		// slow but implementation independent
		for (size_t bit = _Size; bit-- > 0;)
			if (left[bit] ^ right[bit])
				return right[bit];
#endif
		return false;
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_TOOLS_DEFS_H_*/
