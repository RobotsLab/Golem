/** @file GolemInteropDefs.h
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
#ifndef _GOLEM_INTEROP_INTEROP_DEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_DEFS_H_

//------------------------------------------------------------------------------

#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Generic data structures
	*
	**************************************************************************/

	/** Default floating point */
	typedef double float_t;

	/** String sequence */
	typedef std::vector<std::string> StringSeq;

	/** Generic vector with a fixed maximum size */
	template <typename _Type, size_t _Size> class Vec {
	public:
		/** Sequence */
		typedef std::vector<Vec> Seq;

		/** Maximum size */
		static const size_t SIZE = _Size;
		/** Type */
		typedef _Type TYPE;

		/** Vector elements. */
		TYPE v[SIZE];

		/** Default constructor sets the default values. */
		Vec() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			std::fill(v, v + SIZE, TYPE()); // zero-initialisation for primitive types, default constructor otherwise
		}
		/** Access vector as an array. */
		inline TYPE& operator [] (size_t idx) {
			return v[idx];
		}
		/** Access vector as an array. */
		inline const TYPE& operator [] (size_t idx) const {
			return v[idx];
		}
	};

	/** 3 Element vector class. */
	typedef Vec<float_t, 3> Vec3;

	/** Matrix representation of SO(3) group of rotations. */
	class Mat33 {
	public:
		/** Matrix elements. */
		union {
			struct {
				float_t m11, m12, m13;
				float_t m21, m22, m23;
				float_t m31, m32, m33;
			};
			float_t m[3][3];
		};
		/** Default constructor sets the default values. */
		Mat33() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			// ID transformation
			m11 = float_t(1.); m12 = float_t(0.); m13 = float_t(0.);
			m21 = float_t(0.); m22 = float_t(1.); m23 = float_t(0.);
			m31 = float_t(0.); m32 = float_t(0.); m33 = float_t(1.);
		}
	};

	/** Quaternion representation of SO(3) group of rotations. */
	class Quat {
	public:
		/** Quaternion elements. */
		union {
			struct {
				float_t q0, q1, q2, q3;
			};
			struct {
				float_t w, x, y, z;
			};
			float_t q[4];
		};
		/** Default constructor sets the default values. */
		Quat() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			// ID transformation
			q0 = float_t(1.); q1 = float_t(0.); q2 = float_t(0.); q3 = float_t(0.);
		}
	};

	/** Homogeneous representation of SE(3) rigid body transformations. */
	class Mat34 {
	public:
		/** rotation matrix	*/
		Mat33 R;
		/** translation	*/
		Vec3 p;

		/** Default constructor sets the default values. */
		Mat34() {
			//clear();
		}
		/** Set to the default values. */
		void clear() {
			// ID transformation
			R.clear();
			p.clear();
		}
	};

	/** Quaternion-based paremetrisation of SE(3) rigid body transformations. */
	class Frame3D {
	public:
		/** orientation	*/
		Quat q;
		/** translation	*/
		Vec3 p;

		/** Default constructor sets the default values. */
		Frame3D() {
			//clear();
		}
		/** Set to the default values. */
		void clear() {
			// ID transformation
			q.clear();
			p.clear();
		}
	};

	/** Twist coordinates of rigid body transformation. */
	class Twist {
	public:
		/** linear component */
		Vec3 v;
		/** angular component */
		Vec3 w;

		/** Default constructor sets the default values. */
		Twist() {
			//clear();
		}
		/** Set to the default values. */
		void clear() {
			// ID transformation
			v.clear();
			w.clear();
		}
	};

	/** Rigid body transformation distance. */
	class Dist {
	public:
		/** Linear */
		float_t lin;
		/** Angular	*/
		float_t ang;

		/** Default constructor sets the default values. */
		Dist() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			lin = float_t(0.);
			ang = float_t(0.);
		}
	};

	/** Density estimator sample. */
	class Sample {
	public:
		/** Weight */
		float_t weight;

		/** Default constructor sets the default values. */
		Sample() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			weight = float_t(1.); // equally weighted
		}
	};

	/*************************************************************************
	*
	* Sensor data structures
	*
	**************************************************************************/

	/** RGBA colour space */
	class RGBA {
	public:
		/** Colour elements. */
		union {
			struct {
				std::uint8_t r, g, b, a;
			};
			std::uint32_t uint32;
			std::uint8_t uint8[4];
		};
		/** Default constructor sets the default values. */
		RGBA() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			// grey colour
			r = 127; g = 127; b = 127; a = 255;
			// white colour
			//uint32 = UINT32_MAX;
		}
	};

	/** 3D point with normal. */
	class Point3D {
	public:
		/** Position */
		Vec3 position;
		/** Normal */
		Vec3 normal;
		/** Colour */
		RGBA colour;

		/** Default constructor sets the default values. */
		Point3D() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			position.clear();
			normal[0] = float_t(0.); normal[1] = float_t(0.); normal[2] = float_t(0.); // invalidate: norm() = 0
			colour.clear();
		}
	};

	/** Point cloud */
	template <typename _Point> class Cloud : public std::vector<_Point> {
	public:
		/** Sequence */
		typedef std::vector<Cloud> Seq;

		/** Dimensions: width */
		std::uint32_t width;
		/** Dimensions: height */
		std::uint32_t height;
		/** Sensor frame */
		Frame3D frame;

		/** Default constructor sets the default values. */
		Cloud() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			std::vector<_Point>::clear();
			width = 0;
			height = 0;
			frame.clear();
		}
	};

	/** 3D point with normal cloud. */
	typedef Cloud<Point3D> Point3DCloud;

	/*************************************************************************
	*
	* Control data structures
	*
	**************************************************************************/

	/** Robot joint configuration, ConfigspaceCoord::SIZE = maximum number of joints */
	typedef Vec<float_t, 70> ConfigspaceCoord;
	/** Robot Cartesian configuration of its end-effectors, WorkspaceCoord::SIZE = maximum number of kinematic chains */
	typedef Vec<Mat34, 18> WorkspaceCoord;
	/** Workspace distance */
	typedef Vec<Dist, WorkspaceCoord::SIZE> WorkspaceDist;

	/** Robot config */
	class Config {
	public:
		/** Sequence */
		typedef std::vector<Config> Seq;

		/** Reserved area */
		typedef Vec<std::uint8_t, 4 * 1024> Reserved;

		/** Time stamp */
		double t;

		/** Actual configuration space dim. */
		//std::uint32_t configspaceSize;

		/** Configuration space position */
		ConfigspaceCoord cpos;
		/** Configuration space velocity */
		ConfigspaceCoord cvel;

		/** Reserved area */
		Reserved reserved;

		/** Default constructor sets the default values. */
		Config() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			t = double(0.);
			//configspaceSize = 0;
			cpos.clear();
			cvel.clear();
			reserved.clear();
		}
	};

	/** Robot trajectory */
	class Trajectory {
	public:
		/** Sequence of trajectories */
		typedef std::vector<Trajectory> Seq;

		/** Trajectory type */
		std::string type;
		/** Trajectory waypoints */
		Config::Seq trajectory;
		/** Trajectory workspace error */
		WorkspaceDist error;

		/** Default constructor sets the default values. */
		Trajectory() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			type.clear();
			trajectory.clear();
			error.clear();
		}
	};

	/*************************************************************************
	*
	* Definitions
	*
	**************************************************************************/

	/** String set */
	typedef std::set<std::string> StringSet;
	/** String sequence */
	typedef std::vector<std::string> StringSeq;

	/** Memory buffer */
	typedef std::vector<std::uint8_t> Buffer;

	/** Runs guard function after leaving variable scope */
	class ScopeGuard {
	public:
		/** Guard function */
		typedef std::function<void()> Function;

		/** Initialise guard */
		ScopeGuard(Function function) : function(function) {
		}
		/** Run guard function */
		~ScopeGuard() {
			function();
		}

	private:
		Function function;
	};

	/*************************************************************************
	*
	* Basic type conversion
	*
	**************************************************************************/

	/** Basic type conversion */
	template <typename _Src, typename _Dst> void convert(const _Src& src, _Dst& dst) {
		dst = static_cast<_Dst>(src);
	}
	template <typename _Src, typename _Dst> void copy(_Src begin, _Src end, _Dst ptr) {
		for (; begin != end; ++ptr, ++begin)
			convert(*begin, *ptr);
	}

	template <typename _Src, typename _Dst> void convert(const _Src& src, std::vector<_Dst>& dst) {
		dst.clear();
		dst.reserve(src.size());
		for (typename _Src::const_iterator i = src.begin(); i != src.end(); ++i) {
			_Dst val;
			convert(*i, val);
			dst.insert(dst.end(), val);
		}
	}

	template <typename _Src, typename _Dst> void convert(const _Src& src, std::set<_Dst>& dst) {
		dst.clear();
		for (typename _Src::const_iterator i = src.begin(); i != src.end(); ++i) {
			_Dst val;
			convert(*i, val);
			dst.insert(dst.end(), val);
		}
	}

	template <typename _Src, typename _Dst> void convert(const _Src& src, std::unordered_set<_Dst>& dst) {
		dst.clear();
		for (typename _Src::const_iterator i = src.begin(); i != src.end(); ++i) {
			_Dst val;
			convert(*i, val);
			dst.insert(dst.end(), val);
		}
	}

	template <typename _Src, typename _DstKey, typename _DstVal> void convert(const _Src& src, std::map<_DstKey, _DstVal>& dst) {
		dst.clear();
		for (typename _Src::const_iterator i = src.begin(); i != src.end(); ++i) {
			_DstKey key;
			convert(i->first, key);
			convert(i->second, dst[key]);
		}
	}

	template <typename _Src, typename _DstKey, typename _DstVal> void convert(const _Src& src, std::unordered_map<_DstKey, _DstVal>& dst) {
		dst.clear();
		for (typename _Src::const_iterator i = src.begin(); i != src.end(); ++i) {
			_DstKey key;
			convert(i->first, key);
			convert(i->second, dst[key]);
		}
	}

	/** Memory buffer */
	typedef std::vector<std::uint8_t> Buffer;

	/** Memory buffer streaming */
	template <typename _Dst> inline Buffer::const_iterator bufferCopy(Buffer::const_iterator ptr, _Dst& dst) {
		dst = *reinterpret_cast<const _Dst*>(&*ptr);
		return ptr + sizeof(_Dst);
	}
	template <typename _Src> inline Buffer::iterator bufferCopy(const _Src& src, Buffer& dst, Buffer::iterator ptr) {
		const size_t pos = ptr - dst.begin(), size = pos + sizeof(_Src);
		if (size > dst.size()) {
			dst.resize(size);
			ptr = dst.begin() + pos;
		}
		*reinterpret_cast<_Src*>(&*ptr) = src;
		return ptr + sizeof(_Src);
	}

	/** Memory buffer streaming from/into sequence */
	template <typename _Dst> inline Buffer::const_iterator bufferCopySeq(Buffer::const_iterator ptr, _Dst& dst) {
		std::uint64_t size = 0;
		ptr = bufferCopy(ptr, size);
		dst.clear();
		while (size-- > 0) {
			typename _Dst::value_type value;
			ptr = bufferCopy(ptr, value);
			dst.insert(dst.end(), value);
		}
		return ptr;
	}
	template <typename _Src> inline Buffer::iterator bufferCopySeq(const _Src& src, Buffer& dst, Buffer::iterator ptr) {
		const std::uint64_t size = src.size();
		ptr = bufferCopy(size, dst, ptr);
		for (typename _Src::const_iterator i = src.begin(); i != src.end(); ++i)
			ptr = bufferCopy(*i, dst, ptr);
		return ptr;
	}

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_DEFS_H_