/** @file Import.h
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
#ifndef _GOLEM_TOOLS_IMPORT_H_
#define _GOLEM_TOOLS_IMPORT_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/Math/TriangleMesh.h>
#include <Golem/Math/Rand.h>
#include <iostream>

//------------------------------------------------------------------------------

namespace golem {
	class Context;
	class XMLContext;
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Formatted input stream wrapper */
class IStream {
public:
	/** Data */
	typedef std::vector<char> Data;

	/** Uses a generic std::istream */
	IStream(std::istream& stream, const char* delim = nullptr);

	/** Delimiters, nullptr for binary mode */
	inline void setDelim(const char* delim = nullptr) {
		this->delim = delim;
	}
	/** Delimiters */
	inline const char* getDelim() const {
		return delim;
	}
		
	/** End of stream */
	inline bool eos() const {
		return stream.eof();
	}
	/** Pointer increment */
	template <typename _Type> inline void inc() {
		inc(sizeof(_Type));
	}

	/** Raw data conversion */
	template <typename _Type> inline _Type to() const {
		return (_Type)data.data();
	}
	/** Raw data increment */
	template <typename _Type> inline _Type next() {
		inc<_Type>();
		return to<_Type>();
	}

	/** Data conversion */
	template <typename _Type> _Type toInt() const {
		return getDelim() == nullptr ? *to<const _Type*>() : (_Type)strtol(data.data());
	}
	/** Data increment */
	template <typename _Type> _Type nextInt() {
		inc<_Type>();
		return toInt<_Type>();
	}

	/** Data conversion */
	template <typename _Type> _Type toFloat() const {
		return getDelim() == nullptr ? *to<const _Type*>() : (_Type)strtod(data.data());
	}
	/** Data increment */
	template <typename _Type> _Type nextFloat() {
		inc<_Type>();
		return toFloat<_Type>();
	}

private:
	/** Generic std::istream */
	std::istream& stream;
	/** Data buffer */
	Data data;
	/** Delimiters */
	const char* delim;
		
	/** Increment */
	void inc(size_t size);
	/** Integer conversion */
	long strtol(const char* str) const;
	/** Floating-point conversion */
	double strtod(const char* str) const;
};

//------------------------------------------------------------------------------

/** Import algortihms */
class Import {
public:
	/** XYZ, normal callback */
	typedef std::function<void(const golem::Vec3&, const golem::Vec3&)> XYZNormalFunc;

	/** File extension: cloud obj */
	static const std::string FILE_EXT_CLOUD_OBJ;
	/** File extension: cloud ply */
	static const std::string FILE_EXT_CLOUD_PLY;
	/** File extension: cloud pcd */
	static const std::string FILE_EXT_CLOUD_PCD;

	/** Size */
	golem::U32 size;
	/** Scale */
	golem::Real scale;
	/** Delimiters */
	std::string delim;
	/** Triangle vertex index order */
	bool clockwise;
	
	/** Constructs the object */
	Import() {
		setToDefault();
	}
	/** Sets the parameters to the default values */
	void setToDefault() {
		size = 100000;
		scale = golem::REAL_ONE;
		delim = " \n\t\r,;";
		clockwise = true;
	}
	/** Assert that the object is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(size > 0, ac, "size: 0");
		Assert::valid(scale > golem::REAL_ZERO, ac, "scale: <= 0");
	}
	/** Reads/writes object from/to a given XML context */
	void xmlData(golem::XMLContext* context, bool create = false);

	/** Next integer number */
	template <typename _Type, typename _CastType> static void nextInt(IStream& stream, _CastType& val) {
		val = (_CastType)stream.nextInt<_Type>();
	}
	/** Next float number */
	template <typename _Type, typename _CastType> static void nextFloat(IStream& stream, _CastType& val) {
		val = (_CastType)stream.nextFloat<_Type>();
	}

	/** XYZ coordinate parser */
	template <typename _Type, typename _Data> static void nextXYZ(IStream& stream, _Data& data) {
		nextFloat<_Type>(stream, data.x);
		nextFloat<_Type>(stream, data.y);
		nextFloat<_Type>(stream, data.z);
	}
	/** RGBA parser */
	template <typename _Type, typename _Data> static void nextRGBA(IStream& stream, _Data& data) {
		nextInt<_Type>(stream, data.r);
		nextInt<_Type>(stream, data.g);
		nextInt<_Type>(stream, data.b);
		nextInt<_Type>(stream, data.a);
	}
	/** Triangle parser */
	template <typename _Type, typename _Data> static void nextTriangle(IStream& stream, _Data& data) {
		nextInt<_Type>(stream, data.t1);
		nextInt<_Type>(stream, data.t2);
		nextInt<_Type>(stream, data.t3);
	}

	/** Import point cloud from XYZ and Normal text file */
	void pointCloudXYZNormal(const std::string& path, XYZNormalFunc func) const;
	/** Import triangle mesh from obj file */
	void pointCloudObj(const std::string& path, Vec3Seq& vertices, TriangleSeq& triangles) const;
	/** Import triangle mesh from ply file */
	void pointCloudPly(const std::string& path, Vec3Seq& vertices, TriangleSeq& triangles) const;

	/** Generate points and normals */
	void generate(golem::Rand& rand, const Vec3Seq& vertices, const TriangleSeq& triangles, XYZNormalFunc func) const;
	/** Uniformly downsample sequence */
	template <typename _Rand, typename _Ptr, typename _Seq, typename _Trn> static void downsample(_Rand& rand, size_t size, _Ptr begin, _Ptr end, _Seq& seq, _Trn trn) {
		// generate random element indices
		std::vector<size_t> indices(end - begin);
		for (size_t i = 0; i < indices.size(); ++i)
			indices[i] = i;
		std::random_shuffle(indices.begin(), indices.end(), rand);
		indices.resize(std::min(indices.size(), size));
		// copy elements
		seq.resize(indices.size());
		for (size_t i = 0; i < indices.size(); ++i)
			seq[i] = trn(*(begin + indices[i]));
	}
	/** Uniformly downsample sequence */
	template <typename _Rand, typename _Seq> void downsample(_Rand& rand, _Seq& seq) const {
		if (seq.size() > this->size) {
			// generate random element indices
			std::vector<size_t> indices(seq.size());
			for (size_t i = 0; i < indices.size(); ++i)
				indices[i] = i;
			std::random_shuffle(indices.begin(), indices.end(), rand);
			indices.resize(this->size);
			std::sort(indices.begin(), indices.end());
			// move elements to the front
			for (size_t i = 0; i < indices.size(); ++i)
				seq[i] = seq[indices[i]];
			// compact the sequence
			seq.resize(this->size);
		}
	}

	/** HDF5 RealSeq data record callback */
	typedef std::function<void(const std::string&, size_t, const golem::RealSeq&)> HDF5DumpRealSeqFunc;
	/** HDF5 RealSeq data set name - data record callback map */
	typedef std::map<std::string, HDF5DumpRealSeqFunc> HDF5DumpRealSeqMap;

	/** HDF5 import generic real vector streams from hdf5 text file */
	void hdf5DumpRealSeq(const std::string& path, const HDF5DumpRealSeqMap& map) const;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------


#endif /*_GOLEM_TOOLS_IMPORT_H_*/
