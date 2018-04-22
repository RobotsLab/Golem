/** @file Import.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/Import.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Sample.h>
#include <fstream>
#include <cstring>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

IStream::IStream(std::istream& stream, const char* delim) : stream(stream), delim(delim) {
	if (!stream)
		throw Message(Message::LEVEL_ERROR, "IStream::IStream(): unable to open stream");
}

void IStream::inc(size_t size) {
	data.clear();
	if (!delim) {
		data.resize(size);
		stream.read(data.data(), data.size());
		if (!stream)
			throw Message(Message::LEVEL_ERROR, "IStream::inc(): unable to read data");
	}
	else {
		for (bool ready = false;;) {
			READ:
			if (eos()) {
				if (data.empty())
					throw Message(Message::LEVEL_ERROR, "IStream::inc(): no data to read");
				else
					break;
			}
			char c;
			stream.read(&c, 1);
			for (const char* str = delim; *str != 0; ++str)
				if (c == *str) {
					ready = !data.empty();
					goto READ;
				}
			if (ready) {
				stream.putback(c);
				break;
			}
			data.push_back(c);
		}
		data.push_back(0);
	}
}

long IStream::strtol(const char* str) const {
	char* endptr;
	const long val = std::strtol(str, &endptr, 10);
	if (*endptr)
		throw golem::Message(golem::Message::LEVEL_ERROR, "IStream::strtol(): invalid argument %s", str);
	return val;
}

double IStream::strtod(const char* str) const {
	char* endptr;
	const double val = std::strtod(str, &endptr);
	if (*endptr)
		throw golem::Message(golem::Message::LEVEL_ERROR, "IStream::strtod(): invalid argument %s", str);
	return val;
}

//------------------------------------------------------------------------------

const std::string golem::Import::FILE_EXT_CLOUD_OBJ = ".obj";
const std::string golem::Import::FILE_EXT_CLOUD_PLY = ".ply";
const std::string golem::Import::FILE_EXT_CLOUD_PCD = ".pcd";

void Import::xmlData(golem::XMLContext* context, bool create) {
	golem::XMLData("size", size, context, create);
	golem::XMLData("scale",scale, context, create);
	golem::XMLData("delim", delim, context, create);
	golem::XMLData("clockwise", clockwise, context, create);
}

void Import::pointCloudXYZNormal(const std::string& path, XYZNormalFunc func) const {
	std::ifstream ifstream(path, std::ios::binary | std::ios::in);
	IStream istream(ifstream, delim.c_str());
	while (!istream.eos()) {
		Vec3 p, n;
		try {
			nextXYZ<golem::Real>(istream, p);
			nextXYZ<golem::Real>(istream, n);
		}
		catch (const Message&) {}
		p.multiply(scale, p);
		func(p, n);
	};
}

void Import::pointCloudObj(const std::string& path, Vec3Seq& vertices, TriangleSeq& triangles) const {
	std::ifstream ifstream(path, std::ios::binary | std::ios::in);
	IStream istream(ifstream, "\r\n\t ");
	while (!istream.eos()) {
		const char* str = istream.next<const char*>();
		if (std::strcmp(str, "v") == 0) {
			Vec3 vertex;
			nextXYZ<golem::Real>(istream, vertex);
			vertex.multiply(scale, vertex);
			vertices.push_back(vertex);
		}
		else if (std::strcmp(str, "f") == 0) {
			Triangle triangle;
			nextTriangle<golem::Real>(istream, triangle);
			triangle.t1 -= 1; // [1..n] -> [0..n-1]
			triangle.t2 -= 1; // [1..n] -> [0..n-1]
			triangle.t3 -= 1; // [1..n] -> [0..n-1]
			triangles.push_back(triangle);
		}
	};
}

void Import::pointCloudPly(const std::string& path, Vec3Seq& vertices, TriangleSeq& triangles) const {
	bool binary = false;
	bool header = true;
	bool normals = false;
	int vertices_num = 0;
	int vertex_properties = 0;
	bool vertex_flags = false;
	int triangles_num = 0;
	int triangle_properties = 0;
	bool triangle_flags = false;
	std::ifstream ifstream(path, std::ios::binary | std::ios::in);
	IStream istream(ifstream, "\r\n\t ");
	while (!istream.eos()) {
		// header
		if (header) {
			const char* str = istream.next<const char*>();
			if (std::strcmp(str, "end_header") == 0) {
				header = false;
				normals = vertex_properties >= 6;
				vertex_flags = vertex_properties > 3 && vertex_properties != 6;
				triangle_flags = triangle_properties > 1;
				if (binary)
					istream.setDelim(nullptr);
			}
			else if (std::strcmp(str, "format") == 0) {
				binary = std::strcmp(istream.next<const char*>(), "ascii") != 0;
			}
			else if (std::strcmp(str, "element") == 0) {
				const char* element = istream.next<const char*>();
				if (std::strcmp(element, "vertex") == 0)
					vertices_num = istream.nextInt<int>();
				else if (std::strcmp(element, "face") == 0)
					triangles_num = istream.nextInt<int>();
			}
			else if (std::strcmp(str, "property") == 0) {
				if (triangles_num <= 0)
					++vertex_properties;
				else
					++triangle_properties;
			}
		}
		else if (vertices_num > 0) {
			Vec3 vertex;
			nextXYZ<float>(istream, vertex);
			vertex.multiply(scale, vertex);
			vertices.push_back(vertex);
			if (normals) { // ignore normals
				(void)istream.nextFloat<float>();
				(void)istream.nextFloat<float>();
				(void)istream.nextFloat<float>();
			}
			if (vertex_flags) {
				(void)istream.nextFloat<int>(); // ignore flags
			}
			--vertices_num;
		}
		else if (triangles_num > 0) {
			(void)istream.nextInt<unsigned char>(); // ignore list size
			Triangle triangle;
			nextTriangle<int>(istream, triangle);
			triangles.push_back(triangle);
			if (triangle_flags) {
				(void)istream.nextFloat<int>(); // ignore flags
			}
			--triangles_num;
		}
		else
			break;
	};
}

void Import::generate(golem::Rand& rand, const Vec3Seq& vertices, const TriangleSeq& triangles, XYZNormalFunc func) const {
	if (vertices.empty())
		throw Message(Message::LEVEL_ERROR, "Import::generate(): no vertices");
	if (triangles.empty())
		throw Message(Message::LEVEL_ERROR, "Import::generate(): no triangles");

	// Triangle face with normal
	class Face : public golem::Sample<golem::Real> {
	public:
		typedef std::vector<Face> Seq;
		golem::Vec3 a, b, p, n;
	};
	Face::Seq faces;
	faces.reserve(triangles.size());

	// find triangle face normals and areas
	for (TriangleSeq::const_iterator t = triangles.begin(); t != triangles.end(); ++t) {
		if (t->t1 >= vertices.size() || t->t2 >= vertices.size() || t->t3 >= vertices.size())
			throw Message(Message::LEVEL_ERROR, "Import::generate(): invalid triangle");
		Face face;
		face.p = vertices[t->t1];
		face.a.subtract(vertices[t->t2], face.p);
		face.b.subtract(vertices[t->t3], face.p);
		this->clockwise ? face.n.cross(face.a, face.b) : face.n.cross(face.b, face.a);
		face.weight = face.n.normalise(); // 2x triangle area
		faces.push_back(face);
	}
	if (!Sample<Real>::normalise<golem::Ref1>(faces))
		throw Message(Message::LEVEL_ERROR, "Import::generate(): unable to normalise faces");

	// generate points with normals
	for (U32 i = 0; i < this->size; ++i) {
		const Face::Seq::const_iterator f = golem::Sample<golem::Real>::sample<golem::Ref1, Face::Seq::const_iterator>(faces, rand);
		if (f == faces.end())
			throw Message(Message::LEVEL_ERROR, "Import::generate(): unable to sample faces");
		
		Real s[2];
		do {
			s[0] = rand.nextUniform<Real>();
			s[1] = rand.nextUniform<Real>();
		} while (s[0] + s[1] > REAL_ONE);
		Vec3 p;
		p.add(f->a * s[0], f->b * s[1]);
		p.add(f->p, p);
		
		func(p, f->n);
	}
}

//------------------------------------------------------------------------------

void golem::Import::hdf5DumpRealSeq(const std::string& path, const HDF5DumpRealSeqMap& map) const {
	const std::string delim("\r\n\t ,{}()/");
	const std::string delimDataset("\r\n\t {}\"");
	HDF5DumpRealSeqMap::const_iterator dataset = map.end();
	bool header = true;
	int records = 0, fields = 0, records_ptr = 0, fields_ptr = 0;
	RealSeq valSeq;

	std::ifstream ifstream(path, std::ios::binary | std::ios::in);
	IStream istream(ifstream);
	while (!istream.eos()) {
		istream.setDelim(delim.c_str());
		// data set
		if (dataset == map.end()) {
			if (std::strcmp(istream.next<const char*>(), "DATASET") == 0) {
				istream.setDelim(delimDataset.c_str());
				dataset = map.find(istream.next<const char*>());
				header = true;
			}
		}
		// data set header
		else if (header) {
			if (std::strcmp(istream.next<const char*>(), "DATASPACE") == 0) {
				// type
				if (std::strcmp(istream.next<const char*>(), "SIMPLE") != 0)
					throw Message(Message::LEVEL_ERROR, "Import::hdf5DumpRealSeq(): unknown dataspace type");
				// number of records and fields
				records = istream.nextInt<int>();
				fields = istream.nextInt<int>();
				if (records <= 0 || fields <= 0)
					throw Message(Message::LEVEL_ERROR, "Import::hdf5DumpRealSeq(): invalid data size %ix%i", records, fields);
				records_ptr = records;
				fields_ptr = fields;
				// width check
				if (std::strcmp(istream.next<const char*>(), "H5S_UNLIMITED") != 0)
					throw Message(Message::LEVEL_ERROR, "Import::hdf5DumpRealSeq(): unlimited width of formatted output required");
				(void)istream.nextInt<int>(); // ignore
				// ready for data
				if (std::strcmp(istream.next<const char*>(), "DATA") != 0)
					throw Message(Message::LEVEL_ERROR, "Import::hdf5DumpRealSeq(): data expected");
				// done
				header = false;
			}
		}
		// data set records
		else {
			valSeq.push_back(istream.nextFloat<Real>());
			if (--fields_ptr <= 0) {
				dataset->second(dataset->first, (size_t)(records - records_ptr), valSeq);
				valSeq.clear();
				fields_ptr = fields;
				if (--records_ptr <= 0)
					dataset = map.end();
			}
		}
	};
}

//------------------------------------------------------------------------------
