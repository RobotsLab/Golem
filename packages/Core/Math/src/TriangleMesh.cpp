/** @file TriangleMesh.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Math/TriangleMesh.h>
#include <stdexcept>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

bool ITriangleMesh::findTriangles(const std::vector<Vec3> &vertices, std::vector<Triangle> &triangles) {
	// TODO use any convex hull algorithm (e.g. quick hull)
	return false;
}

bool ITriangleMesh::findNormals(const std::vector<Vec3>& vertices, const std::vector<Triangle>& triangles, std::vector<Vec3>& normals, std::vector<Real>& distances) {
	const size_t TRIANGLES = triangles.size();
	normals.resize(TRIANGLES);
	distances.resize(TRIANGLES);
	for (size_t i = 0; i < TRIANGLES; ++i) {
		const Triangle t = triangles[i];
		const Vec3 p0 = vertices[t.t1];
		const Vec3 p1 = vertices[t.t2];
		const Vec3 p2 = vertices[t.t3];
		
		Vec3 p01, p02, normal;
		p01.subtract(p1, p0);
		p02.subtract(p2, p0);
		normal.cross(p02, p01);
		normal.normalise();
		
		normals[i] = normal;
		distances[i] = normal.dot(p0);
	}

	return true;
}

bool ITriangleMesh::intersect(const Vec3& t1, const Vec3& t2, const Vec3& t3, const Vec3& r1, const Vec3& r2, Vec3& p) {
	const Vec3 t21 = t2 - t1;
	const Vec3 t31 = t3 - t1;
	const Vec3 tn = t21 ^ t31;
	if (tn.isZero())
		return false;

	const Vec3 rn = r2 - r1;
	const Real tr = tn.dot(rn);
	if (Math::isZeroEps(tr))
		return false;

	const Real s = (tn.dot(r1) + tn.dot(t1))/tr;
	if (s < REAL_ZERO)
		return false;

	p.multiplyAdd(s, rn, r1);

	const Vec3 pt1 = p - t1;
	if (tn.dot(t21 ^ pt1) < REAL_ZERO)
		return false;
	
	const Vec3 t32 = t3 - t2;
	const Vec3 pt2 = p - t2;
	if (tn.dot(t32 ^ pt2) < REAL_ZERO)
		return false;

	const Vec3 pt3 = p - t3;
	if (tn.dot(pt3 ^ t31) < REAL_ZERO)
		return false;

	return true;
}

//------------------------------------------------------------------------------

void TriangleMesh::create(const Desc& desc) {
	if (!desc.isValid())
		throw std::runtime_error("TriangleMesh::create(): invalid description");

	// setup vertices
	vertices = desc.vertices;
	// setup triangles
	if (desc.bCook && !ITriangleMesh::findTriangles(vertices, triangles))
		throw std::runtime_error("TriangleMesh::create(): unable to find triangles");
	else
		triangles = desc.triangles;

	// setup normals
	if (!ITriangleMesh::findNormals(vertices, triangles, normals, distances))
		throw std::runtime_error("TriangleMesh::create(): unable to find normals");
}

//------------------------------------------------------------------------------
