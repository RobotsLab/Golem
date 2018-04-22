/** @file TriangleMesh.h
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
#ifndef _GOLEM_MATH_TRIANGLEMESH_H_
#define _GOLEM_MATH_TRIANGLEMESH_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat34.h>
#include <vector>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

// hack: uncomment if the convex hull algorithm is supported
//#define _GOLEM_TRIANGLEMESH_CONVEX_HULL

//------------------------------------------------------------------------------

/** Triangle. */
class Triangle {
public:
	/** indices */
	union {
		struct {
			U32 t1, t2, t3;
		};
		U32 t[3];
	};

	/** Default constructor does not do any initialisation.
	*/
	inline Triangle() {}

	/** Initialises from 3 scalar parameters.
	*	@param	t1		Value to initialise t1 component.
	*	@param	t2		Value to initialise t1 component.
	*	@param	t3		Value to initialise t1 component.
	*/
	inline Triangle(U32 t1, U32 t2, U32 t3) : t1(t1), t2(t2), t3(t3) {}
		
	/**	Copy constructor.
	*/
	inline Triangle(const Triangle &t) : t1(t.t1), t2(t.t2), t3(t.t3) {}

	/** Writes out the 3 values to dest.
	*	@param	t	Array to write elements to.
	*/
	inline void get(U16 t[]) const {
		t[0] = (U16)this->t1;
		t[1] = (U16)this->t2;
		t[2] = (U16)this->t3;
	}

	/** Writes out the 3 values to dest.
	*	@param	t	Array to write elements to.
	*/
	inline void get(U32 t[]) const {
		t[0] = (U32)this->t1;
		t[1] = (U32)this->t2;
		t[2] = (U32)this->t3;
	}

	/** reads 3 consecutive values from the ptr passed
	*/
	inline void  set(const U16* t) {
		t1 = (U32)t[0];
		t2 = (U32)t[1];
		t3 = (U32)t[2];
	}

	/** reads 3 consecutive values from the ptr passed
	*/
	inline void set(const U32* t) {
		t1 = (U32)t[0];
		t2 = (U32)t[1];
		t3 = (U32)t[2];
	}
};

//------------------------------------------------------------------------------

/** Triangle mesh interface. */
class ITriangleMesh {
public:
	virtual ~ITriangleMesh() {}
	/** Vertices of the mesh. */
	virtual const std::vector<Vec3>& getVertices() const = 0;
	/** Indices of triangles of the mesh. */
	virtual const std::vector<Triangle>& getTriangles() const = 0;
	/** Normals of the mesh faces. */
	virtual const std::vector<Vec3>& getNormals() const = 0;
	/** Distances to the origin of the mesh faces. */
	virtual const std::vector<Real>& getDistances() const = 0;

	/** Find triangles of a convex mesh given vertices. */
	static bool findTriangles(const std::vector<Vec3>& vertices, std::vector<Triangle>& triangles);
	/** Find normals given vertices and triangles. */
	static bool findNormals(const std::vector<Vec3>& vertices, const std::vector<Triangle>& triangles, std::vector<Vec3>& normals, std::vector<Real>& distances);
	/** Triangle (t) - ray (r) intersection point (p). */
	static bool intersect(const Vec3& t1, const Vec3& t2, const Vec3& t3, const Vec3& r1, const Vec3& r2, Vec3& p);
};

//------------------------------------------------------------------------------

/** Triangle mesh. */
class TriangleMesh : public ITriangleMesh {
public:
	/** Mesh description */
	class Desc {
	public:
		/** Vertices of the mesh. */
		std::vector<Vec3> vertices;
		/** Indices of triangles of the mesh (clockwise order). */
		std::vector<Triangle> triangles;
		/** Cooking flag for convex mesh. */
		bool bCook;

		Desc() {
			Desc::setToDefault();
		}

		Desc(const Desc& desc) {
			*this = desc;
		}

		/** virtual destructor */
		virtual ~Desc() {}
		
		/** Sets the parameters to the default values. */
		virtual void setToDefault() {
			vertices.clear();
			triangles.clear();
			bCook = false;
		}

		virtual bool isValid() const {
			if (vertices.empty() || !bCook && triangles.empty())
				return false;

			return true;
		}
	};

//protected:
public:
	/** Vertices of the mesh. */
	std::vector<Vec3> vertices;
	/** Indices of triangles of the mesh (clockwise order). */
	std::vector<Triangle> triangles;
	/** Normals of the mesh faces. */
	std::vector<Vec3> normals;
	/** Distances to the origin of the mesh faces. */
	std::vector<Real> distances;
	
public:
	/** Default constructor resets vertices and triangles. */
	TriangleMesh() {
	}
	/** Constructs convex triangle mesh from description. */
	TriangleMesh(const Desc& desc) {
		TriangleMesh::create(desc);
	}
	
	/** Constructs convex triangle mesh from set of vertices. */
	void create(const Desc& desc);
	
	/** Checks if the mesh is valid. */
	virtual bool isValid() const {
		if (vertices.empty())
			return false;
#ifdef _GOLEM_TRIANGLEMESH_CONVEX_HULL
		if (triangles.empty())
			return false;
#endif

		return true;
	}
	
	/** Transforms mesh to the specified local pose. */
	inline void transform(const Mat34& localPose, const TriangleMesh& mesh) {
		if (this != &mesh)
			vertices.resize(mesh.vertices.size());
		
		for (size_t i = 0; i < vertices.size(); ++i)
			localPose.multiply(vertices[i], mesh.vertices[i]);

		if (this != &mesh)
			triangles = mesh.triangles;

		// setup normals
		(void)ITriangleMesh::findNormals(vertices, triangles, normals, distances);
	}

	/** Vertices of the mesh. */
	virtual const std::vector<Vec3>& getVertices() const {
		return vertices;
	}
	/** Indices of triangles of the mesh. */
	virtual const std::vector<Triangle>& getTriangles() const {
		return triangles;
	}
	/** Normals of the mesh faces. */
	virtual const std::vector<Vec3>& getNormals() const {
		return normals;
	}
	/** Distances to the origin of the mesh faces. */
	virtual const std::vector<Real>& getDistances() const {
		return distances;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_TRIANGLEMESH_H_*/
