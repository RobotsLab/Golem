/** @file Bounds.cpp
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

#include <Golem/Math/Bounds.h>
#ifdef _BOUNDS_PERFMON
#include <Golem/Sys/Context.h>
#endif
#include <stdexcept>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

/** Names of bounds. */
const char* Bounds::Name[] = {
	/** plane */
	"Bounding Plane",
	/** sphere */
	"Bounding Sphere",
	/** cylinder */
	"Bounding Cylinder",
	/** capsule */
	"Bounding Capsule",
	/** parallelepiped */
	"Bounding Box",
	/** generic triangle mesh */
	"Bounding Triangle Mesh",
	/** convex triangle mesh */
	"Bounding Convex Mesh",
};

#ifdef _BOUNDS_PERFMON
U32 Bounds::collisionConvex;
U32 Bounds::collisionConvexIter;
U32 Bounds::collisionConvexFound;

U32 Bounds::collisionPlanePlane;
U32 Bounds::collisionPlaneSphere;
U32 Bounds::collisionPlaneCylinder;
U32 Bounds::collisionPlaneBox;
U32 Bounds::collisionPlaneConvex;
U32 Bounds::collisionSphereSphere;
U32 Bounds::collisionSphereCylinder;
U32 Bounds::collisionSphereBox;
U32 Bounds::collisionSphereConvex;
U32 Bounds::collisionCylinderCylinder;
U32 Bounds::collisionCylinderBox;
U32 Bounds::collisionCylinderConvex;
U32 Bounds::collisionBoxBox;
U32 Bounds::collisionBoxConvex;
U32 Bounds::collisionConvexConvex;

void Bounds::resetLog() {
	collisionConvex = 0;
	collisionConvexIter = 0;
	collisionConvexFound = 0;
	
	collisionPlanePlane = 0;
	collisionPlaneSphere = 0;
	collisionPlaneCylinder = 0;
	collisionPlaneBox = 0;
	collisionPlaneConvex = 0;
	collisionSphereSphere = 0;
	collisionSphereCylinder = 0;
	collisionSphereBox = 0;
	collisionSphereConvex = 0;
	collisionCylinderCylinder = 0;
	collisionCylinderBox = 0;
	collisionCylinderConvex = 0;
	collisionBoxBox = 0;
	collisionBoxConvex = 0;
	collisionConvexConvex = 0;
}

void Bounds::writeLog(Context &context, const char *str) {
	context.debug(
		"%s: Cx {tot = %d, avr = %.2f, col = %d}, PP %d, PS %d, PCy %d, PB %d, PCx %d, SS %d, SCy %d, SB %d, SCx %d, CyCy %d, CyB %d, CyCx %d, BB %d, BCx %d, CxCx %d\n",
		str,
		collisionConvex,
		Real(collisionConvexIter)/Real(collisionConvex),
		collisionConvexFound,
		collisionPlanePlane,
		collisionPlaneSphere,
		collisionPlaneCylinder,
		collisionPlaneBox,
		collisionPlaneConvex,
		collisionSphereSphere,
		collisionSphereCylinder,
		collisionSphereBox,
		collisionSphereConvex,
		collisionCylinderCylinder,
		collisionCylinderBox,
		collisionCylinderConvex,
		collisionBoxBox,
		collisionBoxConvex,
		collisionConvexConvex
	);
}
#endif

#ifdef _COLLISION_DEBUG
namespace golem {
	bool bDebug1;
	bool bDebug2;
	Bounds::Ptr pBounds1;
	Bounds::Ptr pBounds2;
};
#endif // _COLLISION_DEBUG

//------------------------------------------------------------------------------

void BoundingPlane::create(const Desc& desc) {
	if (!desc.isValid())
		throw std::runtime_error("BoundingPlane::create(): invalid description");
	
	normal = normalBase = desc.normal;
	distance = distanceBase = desc.distance;
	setPose(desc.pose);
	//pose.setId(); // ignore pose
	group = desc.group;
	gridSize = desc.gridSize;
	gridDelta = desc.gridDelta;
}

void BoundingPlane::create(const Vec3& p0, const Vec3& p1, const Vec3& p2) {
	// TODO check if points are not identical nor co-linear
	Vec3 p01, p02;
	p01.subtract(p1, p0);
	p02.subtract(p2, p0);
	normal.cross(p02, p01);
	normal.normalise();
	distance = normal.dot(p0);
	//pose.setId();
	setPose(pose);
}

Bounds::Desc::Ptr BoundingPlane::toDesc() const {
	BoundingPlane::Desc* pDesc = new BoundingPlane::Desc;
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	
	pDesc->pose = pose;
	pDesc->group = group;
	pDesc->normal = normal;
	pDesc->distance = distance;
	pDesc->gridSize = gridSize;
	pDesc->gridDelta = gridDelta;
	return pBoundsDesc; 
}

void BoundingPlane::transform() {
	((const Mat33&)pose.R).multiply(normal, normalBase);
	distance = distanceBase + normal.dot(pose.p);
}

bool BoundingPlane::intersect(const Vec3& p) const {
	if (normal.dot(p) < distance)
		return true;
	return false;
}

bool BoundingPlane::intersect(const Vec3& p0, const Vec3& p1) const {
	if (normal.dot(p0) < distance || normal.dot(p1) < distance)
		return true;
	return false;
}

bool BoundingPlane::intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const {
	if (normal.dot(p0) < distance || normal.dot(p1) < distance || normal.dot(p2) < distance)
		return true;
	return false;
}

void BoundingPlane::combine(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		combine(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		combine(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		combine(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		combine(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		combine(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingPlane::combine(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::combine(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::combine(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::combine(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::combine(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::combine(const Vec3& p) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::combine(const Vec3 *p, U32 numOfPoints) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::set(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		set(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		set(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		set(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		set(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		set(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingPlane::set(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::set(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::set(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::set(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingPlane::set(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

Real BoundingPlane::getSurfaceDistance(const Vec3& p) const {
	return normal.dot(p) - distance;
}

//------------------------------------------------------------------------------

void BoundingSphere::create(const Desc &desc) {
	if (!desc.isValid())
		throw std::runtime_error("BoundingSphere::create(): invalid description");

	radius = desc.radius;
	pose = desc.pose;
	group = desc.group;
	numOfSlices = desc.numOfSlices;
	numOfStacks = desc.numOfStacks;
}

Bounds::Desc::Ptr BoundingSphere::toDesc() const {
	BoundingSphere::Desc* pDesc = new BoundingSphere::Desc;
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	
	pDesc->pose = pose;
	pDesc->group = group;
	pDesc->radius = radius;
	pDesc->numOfSlices = numOfSlices;
	pDesc->numOfStacks = numOfStacks;
	return pBoundsDesc; 
}

bool BoundingSphere::intersect(const Vec3& p) const {
	if (getPosition().distanceSqr(p) < radius*radius)
		return true;
	return false;
}

/** Tests intersection with the specified ray. */
bool BoundingSphere::intersect(const Vec3& p0, const Vec3& p1) const {
	// TODO
	ASSERT(false)
	return false;
}

/** Tests intersection with the specified triangle. */
bool BoundingSphere::intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const {
	// TODO
	ASSERT(false)
	return false;
};

void BoundingSphere::combine(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		combine(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		combine(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		combine(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		combine(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		combine(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingSphere::combine(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::combine(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::combine(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::combine(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::combine(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::combine(const Vec3& p) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::combine(const Vec3 *p, U32 numOfPoints) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::set(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		set(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		set(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		set(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		set(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		set(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingSphere::set(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::set(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::set(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::set(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingSphere::set(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

Real BoundingSphere::getSurfaceDistance(const Vec3& p) const {
	return getPosition().distance(p) - radius;
}

//------------------------------------------------------------------------------

void BoundingCylinder::Desc::createTriangleMesh(TriangleMesh& mesh) const {
	if (!isValid())
		throw std::runtime_error("BoundingCylinder::Desc::createTriangleMesh(): invalid description");

	const U32 VERTICES = 2 + 2*numOfSlices;
	mesh.vertices.resize(VERTICES);

	const U32 TRIANGLES = 4*numOfSlices;
	mesh.triangles.resize(TRIANGLES);

	const Vec3 p0(REAL_ZERO, REAL_ZERO, -REAL_HALF*length);
	const Vec3 p1(REAL_ZERO, REAL_ZERO, +REAL_HALF*length);
	const Real a = REAL_2_PI/Real(numOfSlices);
	
	// "central" vertices
	mesh.vertices[0] = p0;
	mesh.vertices[1] = p1;

	for (U32 i = 0; i < numOfSlices; ++i) {
		const U32 n[4] = {(2*i+2), (2*i+3), 2+(2*i+2)%(VERTICES-2), 2+(2*i+3)%(VERTICES-2)};

		Real sin, cos;
		Math::sinCos(a*Real(i), sin, cos);
		const Vec3 r(radius*cos, radius*sin, REAL_ZERO);

		// "rim" vertices
		mesh.vertices[n[0]].add(p0, r);
		mesh.vertices[n[1]].add(p1, r);

		// triangles (right-handed screw cylinder-outward direction) 
		//mesh.triangles[4*i+0] = Triangle(0, n[0], n[2]);
		//mesh.triangles[4*i+1] = Triangle(1, n[3], n[1]);
		//mesh.triangles[4*i+2] = Triangle(n[0], n[1], n[2]);
		//mesh.triangles[4*i+3] = Triangle(n[1], n[3], n[2]);
		mesh.triangles[4*i+0] = Triangle(0, n[2], n[0]);
		mesh.triangles[4*i+1] = Triangle(1, n[1], n[3]);
		mesh.triangles[4*i+2] = Triangle(n[0], n[2], n[1]);
		mesh.triangles[4*i+3] = Triangle(n[1], n[2], n[3]);
	}

	if (!ITriangleMesh::findNormals(mesh.vertices, mesh.triangles, mesh.normals, mesh.distances))
		throw std::runtime_error("BoundingCylinder::Desc::createTriangleMesh(): unable to find normals");
}

void BoundingCylinder::create(const Desc &desc) {
	if (!desc.isValid())
		throw std::runtime_error("BoundingCylinder::create(): invalid description");

	radius = desc.radius;
	length = desc.length;
	numOfSlices = desc.numOfSlices;

	desc.createTriangleMesh(data[0]); // throws
	
	setPose(desc.pose);
	group = desc.group;
}

Bounds::Desc::Ptr BoundingCylinder::toDesc() const {
	BoundingCylinder::Desc* pDesc = new BoundingCylinder::Desc;
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	
	pDesc->pose = pose;
	pDesc->group = group;
	pDesc->radius = radius;
	pDesc->length = length;
	pDesc->numOfSlices = numOfSlices;
	return pBoundsDesc; 
}

void BoundingCylinder::transform() {
	position.set(REAL_ZERO, REAL_ZERO, -REAL_HALF*length); // Z axis
	pose.multiply(position, position);
	
	normal.set(REAL_ZERO, REAL_ZERO, -REAL_ONE); // "bottom" surface normal
	((const Mat33&)pose.R).multiply(normal, normal);
	
	distance = normal.dot(position); // distance to the origin

	data[1].transform(this->pose, data[0]);
}

bool BoundingCylinder::intersect(const Vec3& p) const {
	Vec3 tmp;
	tmp.subtract(position, p);
	tmp.cross(normal, tmp);
	
	if (tmp.magnitudeSqr() >= radius*radius)
		return false;
	
	const Real d = normal.dot(p);
	if (d >= distance || d <= distance - length)
		return false;
	
	return true;
}

bool BoundingCylinder::intersect(const Vec3& p0, const Vec3& p1) const {
	// TODO
	ASSERT(false)
	return false;
}

bool BoundingCylinder::intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const {
	// TODO
	ASSERT(false)
	return false;
};

void BoundingCylinder::combine(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		combine(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		combine(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		combine(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		combine(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		combine(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingCylinder::combine(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::combine(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::combine(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::combine(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::combine(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::combine(const Vec3& p) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::combine(const Vec3 *p, U32 numOfPoints) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::set(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		set(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		set(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		set(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		set(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		set(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingCylinder::set(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::set(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::set(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::set(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingCylinder::set(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

Real BoundingCylinder::getSurfaceDistance(const Vec3& p) const {
	Vec3 tmp;
	tmp.subtract(position, p);
	tmp.cross(normal, tmp);
	Real dn = normal.dot(p);

	Real distance = numeric_const<Real>::MIN, d;
	d = tmp.magnitudeSqr() - radius*radius;
	if (distance < d)
		distance = d;
	d = dn - distance;
	if (distance < d)
		distance = d;
	d = (distance - length) - dn;
	if (distance < d)
		distance = d;
		
	return distance;
}

//------------------------------------------------------------------------------

void BoundingBox::create() {
	Vec3 tmp;
	
	normals[0] = Vec3::axisX();
	((const Mat33&)pose.R).multiply(normals[0], normals[0]);
	tmp.multiplyAdd(dimensions.v1, normals[0], pose.p);
	distances[0] = normals[0].dot(tmp);
	distances[3] = distances[0] - REAL_TWO*dimensions.v1;

	normals[1] = Vec3::axisY();
	((const Mat33&)pose.R).multiply(normals[1], normals[1]);
	tmp.multiplyAdd(dimensions.v2, normals[1], pose.p);
	distances[1] = normals[1].dot(tmp);
	distances[4] = distances[1] - REAL_TWO*dimensions.v2;
	
	normals[2] = Vec3::axisZ();
	((const Mat33&)pose.R).multiply(normals[2], normals[2]);
	tmp.multiplyAdd(dimensions.v3, normals[2], pose.p);
	distances[2] = normals[2].dot(tmp);
	distances[5] = distances[2] - REAL_TWO*dimensions.v3;

	vertices[0].multiplyAdd( - dimensions.v1, normals[0], pose.p);
	vertices[2] = vertices[0];
	vertices[0].multiplyAdd( - dimensions.v2, normals[1], vertices[0]);
	vertices[2].multiplyAdd( + dimensions.v2, normals[1], vertices[2]);		
	vertices[4] = vertices[0];
	vertices[0].multiplyAdd( - dimensions.v3, normals[2], vertices[0]); // 0 ---
	vertices[4].multiplyAdd( + dimensions.v3, normals[2], vertices[4]); // 4 --+
	vertices[6] = vertices[2];
	vertices[2].multiplyAdd( - dimensions.v3, normals[2], vertices[2]); // 2 -+-
	vertices[6].multiplyAdd( + dimensions.v3, normals[2], vertices[6]); // 6 -++

	vertices[1].multiplyAdd( + dimensions.v1, normals[0], pose.p);
	vertices[3] = vertices[1];
	vertices[1].multiplyAdd( - dimensions.v2, normals[1], vertices[1]);
	vertices[3].multiplyAdd( + dimensions.v2, normals[1], vertices[3]);
	vertices[5] = vertices[1];
	vertices[1].multiplyAdd( - dimensions.v3, normals[2], vertices[1]); // 1 +--
	vertices[5].multiplyAdd( + dimensions.v3, normals[2], vertices[5]); // 5 +-+
	vertices[7] = vertices[3];
	vertices[3].multiplyAdd( - dimensions.v3, normals[2], vertices[3]); // 3 ++-
	vertices[7].multiplyAdd( + dimensions.v3, normals[2], vertices[7]); // 7 +++
}

void BoundingBox::createFrame(Vec3& xb, Vec3& yb, const Vec3& zb) const {
	// base frame 'a'
	const Vec3 xa(Vec3::axisX());
	const Vec3 ya(Vec3::axisY());
	//const Vec3 za(Vec3::axisZ());	
	
	// target frame 'b'
	if (Math::abs(Math::abs(zb.dot(xa)) - REAL_ONE) > REAL_EPS) {
		xb.cross(xa, zb);
		yb.cross(xb, zb);
	}
	else {
		yb.cross(ya, zb);
		xb.cross(yb, zb);
	}
}

void BoundingBox::create(const Desc &desc) {
	if (!desc.isValid())
		throw std::runtime_error("BoundingBox::create(): invalid description");

	dimensions = desc.dimensions;
	setPose(desc.pose);
	group = desc.group;

	triangles[0 ] = Triangle(0, 6, 4);
	triangles[1 ] = Triangle(0, 2, 6);
	triangles[2 ] = Triangle(0, 4, 5);
	triangles[3 ] = Triangle(0, 5, 1);
	triangles[4 ] = Triangle(0, 1, 3);
	triangles[5 ] = Triangle(0, 3, 2);
	triangles[6 ] = Triangle(7, 1, 5);
	triangles[7 ] = Triangle(7, 3, 1);
	triangles[8 ] = Triangle(7, 2, 3);
	triangles[9 ] = Triangle(7, 6, 2);
	triangles[10] = Triangle(7, 5, 4);
	triangles[11] = Triangle(7, 4, 6);
}

Bounds::Desc::Ptr BoundingBox::toDesc() const {
	BoundingBox::Desc* pDesc = new BoundingBox::Desc;
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	
	pDesc->pose = pose;
	pDesc->group = group;
	pDesc->dimensions = dimensions;
	return pBoundsDesc; 
}

bool BoundingBox::intersect(const Vec3& p) const {
	Real distance;

	distance = normals[0].dot(p);
	if (distance >= distances[0] || distance <= distances[3])
		return false;
	
	distance = normals[1].dot(p);
	if (distance >= distances[1] || distance <= distances[4])
		return false;
	
	distance = normals[2].dot(p);
	if (distance >= distances[2] || distance <= distances[5])
		return false;
	
	return true;
}

/** Tests intersection with the specified ray. */
bool BoundingBox::intersect(const Vec3& p0, const Vec3& p1) const {
	// TODO
	ASSERT(false)
	return false;
}

/** Tests intersection with the specified triangle. */
bool BoundingBox::intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const {
	// TODO
	ASSERT(false)
	return false;
};

void BoundingBox::combine(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		combine(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		combine(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		combine(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		combine(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		combine(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingBox::combine(const BoundingPlane& plane) {
	Vec3 point;
	point.multiply(plane.getDistance(), plane.getNormal());
	combine(point);
}

void BoundingBox::combine(const BoundingSphere& sphere) {
	BoundingBox box;
	box.set(sphere);
	combine(box.getVertices(), 8);
}

void BoundingBox::combine(const BoundingCylinder& cylinder) {
	BoundingBox box;
	box.set(cylinder);
	combine(box.getVertices(), 8);
}

void BoundingBox::combine(const BoundingBox& box) {
	combine(box.getVertices(), 8);
}

void BoundingBox::combine(const BoundingConvexMesh& mesh) {
	combine(&mesh.getVertices().front(), (U32)mesh.getVertices().size());
}

void BoundingBox::combine(const Vec3& p) {
	combine(&p, 1);
}

void BoundingBox::combine(const Vec3 p[], U32 numOfPoints) {
	Mat34 poseInv;
	poseInv.setInverseRT(pose);
	
	// min, max
	Vec3 min(-dimensions.v1, -dimensions.v2, -dimensions.v3);
	Vec3 max(+dimensions.v1, +dimensions.v2, +dimensions.v3);
	
	for (U32 i = 0; i < numOfPoints; ++i) {
		Vec3 point;
		poseInv.multiply(point, p[i]);

		if (min.v1 > point.v1) min.v1 = point.v1;
		if (max.v1 < point.v1) max.v1 = point.v1;
		if (min.v2 > point.v2) min.v2 = point.v2;
		if (max.v2 < point.v2) max.v2 = point.v2;
		if (min.v3 > point.v3) min.v3 = point.v3;
		if (max.v3 < point.v3) max.v3 = point.v3;
	}

	// translation vector
	Vec3 diff(REAL_HALF*(max.v1 + min.v1), REAL_HALF*(max.v2 + min.v2), REAL_HALF*(max.v3 + min.v3));
	((const Mat33&)pose.R).multiply(diff, diff);
	pose.p.add(pose.p, diff);
	// rotation matrix is unchanged
	
	// dimensions
	dimensions.set(REAL_HALF*(max.v1 - min.v1), REAL_HALF*(max.v2 - min.v2), REAL_HALF*(max.v3 - min.v3));
	
	create();
}

void BoundingBox::set(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		set(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		set(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		set(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		set(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		set(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingBox::set(const BoundingPlane& plane) {
	// dimensions
	dimensions.set(REAL_ONE);	
	// translation vector
	pose.p.multiply(plane.getDistance(), plane.getNormal());
	// rotation matrix
	Vec3 xb, yb;
	createFrame(xb, yb, plane.getNormal());
	pose.R.fromAxes(xb, yb, plane.getNormal());
	
	create();
}

void BoundingBox::set(const BoundingSphere& sphere) {
	// dimensions
	dimensions.set(sphere.getRadius());
	// translation vector
	pose.p = sphere.getPosition();
	// rotation matrix
	pose.R.setId();
	
	create();
}

void BoundingBox::set(const BoundingCylinder& cylinder) {
	// dimensions
	dimensions.set(cylinder.getRadius(), cylinder.getRadius(), REAL_HALF*cylinder.getLength());
	// pose
	pose = cylinder.getPose();
	
	create();
}

void BoundingBox::set(const BoundingBox& box) {
	operator = (box);
}

void BoundingBox::set(const BoundingConvexMesh& mesh) {
	// TODO use better algorithm
	Vec3 min(numeric_const<Real>::MAX), max(numeric_const<Real>::MIN);
	for (std::vector<Vec3>::const_iterator i = mesh.getVertices().begin(); i != mesh.getVertices().end(); ++i) {
		if (min.v1 > i->v1) min.v1 = i->v1;
		if (min.v2 > i->v2) min.v2 = i->v2;
		if (min.v3 > i->v3) min.v3 = i->v3;
		if (max.v1 < i->v1) max.v1 = i->v1;
		if (max.v2 < i->v2) max.v2 = i->v2;
		if (max.v3 < i->v3) max.v3 = i->v3;
	}

	dimensions.set(REAL_HALF*(max.v1 - min.v1), REAL_HALF*(max.v2 - min.v2), REAL_HALF*(max.v3 - min.v3));
	pose.p.add(min, dimensions);
	pose.R.setId(); // fixed
	
	create();
}

Real BoundingBox::getSurfaceDistance(const Vec3& p) const {
	Real distance = numeric_const<Real>::MIN;
	
	for (U32 i = 0; i < 3; ++i) {
		const Real dn = normals[i].dot(p);

		const Real d1 = dn - distances[i];
		if (distance < d1)
			distance = d1;

		const Real d2 = distances[3 + i] - dn;
		if (distance < d2)
			distance = d2;
	}
		
	return distance;
}

//------------------------------------------------------------------------------

void BoundingConvexMesh::create(const Desc& desc) {
	if (!desc.isValid())
		throw std::runtime_error("BoundingConvexMesh::create(): invalid description");
	
	data[0].create(desc); // throws

	setPose(desc.pose);
	group = desc.group;
}

Bounds::Desc::Ptr BoundingConvexMesh::toDesc() const {
	BoundingConvexMesh::Desc* pDesc = new BoundingConvexMesh::Desc;
	Bounds::Desc::Ptr pBoundsDesc(pDesc);
	
	pDesc->pose = pose;
	pDesc->group = group;
	pDesc->vertices = getVertices();
	pDesc->bCook = true;
	pDesc->triangles = getTriangles();

	return pBoundsDesc; 
}

bool BoundingConvexMesh::intersect(const Vec3& p) const {
	const U32 TRIANGLES = (U32)data[1].triangles.size();
	for (U32 i = 0; i < TRIANGLES; ++i)
		if (data[1].normals[i].dot(p) >= data[1].distances[i])
			return false;

	return true;
}

/** Tests intersection with the specified ray. */
bool BoundingConvexMesh::intersect(const Vec3& p0, const Vec3& p1) const {
	// TODO
	ASSERT(false)
	return false;
}

/** Tests intersection with the specified triangle. */
bool BoundingConvexMesh::intersect(const Vec3& p0, const Vec3& p1, const Vec3& p2) const {
	// TODO
	ASSERT(false)
	return false;
};

void BoundingConvexMesh::combine(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		combine(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		combine(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		combine(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		combine(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		combine(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingConvexMesh::combine(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::combine(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::combine(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::combine(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::combine(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::combine(const Vec3& p) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::combine(const Vec3 *p, U32 numOfPoints) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::set(const Bounds &bounds) {
	switch (bounds.getType()) {
	case TYPE_PLANE:
		set(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case TYPE_SPHERE:
		set(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case TYPE_CYLINDER:
		set(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case TYPE_BOX:
		set(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case TYPE_CONVEX_MESH:
		set(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		ASSERT(false)
		break;
	}
}

void BoundingConvexMesh::set(const BoundingPlane& plane) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::set(const BoundingSphere& sphere) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::set(const BoundingCylinder& cylinder) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::set(const BoundingBox& box) {
	// TODO
	ASSERT(false)
}

void BoundingConvexMesh::set(const BoundingConvexMesh& mesh) {
	// TODO
	ASSERT(false)
}

Real BoundingConvexMesh::getSurfaceDistance(const Vec3& p) const {
	Real distance = numeric_const<Real>::MIN;
	const U32 TRIANGLES = (U32)data[1].triangles.size();
	for (U32 i = 0; i < TRIANGLES; ++i) {
		const Real d = data[1].normals[i].dot(p) - data[1].distances[i];
		if (distance < d)
			distance = d;
	}
		
	return distance;
}

//------------------------------------------------------------------------------

bool Bounds::intersect2(const Vec3 *vertices0, U32 numOfVertices0, const Vec3 &centre0, const Vec3 *vertices1, U32 numOfVertices1, const Vec3 &centre1) const {
#ifdef _BOUNDS_PERFMON
	++Bounds::collisionConvex;
#endif

	class Projection {
	public:
		const Vec3 *vertices;
		const U32 numOfVertices;
		Real p[2];
		U32 n[2];

		Projection(const Vec3 *vertices, U32 numOfVertices) :
			vertices(vertices), numOfVertices(numOfVertices)
		{
			ASSERT(numOfVertices >= 2)
		}

		inline void min1(const Vec3 &axis) {
			p[0] = axis.dot(vertices[0]);
			n[0] = 0;

			for (U32 i = 1; i < numOfVertices; ++i) {
				const Real projection = axis.dot(vertices[i]);
				if (p[0] > projection) {
					p[0] = projection;
					n[0] = i;
				}
			}
		}
		
		inline void min2(const Vec3 &axis) {
			p[0] = axis.dot(vertices[0]);
			n[0] = 0;
			p[1] = axis.dot(vertices[1]);
			n[1] = 1;
			if (p[1] < p[0]) {
				std::swap(p[1], p[0]);
				std::swap(n[1], n[0]);
			}

			for (U32 i = 2; i < numOfVertices; ++i) {
				const Real projection = axis.dot(vertices[i]);
				if (p[0] > projection) {
					p[1] = p[0];
					p[0] = projection;
					n[1] = n[0];
					n[0] = i;
				}
			}
		}
		
		inline void max1(const Vec3 &axis) {
			p[0] = axis.dot(vertices[0]);
			n[0] = 0;
			
			for (U32 i = 1; i < numOfVertices; ++i) {
				const Real projection = axis.dot(vertices[i]);
				if (p[0] < projection) {
					p[0] = projection;
					n[0] = i;
				}
			}
		}
		
		inline void max2(const Vec3 &axis) {
			p[0] = axis.dot(vertices[0]);
			n[0] = 0;
			p[1] = axis.dot(vertices[1]);
			n[1] = 1;
			if (p[1] > p[0]) {
				std::swap(p[1], p[0]);
				std::swap(n[1], n[0]);
			}
			
			for (U32 i = 2; i < numOfVertices; ++i) {
				const Real projection = axis.dot(vertices[i]);
				if (p[0] < projection) {
					p[1] = p[0];
					p[0] = projection;
					n[1] = n[0];
					n[0] = i;
				}
			}
		}
	};

	// projections
	Projection max0(vertices0, numOfVertices0), min1(vertices1, numOfVertices1);
	
	// initial axis
	Vec3 axis;
	axis.subtract(centre1, centre0);
	// ensure that centre0 and centre1 do not overlap
	if (axis.normalise() <= REAL_EPS) {
#ifdef _BOUNDS_PERFMON
				++Bounds::collisionConvexFound;
#endif
		return true;
	}

#ifdef _BOUNDS_PERFMON
	++Bounds::collisionConvexIter;
#endif

	// compute projections
	max0.max2(axis);
	min1.min1(axis);	
	
	// initial delta
	Real delta = max0.p[0] - min1.p[0];

	// maximum number of iterations
	const U32 iterations = numOfVertices0;
	
	// Perform greedy (asymmetric) search for an axis such that two sets of projections
	// of vertices of the objects onto the axis do not "overlap" (no collision).
	// Axes are created from 3 non-colinear points which 2 belongs to the "left object",
	// and one to the "right object".
	while (delta > REAL_EPS) {
		U32 n0 = max0.n[0], n1 = min1.n[0], n2 = max0.n[1];

		const Vec3 &p0 = vertices0[n0];
		const Vec3 &p1 = vertices1[n1];
		Vec3 p01, p02;
		p01.subtract(p1, p0);
		
		for (U32 i = 0; ; ++i) {
			if (i >= iterations) {
#ifdef _BOUNDS_PERFMON
				++Bounds::collisionConvexFound;
#endif
				return true;
			}
#ifdef _BOUNDS_PERFMON
			++Bounds::collisionConvexIter;
#endif

			// check n2 first
			const U32 j = (i + n2)%iterations;
			// n2 must be different than n0
			if (j == n0)
				continue;
			const Vec3 *p2 = &vertices0[j];

			p02.subtract(*p2, p0);

			// find new projection axis
			Vec3 newAxis;
			newAxis.cross(p02, p01);

			// ensure that p02 and p01 are not colinear
			if (newAxis.normalise() <= REAL_EPS)
				continue;

			// preserve minima/maxima of projected vertices
			if (newAxis.dot(axis) <= REAL_ZERO)
				newAxis.setNegative();

			// compute projections
			max0.max2(newAxis);
			min1.min1(newAxis);

			// new delta
			Real newDelta = max0.p[0] - min1.p[0];
			if (delta > newDelta) {
				axis = newAxis;
				delta = newDelta;
				break;
			}
		}		
	}

	return false;
}

//bool Bounds::intersect(const Vec3 *vertices0, U32 numOfVertices0, const Vec3 &centre0, const Vec3 *vertices1, U32 numOfVertices1, const Vec3 &centre1) const {
//#ifdef _BOUNDS_PERFMON
//	++collisionConvex;
//#endif
//
//	class Projection {
//	public:
//		const Vec3 *vertices;
//		const U32 numOfVertices;
//		Real p[2];
//		U32 n[2];
//
//		Projection(const Vec3 *vertices, U32 numOfVertices) :
//			vertices(vertices), numOfVertices(numOfVertices)
//		{
//			ASSERT(numOfVertices >= 2)
//		}
//
//		inline void min(const Vec3 &axis) {
//			p[0] = axis.dot(vertices[0]);
//			n[0] = 0;
//			p[1] = axis.dot(vertices[1]);
//			n[1] = 1;
//			if (p[1] < p[0]) {
//				std::swap(p[1], p[0]);
//				std::swap(n[1], n[0]);
//			}
//
//			for (U32 i = 2; i < numOfVertices; ++i) {
//				const Real projection = axis.dot(vertices[i]);
//				if (p[0] > projection) {
//					p[1] = p[0];
//					p[0] = projection;
//					n[1] = n[0];
//					n[0] = i;
//				}
//			}
//		}
//		
//		inline void max(const Vec3 &axis) {
//			p[0] = axis.dot(vertices[0]);
//			n[0] = 0;
//			p[1] = axis.dot(vertices[1]);
//			n[1] = 1;
//			if (p[1] > p[0]) {
//				std::swap(p[1], p[0]);
//				std::swap(n[1], n[0]);
//			}
//			
//			for (U32 i = 2; i < numOfVertices; ++i) {
//				const Real projection = axis.dot(vertices[i]);
//				if (p[0] < projection) {
//					p[1] = p[0];
//					p[0] = projection;
//					n[1] = n[0];
//					n[0] = i;
//				}
//			}
//		}
//	};
//
//#ifdef _BOUNDS_PERFMON
//	++collisionConvexIter;
//#endif
//
//	// projections
//	Projection max0(vertices0, numOfVertices0), min1(vertices1, numOfVertices1);
//	
//	// initial axis
//	Vec3 axis;
//	axis.subtract(centre1, centre0);
//	axis.normalise();
//
//	// compute projections
//	max0.max(axis);
//	min1.min(axis);	
//	
//	// initial delta
//	Real delta = max0.p[0] - min1.p[0];
//
//	// maximum number of iterations
//	const U32 iterations = numOfVertices0 + numOfVertices1;
//	
//	while (delta > REAL_EPS) {
//		U32 n0 = max0.n[0], n1 = min1.n[0], n20 = max0.n[1], n21 = min1.n[1];
//
//		const Vec3 &p0 = vertices0[n0];
//		const Vec3 &p1 = vertices1[n1];
//		Vec3 p01, p02;
//		p01.subtract(p1, p0);
//		
//		bool b20 = max0.p[0] - max0.p[1] < min1.p[1] - min1.p[0];
//
//		for (U32 i = 0; ; ++i) {
//			if (i >= iterations) {
//#ifdef _BOUNDS_PERFMON
//				++collisionConvexFound;
//#endif
//				return true;
//			}
//#ifdef _BOUNDS_PERFMON
//			++collisionConvexIter;
//#endif
//
//			const Vec3 *p2;
//			if (i < 2) {
//				p2 = b20 ? &vertices0[n20] : &vertices1[n21];
//				b20 = !b20;
//			}
//			else if (i < numOfVertices0) {
//				if (i == n0 || i == n20)
//					continue;
//				p2 = &vertices0[i];
//			}
//			else {
//				U32 j = i - numOfVertices0;
//				if (j == n1 || j == n21)
//					continue;
//				p2 = &vertices1[j];
//			}
//
//			p02.subtract(*p2, p0);
//
//			// find new projection axis
//			Vec3 newAxis;
//			newAxis.cross(p02, p01);
//
//			// check if p02 and p01 are not colinear
//			if (newAxis.normalise() <= REAL_EPS)
//				continue;
//
//			// preserve minima/maxima of projected vertices
//			if (newAxis.dot(axis) <= REAL_ZERO)
//				newAxis.setNegative();
//
//			// compute projections
//			max0.max(newAxis);
//			min1.min(newAxis);
//
//			// new delta
//			Real newDelta = max0.p[0] - min1.p[0];
//			if (delta >= newDelta) {
//				delta = newDelta;
//				axis = newAxis;
//				break;
//			}
//		}		
//	}
//
//	return false;
//}

//------------------------------------------------------------------------------

/** Tests intersection between specified planes. */
bool Bounds::intersect(const BoundingPlane& plane0, const BoundingPlane& plane1) const {
#ifdef _BOUNDS_PERFMON
	++collisionPlanePlane;
#endif
	if (Math::abs(plane0.getNormal().dot(plane1.getNormal())) < REAL_ONE)
		return true;

	return false;
}

/** Tests intersection between specified plane and sphere. */
bool Bounds::intersect(const BoundingPlane& plane, const BoundingSphere& sphere) const {
#ifdef _BOUNDS_PERFMON
	++collisionPlaneSphere;
#endif
	if (plane.getNormal().dot(sphere.getPosition()) < plane.getDistance() + sphere.getRadius())
		return true;

	return false;
}

/** Tests intersection between specified plane and cylinder. */
bool Bounds::intersect(const BoundingPlane& plane, const BoundingCylinder& cylinder) const {
#ifdef _BOUNDS_PERFMON
	++collisionPlaneCylinder;
#endif
	Real projection = plane.getNormal().dot(cylinder.getNormal());
	projection = cylinder.getRadius()*Math::sqrt(REAL_ONE - projection*projection);

	if (plane.getNormal().dot(cylinder.getPosition()) < plane.getDistance() + projection)
		return true;

	Vec3 tmp;
	tmp.multiplyAdd( - cylinder.getLength(), cylinder.getNormal(), cylinder.getPosition());
	if (plane.getNormal().dot(tmp) < plane.getDistance() + projection)
		return true;
	
	return false;
}

/** Tests intersection between specified plane and box. */
bool Bounds::intersect(const BoundingPlane& plane, const BoundingBox& box) const {
#ifdef _BOUNDS_PERFMON
	++collisionPlaneBox;
#endif
	for (U32 i = 0; i < 8; ++i)
		if (plane.getNormal().dot(box.getVertices()[i]) < plane.getDistance())
			return true;

	return false;
}

/** Tests intersection between specified plane and convex mesh. */
bool Bounds::intersect(const BoundingPlane& plane, const BoundingConvexMesh& mesh) const {
#ifdef _BOUNDS_PERFMON
	++collisionPlaneConvex;
#endif
	const U32 VERTICES = (U32)mesh.getVertices().size();
	for (U32 i = 0; i < VERTICES; ++i)
		if (plane.getNormal().dot(mesh.getVertices()[i]) < plane.getDistance())
			return true;

	return false;
}

/** Tests intersection between specified spheres. */
bool Bounds::intersect(const BoundingSphere& sphere0, const BoundingSphere& sphere1) const {
#ifdef _BOUNDS_PERFMON
	++collisionSphereSphere;
#endif
	if (sphere0.getPosition().distance(sphere1.getPosition()) < sphere0.getRadius() + sphere1.getRadius())
		return true;

	return false;
}

/** Tests intersection between specified sphere and cylinder. */
bool Bounds::intersect(const BoundingSphere& sphere, const BoundingCylinder& cylinder) const {
#ifdef _BOUNDS_PERFMON
	++collisionSphereCylinder;
#endif
	Vec3 tmp;
	tmp.subtract(cylinder.getPosition(), sphere.getPosition());
	tmp.cross(cylinder.getNormal(), tmp);
	
	if (tmp.magnitude() >= cylinder.getRadius() + sphere.getRadius())
		return false;
	
	const Real d = cylinder.getNormal().dot(sphere.getPosition());
	if (d >= cylinder.getDistance() + sphere.getRadius() || 
		d <= cylinder.getDistance() - sphere.getRadius() - cylinder.getLength())
		return false;

	return true;
}

/** Tests intersection between specified sphere and box. */
bool Bounds::intersect(const BoundingSphere& sphere, const BoundingBox& box) const {
#ifdef _BOUNDS_PERFMON
	++collisionSphereBox;
#endif
	Real d;

	d = box.getNormals()[0].dot(sphere.getPosition());
	if (d >= box.getDistances()[0] + sphere.getRadius() || 
		d <= box.getDistances()[3] - sphere.getRadius())
		return false;
	
	d = box.getNormals()[1].dot(sphere.getPosition());
	if (d >= box.getDistances()[1] + sphere.getRadius() || 
		d <= box.getDistances()[4] - sphere.getRadius())
		return false;
	
	d = box.getNormals()[2].dot(sphere.getPosition());
	if (d >= box.getDistances()[2] + sphere.getRadius() || 
		d <= box.getDistances()[5] - sphere.getRadius())
		return false;
	
	return true;
}

/** Tests intersection between specified sphere and convex mesh. */
bool Bounds::intersect(const BoundingSphere& sphere, const BoundingConvexMesh& mesh) const {
#ifdef _BOUNDS_PERFMON
	++collisionSphereConvex;
#endif
	// this works only for convex meshes!
	const U32 TRIANGLES = (U32)mesh.getTriangles().size();
	for (U32 i = 0; i < TRIANGLES; ++i)
		if (mesh.getNormals()[i].dot(sphere.getPosition()) >= mesh.getDistances()[i] + sphere.getRadius())
			return false;

	return true;
}

/** Tests intersection between specified cylinders. */
bool Bounds::intersect(const BoundingCylinder& cylinder0, const BoundingCylinder& cylinder1) const {
#ifdef _BOUNDS_PERFMON
	++collisionCylinderCylinder;
#endif
	// TODO proper collision detection cylinder-cylinder without convex mesh approx

	Vec3 centre0;
	centre0.multiplyAdd(- REAL_HALF*cylinder0.getLength(), cylinder0.getNormal(), cylinder0.getPosition());
	
	Vec3 centre1;
	centre1.multiplyAdd(- REAL_HALF*cylinder1.getLength(), cylinder1.getNormal(), cylinder1.getPosition());

	return Bounds::intersect(
		&cylinder0.getVertices().front(), (U32)cylinder0.getVertices().size(), centre0, &cylinder1.getVertices().front(), (U32)cylinder1.getVertices().size(), centre1
	);
}

/** Tests intersection between specified cylinder and box. */
bool Bounds::intersect(const BoundingCylinder& cylinder, const BoundingBox& box) const {
#ifdef _BOUNDS_PERFMON
	++collisionCylinderBox;
#endif
	// TODO proper collision detection cylinder-box without convex mesh approx
	
	Vec3 centre;
	centre.multiplyAdd(- REAL_HALF*cylinder.getLength(), cylinder.getNormal(), cylinder.getPosition());

	return Bounds::intersect(
		&cylinder.getVertices().front(), (U32)cylinder.getVertices().size(), centre, box.getVertices(), 8, box.getPose().p
	);
}

/** Tests intersection between specified cylinder and convex mesh. */
bool Bounds::intersect(const BoundingCylinder& cylinder, const BoundingConvexMesh& mesh) const {
#ifdef _BOUNDS_PERFMON
	++collisionCylinderConvex;
#endif
	// TODO proper collision detection cylinder-mesh without convex mesh approx
	
	Vec3 centre0;
	centre0.multiplyAdd(- REAL_HALF*cylinder.getLength(), cylinder.getNormal(), cylinder.getPosition());
	
	// compute centroid
	const U32 VERTICES = (U32)mesh.getVertices().size();
	Vec3 centre1(REAL_ZERO);
	for (U32 i = 0; i < VERTICES; ++i)
		centre1.add(centre1, mesh.getVertices()[i]);
	centre1.multiply(REAL_ONE/Real(VERTICES), centre1);

	return Bounds::intersect(
		&cylinder.getVertices().front(), (U32)cylinder.getVertices().size(), centre0, &mesh.getVertices().front(), VERTICES, centre1
	);
}

/** Tests intersection of the specified boxs. */
bool Bounds::intersect(const BoundingBox& box0, const BoundingBox& box1) const {
#ifdef _BOUNDS_PERFMON
	++collisionBoxBox;
#endif
	// TODO proper collision detection box-box without using generic mesh-mesh routines
	return Bounds::intersect(
		box0.getVertices(), 8, box0.getPose().p, box1.getVertices(), 8, box1.getPose().p
	);
}

/** Tests intersection between specified cylinder and convex mesh. */
bool Bounds::intersect(const BoundingBox& box, const BoundingConvexMesh& mesh) const {
#ifdef _BOUNDS_PERFMON
	++collisionBoxConvex;
#endif
	// compute centroid
	const U32 VERTICES = (U32)mesh.getVertices().size();
	Vec3 centre(REAL_ZERO);
	for (U32 i = 0; i < VERTICES; ++i)
		centre.add(centre, mesh.getVertices()[i]);
	centre.multiply(REAL_ONE/Real(VERTICES), centre);

	return Bounds::intersect(
		box.getVertices(), 8, box.getPose().p, &mesh.getVertices().front(), VERTICES, centre
	);
}

/** Tests intersection between specified convex meshes. */
bool Bounds::intersect(const BoundingConvexMesh& mesh0, const BoundingConvexMesh& mesh1) const {
#ifdef _BOUNDS_PERFMON
	++collisionConvexConvex;
#endif
	// compute centroid
	const U32 VERTICES0 = (U32)mesh0.getVertices().size();
	Vec3 centre0(REAL_ZERO);
	for (U32 i = 0; i < VERTICES0; ++i)
		centre0.add(centre0, mesh0.getVertices()[i]);
	centre0.multiply(REAL_ONE/Real(VERTICES0), centre0);
	
	// compute centroid
	const U32 VERTICES1 = (U32)mesh1.getVertices().size();
	Vec3 centre1(REAL_ZERO);
	for (U32 i = 0; i < VERTICES1; ++i)
		centre1.add(centre1, mesh1.getVertices()[i]);
	centre1.multiply(REAL_ONE/Real(VERTICES1), centre1);

	return Bounds::intersect(
		&mesh0.getVertices().front(), VERTICES0, centre0, &mesh1.getVertices().front(), VERTICES1, centre1
	);
}

//------------------------------------------------------------------------------
