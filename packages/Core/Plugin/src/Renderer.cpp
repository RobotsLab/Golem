/** @file Renderer.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Plugin/Renderer.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <memory.h>

using namespace golem;

//------------------------------------------------------------------------------

const RGBA RGBA::BLACK		= RGBA(  0,   0,   0, 255);
const RGBA RGBA::RED		= RGBA(255,   0,   0, 255);
const RGBA RGBA::GREEN		= RGBA(  0, 255,   0, 255);
const RGBA RGBA::YELLOW		= RGBA(255, 255,   0, 255);
const RGBA RGBA::BLUE		= RGBA(  0,   0, 255, 255);
const RGBA RGBA::MAGENTA	= RGBA(255,   0, 255, 255);
const RGBA RGBA::CYAN		= RGBA(  0, 255, 255, 255);
const RGBA RGBA::WHITE		= RGBA(255, 255, 255, 255);

//------------------------------------------------------------------------------

const RGBA BoundsRenderer::glWireColourDflt = RGBA(127, 127, 127, 255);
const RGBA BoundsRenderer::glSolidColourDflt = RGBA(127, 127, 127, 127);
const RGBA BoundsRenderer::glShadowColourDflt = RGBA(12, 25, 37, 255);
const Real BoundsRenderer::glLineWidthDflt = 1.0;

const ::GLfloat BoundsRenderer::glMatDflt[16] = {
	1.0f, 0.0f, 0.0f, 0.0f, 
	0.0f, 1.0f, 0.0f, 0.0f, 
	0.0f, 0.0f, 1.0f, 0.0f, 
	0.0f, 0.0f, 0.0f, 1.0f, 
};
const ::GLfloat BoundsRenderer::glShadowMatDflt[16] = {
	1.0f, 0.0f, 0.0f, 0.0f, 
	0.0f, 1.0f, 0.0f, 0.0f, 
	0.0f, 0.0f, 0.0f, 0.0f, 
	0.0f, 0.0f, 0.0f, 1.0f, 
};

BoundsRenderer::Data BoundsRenderer::cylinderTriangles;
BoundsRenderer::Data BoundsRenderer::cylinderNormals;
BoundsRenderer::Data BoundsRenderer::cylinderLines;

BoundsRenderer::BoundsRenderer() {
	reset();
}

void BoundsRenderer::reset() {
	::memcpy(glMat, glMatDflt, sizeof(glMat));
	mat.setId();
	::memcpy(glShadowMat, glShadowMatDflt, sizeof(glShadowMat));
	glLineWidth = ::GLfloat(glLineWidthDflt);
	glWireColour = glWireColourDflt;
	glSolidColour = glSolidColourDflt;
	glShadowColour = glShadowColourDflt;
	verticesData.resize(0);
	normalsData.resize(0);

	if (cylinderTriangles.empty() || cylinderNormals.empty() || cylinderLines.empty())
		initCylinder();
}

void BoundsRenderer::initCylinder() {
	const Vec3 p0(REAL_ZERO, REAL_ZERO, -REAL_HALF);
	const Vec3 p1(REAL_ZERO, REAL_ZERO, +REAL_HALF);
	const Vec3 n0(REAL_ZERO, REAL_ZERO, -REAL_ONE);
	const Vec3 n1(REAL_ZERO, REAL_ZERO, +REAL_ONE);
	const Real a = REAL_2_PI/Real(BOUNDING_CYLINDER_SLICES);
	
	cylinderTriangles.resize(BOUNDING_CYLINDER_SLICES*3*12);
	::GLfloat *pTriangles = &cylinderTriangles.front();
	cylinderNormals.resize(BOUNDING_CYLINDER_SLICES*3*12);
	::GLfloat *pNormals = &cylinderNormals.front();
	cylinderLines.resize(BOUNDING_CYLINDER_SLICES*3*24);
	::GLfloat *pLines = &cylinderLines.front();
	
	Vec3 prev0, prev1, next0, next1;
	prev0.set(REAL_ONE, REAL_ZERO, -REAL_HALF);
	prev1.set(REAL_ONE, REAL_ZERO, +REAL_HALF);
	for (U32 i = 0; i < BOUNDING_CYLINDER_SLICES; ++i) {
		Real sin, cos;
		Math::sinCos(a*Real(i + 1), sin, cos);
		const Vec3 n(cos, sin, REAL_ZERO);

		next0.add(p0, n);
		next1.add(p1, n);
		
		// bottom
		p0.getColumn3(&pTriangles[0]);
		next0.getColumn3(&pTriangles[3]);
		prev0.getColumn3(&pTriangles[6]);
		pTriangles += 9;
		n0.getColumn3(&pNormals[0]);
		n0.getColumn3(&pNormals[3]);
		n0.getColumn3(&pNormals[6]);
		pNormals += 9;
		p0.getColumn3(&pLines[0]);
		prev0.getColumn3(&pLines[3]);
		prev0.getColumn3(&pLines[6]);
		next0.getColumn3(&pLines[9]);
		next0.getColumn3(&pLines[12]);
		p0.getColumn3(&pLines[15]);
		pLines += 18;
		
		// top
		p1.getColumn3(&pTriangles[0]);
		prev1.getColumn3(&pTriangles[3]);
		next1.getColumn3(&pTriangles[6]);
		pTriangles += 9;
		n1.getColumn3(&pNormals[0]);
		n1.getColumn3(&pNormals[3]);
		n1.getColumn3(&pNormals[6]);
		pNormals += 9;
		p1.getColumn3(&pLines[0]);
		prev1.getColumn3(&pLines[3]);
		prev1.getColumn3(&pLines[6]);
		next1.getColumn3(&pLines[9]);
		next1.getColumn3(&pLines[12]);
		p1.getColumn3(&pLines[15]);
		pLines += 18;
		
		// side
		prev0.getColumn3(&pTriangles[0]);
		next0.getColumn3(&pTriangles[3]);
		prev1.getColumn3(&pTriangles[6]);
		next1.getColumn3(&pTriangles[9]);
		prev1.getColumn3(&pTriangles[12]);
		next0.getColumn3(&pTriangles[15]);
		pTriangles += 18;
		n.getColumn3(&pNormals[0]);
		n.getColumn3(&pNormals[3]);
		n.getColumn3(&pNormals[6]);
		n.getColumn3(&pNormals[9]);
		n.getColumn3(&pNormals[12]);
		n.getColumn3(&pNormals[15]);
		pNormals += 18;
		prev0.getColumn3(&pLines[0]);
		next0.getColumn3(&pLines[3]);
		next0.getColumn3(&pLines[6]);
		prev1.getColumn3(&pLines[9]);
		prev1.getColumn3(&pLines[12]);
		prev0.getColumn3(&pLines[15]);
		prev1.getColumn3(&pLines[18]);
		next1.getColumn3(&pLines[21]);
		next1.getColumn3(&pLines[24]);
		next0.getColumn3(&pLines[27]);
		next0.getColumn3(&pLines[30]);
		prev1.getColumn3(&pLines[33]);
		pLines += 36;
		
		prev0 = next0;
		prev1 = next1;
	}
}

//------------------------------------------------------------------------------

void BoundsRenderer::renderWire(const golem::BoundingPlane& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	
	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	::glBegin(GL_LINES);
	const ::GLfloat delta = (GLfloat)bounds.getGridDelta()*bounds.getGridSize();
	for (I32 i = -I32(bounds.getGridSize()); i <= I32(bounds.getGridSize()); ++i) {
		const ::GLfloat c = (GLfloat)bounds.getGridDelta()*i;
		::glVertex3f(c, +delta, 0.0f);
		::glVertex3f(c, -delta, 0.0f);
		::glVertex3f(+delta, c, 0.0f);
		::glVertex3f(-delta, c, 0.0f);
	}
	::glEnd();

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingPlane& bounds) {
	BoundsRenderer::renderWire(bounds);
}

void BoundsRenderer::renderWire(const golem::BoundingSphere& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Real r = bounds.getRadius();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));

	::glScaled(r, r, r);
	::glutWireSphere(GLdouble(1.0), bounds.getNumOfSlices(), bounds.getNumOfStacks());

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingSphere& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Real r = bounds.getRadius();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));

	::glScaled(r, r, r);
	::glutSolidSphere(GLdouble(1.0), bounds.getNumOfSlices(), bounds.getNumOfStacks());

	::glPopMatrix();
}

void BoundsRenderer::renderWire(const golem::BoundingCylinder& bounds, bool useITriangleMesh) {
	if (useITriangleMesh) {
		renderWire((const golem::ITriangleMesh&)bounds);
		return;
	}

	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	
	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	Real r = bounds.getRadius();
	Real l = bounds.getLength();
	::glScaled(r, r, l);
	
	glDrawLines(&cylinderLines.front(), (GLsizei)cylinderLines.size()/6);

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingCylinder& bounds, bool useITriangleMesh) {
	if (useITriangleMesh) {
		renderSolid((const golem::ITriangleMesh&)bounds);
		return;
	}

	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	
	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));

	Real r = bounds.getRadius();
	Real l = bounds.getLength();
	::glScaled(r, r, l);

	glDrawTriangles(&cylinderTriangles.front(), &cylinderNormals.front(), (GLsizei)cylinderTriangles.size()/9);

	::glPopMatrix();
}

void BoundsRenderer::renderWire(const golem::BoundingBox& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Vec3 d = bounds.getDimensions();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	::glScaled(d.v1, d.v2, d.v3);
	::glutWireCube(GLdouble(2.0));
	
	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::BoundingBox& bounds) {
	Mat34 pose;
	pose.multiply(this->mat, bounds.getPose());
	Vec3 d = bounds.getDimensions();

	::glPushMatrix();
	setMat(glMat, pose);
	::glMultMatrixf(&(glMat[0]));
	
	::glScaled(d.v1, d.v2, d.v3);
	::glutSolidCube(GLdouble(2.0));
	
	::glPopMatrix();
}

void BoundsRenderer::renderWire(const golem::BoundingConvexMesh& bounds) {
	renderWire((const golem::ITriangleMesh&)bounds);
}

void BoundsRenderer::renderSolid(const golem::BoundingConvexMesh& bounds) {
	renderSolid((const golem::ITriangleMesh&)bounds);
}

void BoundsRenderer::renderWire(const golem::ITriangleMesh& mesh) {
	::glPushMatrix();
	setMat(glMat, this->mat);
	::glMultMatrixf(&(glMat[0]));
	
	const U32 numOfTriangles = (U32)mesh.getTriangles().size();
	verticesData.resize(6*3*numOfTriangles);
	
	const Vec3* pVertices = &mesh.getVertices().front();
	const Triangle* pTriangles = &mesh.getTriangles().front();
	::GLfloat *pVerticesData = &verticesData.front();
	for (U32 i = 0; i < numOfTriangles; ++i) {
		const Vec3 &p0 = pVertices[pTriangles->t1];
		const Vec3 &p1 = pVertices[pTriangles->t2];
		const Vec3 &p2 = pVertices[pTriangles->t3];
		
		p0.getColumn3(&pVerticesData[0]);
		p1.getColumn3(&pVerticesData[3]);
		p1.getColumn3(&pVerticesData[6]);
		p2.getColumn3(&pVerticesData[9]);
		p2.getColumn3(&pVerticesData[12]);
		p0.getColumn3(&pVerticesData[15]);
		
		pTriangles += 1;
		pVerticesData += 6*3;
	}
	
	glDrawLines(&verticesData.front(), 3*numOfTriangles);

	::glPopMatrix();
}

void BoundsRenderer::renderSolid(const golem::ITriangleMesh& mesh) {
	::glPushMatrix();
	setMat(glMat, this->mat);
	::glMultMatrixf(&(glMat[0]));
	
	const U32 numOfTriangles = (U32)mesh.getTriangles().size();
	verticesData.resize(3*3*numOfTriangles);
	normalsData.resize(3*3*numOfTriangles);
	
	const Vec3* pVertices = &mesh.getVertices().front();
	const Vec3* pNormals = &mesh.getNormals().front();
	const Triangle* pTriangles = &mesh.getTriangles().front();
	::GLfloat *pVerticesData = &verticesData.front();
	::GLfloat *pNormalsData = &normalsData.front();
	for (U32 i = 0; i < numOfTriangles; ++i) {
		const Vec3 &p0 = pVertices[pTriangles->t1];
		const Vec3 &p1 = pVertices[pTriangles->t2];
		const Vec3 &p2 = pVertices[pTriangles->t3];
		
		p0.getColumn3(&pVerticesData[0]);
		p1.getColumn3(&pVerticesData[3]);
		p2.getColumn3(&pVerticesData[6]);
		
		Vec3 n = pNormals[i];
		n.setNegative();
		
		n.getColumn3(&pNormalsData[0]);
		n.getColumn3(&pNormalsData[3]);
		n.getColumn3(&pNormalsData[6]);
		
		pTriangles += 1;
		pVerticesData += 3*3;
		pNormalsData += 3*3;
	}
	
	glDrawTriangles(&verticesData.front(), &normalsData.front(), numOfTriangles);

	::glPopMatrix();
}

//------------------------------------------------------------------------------

void BoundsRenderer::renderWire(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	case Bounds::TYPE_PLANE:
		renderWire(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case Bounds::TYPE_SPHERE:
		renderWire(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		renderWire(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		renderWire(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		renderWire(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

void BoundsRenderer::renderSolid(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	case Bounds::TYPE_PLANE:
		renderSolid(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case Bounds::TYPE_SPHERE:
		renderSolid(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		renderSolid(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		renderSolid(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		renderSolid(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

void BoundsRenderer::renderShadow(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	//case Bounds::TYPE_PLANE:
	//	renderSolid(dynamic_cast<const BoundingPlane&>(bounds));
	//	break;
	case Bounds::TYPE_SPHERE:
		renderSolid(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		renderSolid(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		renderSolid(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		renderSolid(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

//------------------------------------------------------------------------------

DebugRenderer::DebugRenderer() {
	glLineWidth = ::GLfloat(1.0f);
	glPointSize = ::GLfloat(1.0f);
}

void DebugRenderer::render(const Real *pData, ::GLenum mode, ::GLsizei count) const {
	::glDisable(GL_LIGHTING);
	::glEnableClientState(GL_VERTEX_ARRAY);
#ifdef REAL_F32
	::glVertexPointer(3, GL_FLOAT, 4*sizeof(::GLfloat), pData);
#else // REAL_F64
	::glVertexPointer(3, GL_DOUBLE, 4*sizeof(::GLdouble), pData);
#endif
	::glEnableClientState(GL_COLOR_ARRAY);
#ifdef REAL_F32
	::glColorPointer(4, GL_UNSIGNED_BYTE, 4*sizeof(::GLfloat), pData + 3);
#else // REAL_F64
	::glColorPointer(4, GL_UNSIGNED_BYTE, 4*sizeof(::GLdouble), pData + 3);
#endif
	::glDrawArrays(mode, (::GLint)0, count/4);
	::glDisableClientState(GL_COLOR_ARRAY);
	::glDisableClientState(GL_VERTEX_ARRAY);
	::glEnable(GL_LIGHTING);
}

/** Renders the object */
void DebugRenderer::render() const {
	renderPoints();
	renderLines();
	renderTriangles();
}

/** Resets render buffers */
void DebugRenderer::reset() {
	resizePoints();
	resizeLines();
	resizeTriangles();
}

/** Renders points */
void DebugRenderer::renderPoints() const {
	const ::GLsizei count = (::GLsizei)pointData.size();
	if (count > 0) {
		::GLfloat size;
		::glGetFloatv(GL_POINT_SIZE, &size);
		::glPointSize(glPointSize);
		render((const Real*)&pointData.front(), GL_POINTS, count);
		::glPointSize(size);
	}
}

/** Renders lines */
void DebugRenderer::renderLines() const {
	const ::GLsizei count = (::GLsizei)lineData.size();
	if (count > 0) {
		::GLfloat width;
		::glGetFloatv(GL_LINE_WIDTH, &width);
		::glLineWidth(glLineWidth);
		render((const Real*)&lineData.front(), GL_LINES, count);
		::glLineWidth(width);
	}
}

/** Renders triangles */
void DebugRenderer::renderTriangles() const {
	const ::GLsizei count = (::GLsizei)triangleData.size();
	if (count > 0)
		render((const Real*)&triangleData.front(), GL_TRIANGLES, count);
}

//------------------------------------------------------------------------------

void DebugRenderer::addAxis3D(U32 type, const Mat34 &pose, Real size, const RGBA& colour, Real width, Real thick, U32 segments, Real angle) {
	const Real tang = Math::tan(angle);
	const Real wlen = width*size;

	Vec3 axis[3] = {Vec3::zero(), Vec3::zero(), Vec3::zero()};
	axis[0][(type + 0)%3] = REAL_ONE;
	axis[1][(type + 1)%3] = tang*wlen;
	axis[2][(type + 2)%3] = tang*wlen;
	pose.R.multiply(axis[0], axis[0]);
	pose.R.multiply(axis[1], axis[1]);
	pose.R.multiply(axis[2], axis[2]);

	Vec3 x[3];
	x[0].multiply(size, axis[0]);
	x[0].add(x[0], pose.p);
	x[1].multiply(size - wlen, axis[0]);
	x[1].add(x[1], pose.p);
	x[2].multiply(thick*wlen, axis[0]);
	x[2].add(x[2], pose.p);

	Vec3 r;
	r.linear(REAL_ONE, axis[1], REAL_ZERO, axis[2]);
	Vec3 beg[2], mid[2], end[2];
	beg[0].multiplyAdd(thick, r, x[2]);
	mid[0].multiplyAdd(thick, r, x[1]);
	end[0].add(r, x[1]);
	for (U32 i = 1; i <= segments; ++i) {
		const Real a = i*REAL_2_PI/segments;
		r.linear(Math::cos(a), axis[1], Math::sin(a), axis[2]);
		beg[1].multiplyAdd(thick, r, x[2]);
		mid[1].multiplyAdd(thick, r, x[1]);
		end[1].add(r, x[1]);

		addTriangle(pose.p, beg[1], beg[0], colour);
		addTriangle(mid[0], beg[0], beg[1], colour);
		addTriangle(beg[1], mid[1], mid[0], colour);
		addTriangle(mid[0], mid[1], end[0], colour);
		addTriangle(mid[1], end[1], end[0], colour);
		addTriangle(x[0], end[0], end[1], colour);

		beg[0] = beg[1];
		mid[0] = mid[1];
		end[0] = end[1];
	}
}

void DebugRenderer::addAxis3D(const Vec3 &p0, const Vec3 &p1, const RGBA& colour, Real width, Real thick, U32 segments, Real angle) {
	Vec3 x;
	x.subtract(p1, p0);
	const Real size = x.normalise();
	Vec3 y, z;
	y.cross(Vec3::axisZ(), x);
	if (y.normalise() < REAL_EPS) {
		y.cross(Vec3::axisY(), x);
		y.normalise();
	}
	z.cross(x, y);
	Mat33 rot;
	rot.fromAxes(x, y, z);
	Mat34 pose(rot, p0);
	addAxis3D(0, pose, size, colour, width, thick, segments, angle);
}

//------------------------------------------------------------------------------

void DebugRenderer::addWire(const golem::BoundingPlane& bounds) {
}

void DebugRenderer::addSolid(const golem::BoundingPlane& bounds) {
	DebugRenderer::addWire(bounds);
}

void DebugRenderer::addWire(const golem::BoundingSphere& bounds) {
}

void DebugRenderer::addSolid(const golem::BoundingSphere& bounds) {
}

void DebugRenderer::addWire(const golem::BoundingCylinder& bounds) {
}

void DebugRenderer::addSolid(const golem::BoundingCylinder& bounds) {
}

void DebugRenderer::addWire(const golem::BoundingBox& bounds) {
	const U32 lines = 12;

	U32 size = sizeLines() + lines;
	if (capacityLines() < size)
		reserveLines(size);

	const Vec3* v = bounds.getVertices();
	addLine(v[0], v[1]);
	addLine(v[1], v[3]);
	addLine(v[3], v[2]);
	addLine(v[2], v[0]);
	addLine(v[4], v[5]);
	addLine(v[5], v[7]);
	addLine(v[7], v[6]);
	addLine(v[6], v[4]);
	addLine(v[0], v[4]);
	addLine(v[1], v[5]);
	addLine(v[3], v[7]);
	addLine(v[2], v[6]);
}

void DebugRenderer::addSolid(const golem::BoundingBox& bounds) {
	addSolid(bounds.getVertices(), 8, bounds.getTriangles(), 12);
}

void DebugRenderer::addWire(const golem::BoundingConvexMesh& bounds) {
	addWire((const golem::ITriangleMesh&)bounds);
}

void DebugRenderer::addSolid(const golem::BoundingConvexMesh& bounds) {
	addSolid((const golem::ITriangleMesh&)bounds);
}

void DebugRenderer::addWire(const golem::ITriangleMesh& mesh) {
	addWire(&mesh.getVertices().front(), (U32)mesh.getVertices().size(), &mesh.getTriangles().front(), (U32)mesh.getTriangles().size());
}

void DebugRenderer::addSolid(const golem::ITriangleMesh& mesh) {
	addSolid(&mesh.getVertices().front(), (U32)mesh.getVertices().size(), &mesh.getTriangles().front(), (U32)mesh.getTriangles().size());
}

void DebugRenderer::addWire(const golem::Vec3* vertices, golem::U32 verticesNum, const golem::Triangle* triangles, golem::U32 trianglesNum) {
	const U32 lines = 3*trianglesNum;
	
	U32 size = sizeLines() + lines;
	if (capacityLines() < size)
		reserveLines(size);

	for (U32 i = 0; i < trianglesNum; ++i) {
		const Vec3 &p0 = vertices[triangles->t1];
		const Vec3 &p1 = vertices[triangles->t2];
		const Vec3 &p2 = vertices[triangles->t3];
		
		addLine(p0, p1);
		addLine(p1, p2);
		addLine(p2, p0);
		
		triangles += 1;
	}
}

void DebugRenderer::addSolid(const golem::Vec3* vertices, golem::U32 verticesNum, const golem::Triangle* triangles, golem::U32 trianglesNum) {
	U32 size = sizeTriangles() + trianglesNum;
	if (capacityTriangles() < size)
		reserveTriangles(size);

	for (U32 i = 0; i < trianglesNum; ++i) {
		const Vec3 &p0 = vertices[triangles->t1];
		const Vec3 &p1 = vertices[triangles->t2];
		const Vec3 &p2 = vertices[triangles->t3];
		
		addTriangle(p1, p0, p2);
		
		triangles += 1;
	}
}

void DebugRenderer::addWire(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	case Bounds::TYPE_PLANE:
		addWire(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case Bounds::TYPE_SPHERE:
		addWire(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		addWire(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		addWire(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		addWire(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

void DebugRenderer::addSolid(const golem::Bounds& bounds) {
	switch (bounds.getType()) {
	case Bounds::TYPE_PLANE:
		addSolid(dynamic_cast<const BoundingPlane&>(bounds));
		break;
	case Bounds::TYPE_SPHERE:
		addSolid(dynamic_cast<const BoundingSphere&>(bounds));
		break;
	case Bounds::TYPE_CYLINDER:
		addSolid(dynamic_cast<const BoundingCylinder&>(bounds));
		break;
	case Bounds::TYPE_BOX:
		addSolid(dynamic_cast<const BoundingBox&>(bounds));
		break;
	case Bounds::TYPE_CONVEX_MESH:
		addSolid(dynamic_cast<const BoundingConvexMesh&>(bounds));
		break;
	default:
		break;
	}
}

//------------------------------------------------------------------------------

void golem::XMLData(golem::RGBA &val, XMLContext* context, bool create) {
	ASSERT(context)

		U32 rgba[4] = { val._rgba.r, val._rgba.g, val._rgba.b, val._rgba.a };
	XMLData("R", rgba[0], context, create);
	XMLData("G", rgba[1], context, create);
	XMLData("B", rgba[2], context, create);
	XMLData("A", rgba[3], context, create);
	val._rgba.r = (U8)rgba[0];
	val._rgba.g = (U8)rgba[1];
	val._rgba.b = (U8)rgba[2];
	val._rgba.a = (U8)rgba[3];
}

void golem::XMLData(Appearance &val, XMLContext* context, bool create) {
	ASSERT(context)

		if (!create)
			val.setToDefault();

	try {
		XMLData("invisible", val.invisible, context);
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
	try {
		XMLData(val.solidColour, context->getContextFirst("solid_colour"));
	}
	catch (const MsgXMLParserNameNotFound&) {}
	try {
		XMLData(val.wireColour, context->getContextFirst("wire_colour"));
	}
	catch (const MsgXMLParserNameNotFound&) {}
	try {
		XMLData(val.shadowColour, context->getContextFirst("shadow_colour"));
	}
	catch (const MsgXMLParserNameNotFound&) {}
	try {
		XMLData("line_width", val.lineWidth, context);
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
}

//------------------------------------------------------------------------------

void golem::XMLData(OpenGL& desc, XMLContext* context, bool create) {
	try {
		XMLData("view_name", desc.viewName, context, create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
	XMLData(desc.viewDir, context->getContextFirst("view_dir"), create);
	XMLData(desc.viewPoint, context->getContextFirst("view_point"), create);
	try {
		XMLData(desc.viewUp, context->getContextFirst("view_up"), create);
	}
	catch (const MsgXMLParserNameNotFound&) {}
	try {
		XMLData("view_inc", desc.viewInc, context, create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {}

	XMLData(desc.clearColor, context->getContextFirst("clear_color"), create);

	XMLDataFlag<OpenGL::DRAW_DEFAULT_SOLID>(desc.draw, "solid", context->getContextFirst("draw"), create);
	XMLDataFlag<OpenGL::DRAW_DEFAULT_WIRE>(desc.draw, "wire", context->getContextFirst("draw"), create);
	XMLDataFlag<OpenGL::DRAW_DEFAULT_SHADOW>(desc.draw, "shadow", context->getContextFirst("draw"), create);
	XMLDataFlag<OpenGL::DRAW_DEBUG_SIMULATION>(desc.draw, "simulation", context->getContextFirst("draw"), create);
	XMLDataFlag<OpenGL::DRAW_DEBUG_NORMALS>(desc.draw, "normals", context->getContextFirst("draw"), create);

	try {
		XMLData("normal_len", desc.drawNormalLen, context->getContextFirst("draw"), create);
	}
	catch (const MsgXMLParserAttributeNotFound&) {}
}

