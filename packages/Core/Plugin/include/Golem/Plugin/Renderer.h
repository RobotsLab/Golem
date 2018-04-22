/** @file Renderer.h
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

#pragma once
#ifndef _GOLEM_PLUGIN_RENDERER_H_
#define _GOLEM_PLUGIN_RENDERER_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/System.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Plugin/UI.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Bounds Renderer */
class BoundsRenderer {
public:
	static const RGBA glWireColourDflt;
	static const RGBA glSolidColourDflt;
	static const RGBA glShadowColourDflt;
	static const Real glLineWidthDflt;

	static const ::GLfloat glMatDflt[16];
	static const ::GLfloat glShadowMatDflt[16];

	static const U32 BOUNDING_CYLINDER_SLICES = 16;

protected:
	typedef std::vector<GLfloat> Data;

	static Data cylinderTriangles;
	static Data cylinderNormals;
	static Data cylinderLines;

	::GLfloat glMat[16], glShadowMat[16];
	::GLfloat glLineWidth;
	RGBA glWireColour, glSolidColour, glShadowColour;
	Data verticesData, normalsData;
	Mat34 mat;

	void initCylinder();

	void glDrawTriangles(const void* vertices, const void* normals, GLsizei numOfTriangles) {
		::glEnableClientState(GL_VERTEX_ARRAY);
		::glVertexPointer(3, GL_FLOAT, 0, vertices);
		::glEnableClientState(GL_NORMAL_ARRAY);
		::glNormalPointer(GL_FLOAT, 0, normals);

		::glDrawArrays(GL_TRIANGLES, 0, 3*numOfTriangles);

		::glDisableClientState(GL_VERTEX_ARRAY);
		::glDisableClientState(GL_NORMAL_ARRAY);
	}
	
	void glDrawLines(const void* vertices, GLsizei numOfLines) {
		::glEnableClientState(GL_VERTEX_ARRAY);
		::glVertexPointer(3, GL_FLOAT, 0, vertices);

		::glDrawArrays(GL_LINES, 0, 2*numOfLines);

		::glDisableClientState(GL_VERTEX_ARRAY);
	}
	
	void setMat(::GLfloat data [], const Mat34& mat) {
		mat.R.getColumn44(&(data[0]));
		mat.p.getColumn3(&(data[12]));

		data[3] = data[7] = data[11] = ::GLfloat(0.0);
		data[15] = ::GLfloat(1.0);
	}

public:
	BoundsRenderer();
	
	void reset();
	
	void setMat(const Mat34& mat) {
		this->mat = mat;
		//setMat(glMat, mat);
	}

	void setShadowMat(const Mat34& mat) {
		setMat(glShadowMat, mat);
	}

	void setLineWidth(Real width) {
		this->glLineWidth = (GLfloat)width;
	}

	void setWireColour(const RGBA& colour) {
		this->glWireColour = colour;
	}

	void setSolidColour(const RGBA& colour) {
		this->glSolidColour = colour;
	}

	void setShadowColour(const RGBA& colour) {
		this->glShadowColour = colour;
	}

	
	void renderWire(const golem::BoundingPlane& bounds);

	void renderSolid(const golem::BoundingPlane& bounds);

	void renderWire(const golem::BoundingSphere& bounds);

	void renderSolid(const golem::BoundingSphere& bounds);

	void renderWire(const golem::BoundingCylinder& bounds, bool useITriangleMesh = false);

	void renderSolid(const golem::BoundingCylinder& bounds, bool useITriangleMesh = false);

	void renderWire(const golem::BoundingBox& bounds);

	void renderSolid(const golem::BoundingBox& bounds);

	void renderWire(const golem::BoundingConvexMesh& bounds);

	void renderSolid(const golem::BoundingConvexMesh& bounds);


	void renderWire(const golem::ITriangleMesh& mesh);

	void renderSolid(const golem::ITriangleMesh& mesh);


	void renderWire(const golem::Bounds& bounds);

	template <typename const_iterator> void renderWire(const_iterator begin, const_iterator end) {
		const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING);
		if (glLighting) ::glDisable(GL_LIGHTING);
		::GLfloat width;
		::glGetFloatv(GL_LINE_WIDTH, &width);

		::glColor4ub(glWireColour._rgba.r, glWireColour._rgba.g, glWireColour._rgba.b, glWireColour._rgba.a);
		::glLineWidth(glLineWidth);
		for (const_iterator i = begin; i != end; ++i)
			renderWire(**i);

		::glLineWidth(width);
		if (glLighting) ::glEnable(GL_LIGHTING);
	}

	void renderSolid(const golem::Bounds& bounds);

	template <typename const_iterator> void renderSolid(const_iterator begin, const_iterator end) {
		const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING);
		if (!glLighting) ::glEnable(GL_LIGHTING);
		
		::glColor4ub(glSolidColour._rgba.r, glSolidColour._rgba.g, glSolidColour._rgba.b, glSolidColour._rgba.a);
		for (const_iterator i = begin; i != end; ++i)
			renderSolid(**i);

		if (!glLighting) ::glDisable(GL_LIGHTING);
	}

	void renderShadow(const golem::Bounds& bounds);
	
	template <typename const_iterator> void renderShadow(const_iterator begin, const_iterator end) {
		const GLboolean glLighting = ::glIsEnabled(GL_LIGHTING);
		if (glLighting) ::glDisable(GL_LIGHTING);
		
		::glPushMatrix();
		::glMultMatrixf(glShadowMat);

		::glColor4ub(glShadowColour._rgba.r, glShadowColour._rgba.g, glShadowColour._rgba.b, glShadowColour._rgba.a);
		for (const_iterator i = begin; i != end; ++i)
			renderShadow(**i);
		
		::glPopMatrix();

		if (glLighting) ::glEnable(GL_LIGHTING);
	}
};

//------------------------------------------------------------------------------

/** Debug buffered renderer */
class DebugRenderer : public UIRenderer {
protected:
	typedef std::vector<Real> Data;

	::GLfloat glLineWidth, glPointSize;
	RGBA colour;
	Data pointData, lineData, triangleData;

	void render(const Real *pData, ::GLenum mode, ::GLsizei count) const;
	
	inline void addData(Data& data, const Vec3& vec3, const RGBA& colour) {
		data.push_back(vec3.v1);
		data.push_back(vec3.v2);
		data.push_back(vec3.v3);
		data.push_back(*(Real*)&colour);
	}
	
public:
	DebugRenderer();
	virtual ~DebugRenderer() {}

	/** golem::UIRenderer: Renders the object */
	virtual void render() const;

	/** golem::UIRenderer: Renders the object */
	virtual void customRender() const {}

	/** Resets render buffers */
	virtual void reset();

	/** Adds data from DebugRenderer */
	inline void add(const DebugRenderer &dr) {
		pointData.insert(pointData.end(), dr.pointData.begin(), dr.pointData.end());
		lineData.insert(lineData.end(), dr.lineData.begin(), dr.lineData.end());
		triangleData.insert(triangleData.end(), dr.triangleData.begin(), dr.triangleData.end());
	}

	/** Current colour */
	inline RGBA getColour() const {
		return colour;
	}
	/** Sets current colour */
	inline void setColour(const RGBA &colour) {
		this->colour = colour;
	}
	/** Line width */
	Real getLineWidth() const {
		return (Real)glLineWidth;
	}
	/** Sets line width */
	void setLineWidth(Real width) {
		this->glLineWidth = (GLfloat)width;
	}
	/** Sets point size */
	Real getPointSize() const {
		return (Real)glPointSize;
	}
	/** Sets point size */
	void setPointSize(Real size) {
		this->glPointSize = (GLfloat)size;
	}
	
	/** Adds new point */
	inline void addPoint(const Vec3 &p, const RGBA &colour) {
		addData(pointData, p, colour);
	}
	/** Adds new point */
	inline void addPoint(const Vec3 &p) {
		addPoint(p, colour);
	}
	/** Resets points memory buffer */
	inline void resizePoints(U32 numOfPoints = 0) {
		pointData.resize(size_t(4*numOfPoints));
	}
	/** Returns number of points */
	inline U32 sizePoints() const {
		return U32(pointData.size()/4);
	}
	/** Reserve points memory buffer */
	inline void reservePoints(U32 numOfPoints) {
		pointData.reserve(size_t(4*numOfPoints));
	}
	/** Returns number of allocated points in the memory buffer */
	inline U32 capacityPoints() const {
		return U32(pointData.capacity()/4);
	}
	/** Renders points */
	virtual void renderPoints() const;

	/** Adds new line */
	inline void addLine(const Vec3 &p0, const Vec3 &p1, const RGBA &colour) {
		addData(lineData, p0, colour);
		addData(lineData, p1, colour);
	}
	/** Adds new line */
	inline void addLine(const Vec3 &p0, const Vec3 &p1) {
		addLine(p0, p1, colour);
	}
	/** Resets lines memory buffer */
	inline void resizeLines(U32 numOfLines = 0) {
		lineData.resize(size_t(8*numOfLines));
	}
	/** Returns number of lines */
	inline U32 sizeLines() const {
		return U32(lineData.size()/8);
	}
	/** Reserve lines memory buffer */
	inline void reserveLines(U32 numOfLines) {
		lineData.reserve(size_t(8*numOfLines));
	}
	/** Returns number of allocated lines in the memory buffer */
	inline U32 capacityLines() const {
		return U32(lineData.capacity()/8);
	}
	/** Renders lines */
	virtual void renderLines() const;

	/** Adds new triangle */
	inline void addTriangle(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const RGBA &colour) {
		addData(triangleData, p0, colour);
		addData(triangleData, p1, colour);
		addData(triangleData, p2, colour);
	}
	/** Adds new triangle */
	inline void addTriangle(const Vec3 &p0, const Vec3 &p1, const Vec3 &p2) {
		addTriangle(p0, p1, p2, colour);
	}
	/** Resets triangles memory buffer */
	inline void resizeTriangles(U32 numOfTriangles = 0) {
		triangleData.resize(size_t(12*numOfTriangles));
	}
	/** Returns number of triangles */
	inline U32 sizeTriangles() const {
		return U32(triangleData.size()/12);
	}
	/** Reserve triangles memory buffer */
	inline void reserveTriangles(U32 numOfTriangles) {
		triangleData.reserve(size_t(12*numOfTriangles));
	}
	/** Returns number of allocated triangles in the memory buffer */
	inline U32 capacityTriangles() const {
		return U32(triangleData.capacity()/12);
	}
	/** Renders triangles */
	virtual void renderTriangles() const;

	/** Adds coordinate frame axes */
	inline void addAxes(const Mat34 &pose, const Vec3 &size, const RGBA &colourX, const RGBA &colourY, const RGBA &colourZ) {
		Vec3 axis;
		
		axis.set(size.v1, Real(0.0), Real(0.0)); // X axis
		pose.multiply(axis, axis);
		addLine(pose.p, axis, colourX);
		
		axis.set(Real(0.0), size.v2, Real(0.0)); // Y axis
		pose.multiply(axis, axis);
		addLine(pose.p, axis, colourY);
		
		axis.set(Real(0.0), Real(0.0), size.v3); // Z axis
		pose.multiply(axis, axis);
		addLine(pose.p, axis, colourZ);
	}
	/** Adds coordinate frame axes */
	inline void addAxes(const Mat34 &pose, const Vec3 &size) {
		addAxes(pose, size, RGBA::RED, RGBA::GREEN, RGBA::BLUE);
	}

	/** Incrementally reserves memory buffer for the specified number of axes */
	inline void reserveAxesInc(U32 numOfAxes) {
		const U32 size = sizeLines() + 3*numOfAxes;
		if (size > capacityLines())
			reserveLines(size);
	}

	/** Adds 3D axis from pose */
	void addAxis3D(U32 type, const Mat34 &pose, Real size, const RGBA& colour, Real width = 0.1, Real thick = 0.25, U32 segments = 8, Real angle = REAL_PI/6.0);

	/** Adds 3D axis from line */
	void addAxis3D(const Vec3 &p0, const Vec3 &p1, const RGBA& colour, Real width = 0.1, Real thick = 0.25, U32 segments = 8, Real angle = REAL_PI/6.0);

	/** Adds coordinate frame 3D axes */
	inline void addAxes3D(const Mat34 &pose, const Vec3 &size, const RGBA &colourX, const RGBA &colourY, const RGBA &colourZ, Real width = 0.1, Real thick = 0.25, U32 segments = 8, Real angle = REAL_PI/6.0) {
		addAxis3D(0, pose, size.x, colourX, width, thick, segments, angle);
		addAxis3D(1, pose, size.y, colourY, width, thick, segments, angle);
		addAxis3D(2, pose, size.z, colourZ, width, thick, segments, angle);
	}

	/** Adds coordinate frame 3D axes */
	inline void addAxes3D(const Mat34 &pose, const Vec3 &size, const U8 a = 127) {
		addAxes3D(pose, size, RGBA(RGBA::RED._rgba.r, RGBA::RED._rgba.g, RGBA::RED._rgba.b, a), RGBA(RGBA::GREEN._rgba.r, RGBA::GREEN._rgba.g, RGBA::GREEN._rgba.b, a), RGBA(RGBA::BLUE._rgba.r, RGBA::BLUE._rgba.g, RGBA::BLUE._rgba.b, a));
	}

	void addWire(const golem::BoundingPlane& bounds);

	void addSolid(const golem::BoundingPlane& bounds);

	void addWire(const golem::BoundingSphere& bounds);

	void addSolid(const golem::BoundingSphere& bounds);

	void addWire(const golem::BoundingCylinder& bounds);

	void addSolid(const golem::BoundingCylinder& bounds);

	void addWire(const golem::BoundingBox& bounds);

	void addSolid(const golem::BoundingBox& bounds);

	void addWire(const golem::BoundingConvexMesh& bounds);

	void addSolid(const golem::BoundingConvexMesh& bounds);


	void addWire(const golem::ITriangleMesh& mesh);

	void addSolid(const golem::ITriangleMesh& mesh);


	void addWire(const golem::Vec3* vertices, golem::U32 verticesNum, const golem::Triangle* triangles, golem::U32 trianglesNum);

	void addSolid(const golem::Vec3* vertices, golem::U32 verticesNum, const golem::Triangle* triangles, golem::U32 trianglesNum);


	void addWire(const golem::Bounds& bounds);

	template <typename const_iterator> void addWire(const_iterator begin, const_iterator end) {
		for (const_iterator i = begin; i != end; ++i)
			addWire(**i);
	}

	void addSolid(const golem::Bounds& bounds);

	template <typename const_iterator> void addSolid(const_iterator begin, const_iterator end) {
		for (const_iterator i = begin; i != end; ++i)
			addSolid(**i);
	}
};

//------------------------------------------------------------------------------

/** Appearance */
class Appearance {
public:
	typedef std::vector<Appearance> Seq;

	/** Invisible object */
	bool invisible;
	/** Surface colour */
	RGBA solidColour;
	/** Edge colour */
	RGBA wireColour;
	/** Shadow colour */
	RGBA shadowColour;
	/** Line width */
	Real lineWidth;

	/** Appearance construction */
	Appearance() {
		Appearance::setToDefault();
	}
	/** Sets the appearance to the default state */
	void setToDefault() {
		invisible = false;
		solidColour = BoundsRenderer::glSolidColourDflt;
		wireColour = BoundsRenderer::glWireColourDflt;
		shadowColour = BoundsRenderer::glShadowColourDflt;
		lineWidth = BoundsRenderer::glLineWidthDflt;
	}
	/** Checks if the appearance is valid. */
	bool isValid() const {
		if (lineWidth < Real(1.0))
			return false;

		return true;
	}
};

//------------------------------------------------------------------------------

class XMLContext;

/** Reads/writes RGBA from/to a given context */
void XMLData(RGBA &val, XMLContext* context, bool create = false);
void XMLData(Appearance &val, XMLContext* context, bool create = false);
void XMLData(OpenGL& desc, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLUGIN_RENDERER_H_*/
