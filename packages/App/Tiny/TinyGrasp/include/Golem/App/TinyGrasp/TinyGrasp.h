/** @file TinyGrasp.h
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
#ifndef _GOLEM_DEMO_TINY_GRASP_H_
#define _GOLEM_DEMO_TINY_GRASP_H_

#include <Golem/Tiny/Tiny.h>
#include <Golem/Math/Quat.h>
#include <Golem/Math/Mat23.h>

//------------------------------------------------------------------------------

namespace golem {

class DebugRenderer;

namespace tiny {

//------------------------------------------------------------------------------

typedef std::vector<Vec2> Vec2Seq;
typedef std::vector<Mat22> Mat22Seq;
typedef std::vector<Mat23> Mat23Seq;

//------------------------------------------------------------------------------

class PoseError {
public:
	Real lin;
	Real ang;

	PoseError(Real lin = REAL_ZERO, Real ang = REAL_ZERO);
	PoseError(const Mat34& a, const Mat34& b);

private:
	Real getLinearDist(const Vec3& v0, const Vec3& v1);
	Real getAngularDist(const Quat& q0, const Quat& q1);	
};

//------------------------------------------------------------------------------

class GraspPose {
public:
	typedef std::vector<GraspPose> Seq;

	Mat34 approach;
	Mat34 grasp;
};

//------------------------------------------------------------------------------

class Debug;

class TinyEx: public Tiny {
private:
	Debug* debug;

public:
	TinyEx(int argc, char *argv[]);
	void render(const DebugRenderer* debugRenderer = NULL);
	XMLContext* getXMLContext();
};

//------------------------------------------------------------------------------

class TinyGrasp {
public:
	/** Creates and calibrates controller, setups robot body */
	TinyGrasp();

	/** Creates box object */
	RigidBody* createObject(const Mat34& pose, const Vec3& dimensions);
	/** Removes object */
	void removeObject(RigidBody*& object);
	
	/** Reads current arm pose */
	Mat34 read();
	/** Simulates movement to pose, returns best approximation of pose  */
	Mat34 moveTry(const Mat34& pose);
	/** Executes movement to a pose previously set by moveTry() */
	void moveExec(Real duration = Real(5.0));

	/** Sets graspable/moveable object */
	void setGraspObject(RigidBody* object);

	/** Returns a set of possible grasp poses for a given grasp object */
	GraspPose::Seq getGraspPoses() const;
	/** Returns grasp success for a given set of possible grasp poses */
	bool graspTry(const GraspPose::Seq& poses);
	/** Returns grasp success for a given grasp pose */
	bool graspTry(const GraspPose& pose);
	/** Executes movement and grasp to a grasp pose previously set by graspTry() */
	bool graspExec(Real duration = Real(5.0));
	/** Releases the grasped object - creates independent object */
	void graspRelease();

	/** Returns a set of possible robot poses for a given grasp object */
	Mat23Seq getRobotPoses(const GraspPose::Seq& poses) const;

	/** Opens gripper */
	void gripperOpen();
	/** Closes gripper */
	void gripperClose();

protected:
	// open chain manipulators have only one kinematic chain with index 0
	static const int chain = 0;

	shared_ptr<TinyEx> tiny;

	GenConfigspaceState cbegin, cend;
	GenWorkspaceChainState end;
	GraspPose gend;
	Mat34 gapproach;

	Mat34 controllerPose;
	Katana* controller;
	Shape* objectGrasped;
	RigidBody* object;

	U32 plannerTries;
	Real approachOffset;
	Real graspOffset;
	PoseError graspError;
	Real robotDistance;

	I32 sensorThreshold;
	I32 encoderThreshold;
	KatanaSensorDataSet zero;

	Mat34 getToolPose();
	void attachObject();
	void releaseObject();
	BoxShapeDesc* getObjectBoxDesc(RigidBody* object) const;

	static Mat33 rotAxis(U32 axis, Real angle);
	static Mat33 rotAxis(U32 axis1, Real angle1, U32 axis2, Real angle2);
	/** Relative transformation: ab*a = b => ab = b*a^-1 */
	static Mat34 diff(const Mat34& a, const Mat34& b);
	/** Inertial frame G to body frame H given reference pose A: H = A^-1 G A */
	static Mat34 toBody(const Mat34& A, const Mat34& G);
	/** Inertial frame G from body frame H given reference pose A: G = A H A^-1 */
	static Mat34 fromBody(const Mat34& A, const Mat34& H);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

#endif /*_GOLEM_DEMO_TINY_GRASP_H_*/
