/** @file Chain.h
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
#ifndef _GOLEM_CTRL_CHAIN_H_
#define _GOLEM_CTRL_CHAIN_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Joint.h>
#include <Golem/Math/Queue.h>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgChain, Message)
MESSAGE_DEF(MsgChainInvalidDesc, MsgChain)

//------------------------------------------------------------------------------

class Controller;

/** Open-chain manipulator interface. */
class Chain {
public:
	typedef shared_ptr<Chain> Ptr;
	typedef std::vector<Ptr> Seq;
	typedef std::vector<Chain*> PtrSeq;
	typedef Chainspace::Coord<Chain*> SpacePtrSeq;
	
	/** Chain description */
	class Desc {
		friend class Chain;
		friend class Controller;

	protected:
		/** Creates Chain given the description object. 
		* @param controller	Controller object
		* @return			pointer to the Chain interface if no errors have occured, throws otherwise
		*/
		CREATE_FROM_OBJECT_DESC_1(Chain, Chain::Ptr, Controller&)

		/** Chain index */
		Chainspace::Index index;
		/** Chain index in the controller */
		idx_t indexLocal;

	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;
		typedef std::vector<const Desc*> PtrSeq;
		
		/** Name ASCII string */
		std::string name;
		/** Joints */
		Joint::Desc::Seq joints;
		/** Linked chain index */
		U32 linkedChainIndex;
		/** Chain local pose */
		Mat34 localPose;
		/** Local reference pose in the tool frame */
		Mat34 referencePose;
		/** Use specialised (overloaded) forward/inverse/jacobian functions */
		bool customKinematics;
		
		/** bounds and visualisation of the joint in local coordinates */
		Bounds::Desc::Seq bounds;

		Desc() {
			Desc::setToDefault();
		}
		
		virtual ~Desc() {
		}
		
		virtual void setToDefault() {
			name = "";
			joints.clear();	
			linkedChainIndex = -1;
			localPose.setId();
			referencePose.setId();
			customKinematics = false;
		}

		virtual bool isValid() const {
			if (joints.size() <= 0 || joints.size() > Configspace::DIM)
				return false;

			for (Joint::Desc::Seq::const_iterator i = joints.begin(); i != joints.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;
			
			if (!localPose.isFinite() || !referencePose.isFinite())
				return false;
			
			// there can be no bounds defined, but if they are - they have to be valid
			for (Bounds::Desc::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;

			return true;
		}
	};

private:
	/** Exponential coordinates of the joint */
	class JointCoord {
	public:
		typedef std::vector<JointCoord> Seq;

		/** exponential coordinates parameterises frame transformation by screw motion */
		ExpCoord trn;
		/** joint index */
		idx_t index;

		JointCoord(const ExpCoord& trn, idx_t index) : trn(trn), index(index) {}
	};

	/** Name ASCII string */
	std::string name;
	/** Chain index */
	Chainspace::Index index;
	/** Chain index in the controller */
	idx_t indexLocal;
	/** Linked chain index */
	Chainspace::Index linkedChainIndex;

	/** Exponential coordinates of the joint */
	JointCoord::Seq jointCoordSeq;

	/** Joints collection */
	Joint::Seq jointSeq;
	/** Joints pointers */
	Joint::PtrSeq joints;
	/** Chain local pose */
	Mat34 localPose;
	/** Tool reference pose */
	Mat34 referencePose;

protected:
	/** Controller is the owner. */
	golem::Controller& controller;
	/** golem::Context object. */
	golem::Context& context;

	/** Use specialised (overloaded) forward/inverse/jacobian functions */
	bool customKinematics;

		/** bounds and visualisation of the joint */
	Bounds::Desc::Seq boundsDescSeq;

	/** Chain data members critical section */
	mutable CriticalSection csData;
	
	/** Creates Chain from the description. 
	* @param desc	Chain description
	*/
	void create(const Desc& desc);

	Chain(Controller& controller);

public:
	/** Each derived class should have virtual destructor releasing resources
	*	to avoid calling virtual functions of non-existing objects
	*/
	virtual ~Chain();

	/** Forward transformation for tool frame
	 * @param cc	configuration space coordinates
	 * @param trn	SE(3) transformation matrix:
	 *				tool frame pose -> base frame pose (includes the chain global pose)
	 */
	virtual void chainForwardTransform(const Real* cc, Mat34& trn) const;

	/** (Extended) forward transformation for all joints
	 * @param cc	configuration space coordinates
	 * @param trn	array of SE(3) transformation matrices for each joint:
	 *				joint frame poses -> base frame pose (includes the chain global pose)
	 */
	virtual void jointForwardTransform(const Real* cc, Mat34* trn) const;

	/** End-effector spatial velocity
	 * @param cc	configuration space coordinates
	 * @param dcc	delta
	 * @param vs	velocity
	 */
	virtual void velocitySpatial(const Real* cc, const Real* dcc, Twist& vs) const;

	/** End-effector body velocity
	 * @param cc	configuration space coordinates
	 * @param dcc	delta
	 * @param vb	velocity
	 */
	virtual void velocityBody(const Real* cc, const Real* dcc, Twist& vb) const;

	/** End-effector velocity
	 * @param cc	configuration space coordinates
	 * @param dcc	delta
	 * @param v		velocity
	 */
	virtual void velocity(const Real* cc, const Real* dcc, Twist& v) const;

	/** End-effector velocity
	 * @param jac	manipulator Jacobian
	 * @param dcc	delta
	 * @param v		velocity
	 */
	virtual void velocityFromJacobian(const Twist* jac, const Real* dcc, Twist& v) const;

	/** End-effector velocity
	 * @param trn	forward transformation
	 * @param vs	spatial velocity
	 * @param v		velocity
	 */
	virtual void velocityFromSpatial(const Mat34& trn, const Twist& vs, Twist& v) const;

	/** End-effector velocity
	 * @param trn	forward transformation
	 * @param vb	body velocity
	 * @param v		velocity
	 */
	virtual void velocityFromBody(const Mat34& trn, const Twist& vb, Twist& v) const;

	/** Spatial manipulator Jacobian
	 * @param cc	configuration space coordinates
	 * @param jac	manipulator Jacobian
	 */
	virtual void jacobianSpatial(const Real* cc, Twist* jac) const;

	/** Body manipulator Jacobian
	 * @param cc	configuration space coordinates
	 * @param jac	manipulator Jacobian
	 */
	virtual void jacobianBody(const Real* cc, Twist* jac) const;

	/** Manipulator Jacobian
	 * @param cc	configuration space coordinates
	 * @param jac	manipulator Jacobian
	 */
	virtual void jacobian(const Real* cc, Twist* jac) const;

	/** Manipulator Jacobian
	 * @param jacs	spatial manipulator Jacobian
	 * @param trn	forward transformation
	 * @param jac	manipulator Jacobian
	 */
	virtual void jacobianFromSpatial(const Twist* jacs, const Mat34& trn, Twist* jac) const;

	/** Manipulator Jacobian
	 * @param jacb	body manipulator Jacobian
	 * @param trn	forward transformation
	 * @param jac	manipulator Jacobian
	 */
	virtual void jacobianFromBody(const Twist* jacb, const Mat34& trn, Twist* jac) const;

	/** Returns Name ASCII string */
	const std::string& getName() const {
		return name;
	}
	/** Chain index */
	const Chainspace::Index& getIndex() const {
		return index;
	}
	/** Chain index in the controller */
	idx_t getIndexLocal() const {
		return indexLocal;
	}
	/** Linked chain index */
	const Chainspace::Index& getLinkedChainIndex() const {
		return linkedChainIndex;
	}

	/** Access to Joints
	 * @return		reference to Joint container
	 */
	const Joint::PtrSeq& getJoints() const {
		return joints;
	}

	/** Returns Chain local pose
	 * @return				Chain local pose
	 */
	virtual Mat34 getLocalPose() const;

	/** Sets Chain local pose
	 * @param pose	Chain local pose
	 */
	virtual void setLocalPose(const Mat34 &localPose);
	
	/** Returns Tool reference pose
	 * @return				Tool reference pose
	 */
	virtual Mat34 getReferencePose() const;

	/** Sets Tool reference pose
	 * @param referencePose	Tool reference pose
	 */
	virtual void setReferencePose(const Mat34 &referencePose);

	/** Adds the bounds description in the local coordinate frame of a given Chain
	 * @param pDesc			description of the bounds
	*/
	virtual bool addBoundsDesc(Bounds::Desc::Ptr pDesc);

	/** Removes the bounds description of a given Chain
	 * @param pDesc			the bounds to be removed
	*/
	virtual bool removeBoundsDesc(const Bounds::Desc* pDesc);
	
	/** Returns reference to the collection of bounds description of the Chain 
	 * @return				reference to the collection of bounds description
	*/
	virtual Bounds::Desc::SeqPtr getBoundsDescSeq() const;

	/** Checks if there is non empty the collection of bounds description of the Chain
	* @return		<code>true</code> if bounds are available; <code>false</code> otherwise 
	*/
	virtual bool hasBoundsDesc() const;

	/** golem::Controller object */
	const golem::Controller& getController() const {
		return controller;
	}
	/** golem::Controller object */
	golem::Controller& getController() {
		return controller;
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_CHAIN_H_*/
