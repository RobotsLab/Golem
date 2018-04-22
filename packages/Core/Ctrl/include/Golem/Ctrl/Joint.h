/** @file Joint.h
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
#ifndef _GOLEM_CTRL_JOINT_H_
#define _GOLEM_CTRL_JOINT_H_

//------------------------------------------------------------------------------

#include <Golem/Sys/Thread.h>
#include <Golem/Math/Bounds.h>
#include <Golem/Ctrl/Types.h>
#include <Golem/Sys/Message.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Exceptions */
MESSAGE_DEF(MsgJoint, Message)
MESSAGE_DEF(MsgJointInvalidDesc, MsgJoint)

//------------------------------------------------------------------------------

class Context;
class Chain;
class Controller;

/** Joint. */
class Joint {
public:
	typedef shared_ptr<Joint> Ptr;
	typedef std::vector<Ptr> Seq;
	typedef std::vector<Joint*> PtrSeq;
	typedef Configspace::Coord<Joint*> SpacePtrSeq;	

	typedef std::vector<ExpCoord> ExpCoordSeq;
	typedef std::vector<Mat34> Mat34Seq;

	/** Joint description */
	class Desc {
		friend class Joint;
		friend class Chain;
	
	public:
		typedef shared_ptr<Desc> Ptr;
		typedef std::vector<Ptr> Seq;
		typedef std::vector<const Desc*> PtrSeq;
		
	protected:
		/** Creates Joint given the description object. 
		* @param chain	Chain interface
		* @return		pointer to the Joint, if no errors have occured;
		*				<code>NULL</code> otherwise 
		*/
		CREATE_FROM_OBJECT_DESC_1(Joint, Joint::Ptr, Chain&)

		/** Joint index */
		Configspace::Index index;
		/** Joint index in the chain */
		idx_t indexLocal;

	public:
		/** Name ASCII string */
		std::string name;
		
 		/** minimum value of extended coordinate of a joint */
		GenCoord min;
		/** maximum value of extended coordinate of a joint */
		GenCoord max;
		/** minimum and maximum limits' offset */
		GenCoord offset;

		/** exponential coordinates defines joint frame transformation as a sequence screw motions */
		ExpCoordSeq trnSeq;
		/** initial frame transformation */
		Mat34 trnInit;

		/** bounds and visualisation of the joint in local coordinates */
		Bounds::Desc::Seq bounds;

		Desc() {
			Desc::setToDefault();
		}

		virtual ~Desc() {}

		/** Resets the description to default one */
		virtual void setToDefault() {
			name = "";

			min.set(-REAL_PI, -REAL_PI, -REAL_PI);
			max.set(+REAL_PI, +REAL_PI, +REAL_PI);
			offset.set(Real(0.001)*REAL_PI, Real(0.01)*REAL_PI, Real(0.1)*REAL_PI);

			trnSeq.resize(1);
			trnSeq[0].setToDefault();
			trnInit.setToDefault();

			bounds.clear();
		}

		/** Checks validity */
		virtual bool isValid() const {
			if (!min.isValid() || !max.isValid() || !offset.isValid())
				return false;
			if (!((min + offset) <= (max - offset)))
				return false;

			if (trnSeq.empty())
				return false;
			for (ExpCoordSeq::const_iterator i = trnSeq.begin(); i != trnSeq.end(); ++i)
				if (!i->isValid())
					return false;
			if (!trnInit.isValid())
				return false;

			// there can be no bounds defined, but if they are - they have to be valid
			for (Bounds::Desc::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
				if (*i == NULL || !(*i)->isValid())
					return false;

			return true;
		}
	};

private:
	/** Name ASCII string */
	std::string name;
	/** Joint index */
	Configspace::Index index;
	/** Joint index in the chain */
	idx_t indexLocal;

protected:
	/** Chain is the owner */
	golem::Chain& chain;
	/** Context object */
	golem::Context& context;

	/** minimum value of extended coordinate of a joint */
	GenCoord min, minOffset;
	/** maximum value of extended coordinate of a joint */
	GenCoord max, maxOffset;
	/** minimum and maximum limits' offset */
	GenCoord offset;

	/** Exponential coordinates of the joint */
	ExpCoord::Seq trnSeq;
		/** initial frame transformation */
	Mat34 trnInit;

	/** bounds and visualisation of the joint */
	Bounds::Desc::Seq boundsDescSeq;

	/** Joint data members critical section */
	mutable CriticalSection csData;
	
	/** Creates Joint from the description. 
	* @param desc	Joint description
	*/
	void create(const Desc& desc);

	Joint(Chain& chain);

public:
	/** Destructor */
	virtual ~Joint() {}

	/** Adds the bounds description in the local coordinate frame of a given Joint
	 * @param pDesc			description of the bounds
	*/
	virtual bool addBoundsDesc(Bounds::Desc::Ptr pDesc);

	/** Removes the bounds description of a given Joint
	 * @param pDesc			the bounds to be removed
	*/
	virtual bool removeBoundsDesc(const Bounds::Desc* pDesc);
	
	/** Returns reference to the collection of bounds description of the Joint 
	 * @return				reference to the collection of bounds description
	*/
	virtual Bounds::Desc::SeqPtr getBoundsDescSeq() const;
	
	/** Checks if there is non empty the collection of bounds description of the Joint
	* @return		<code>true</code> if bounds are available; <code>false</code> otherwise 
	*/
	virtual bool hasBoundsDesc() const;
	
	/** Returns Name ASCII string */
	const std::string& getName() const {
		return name;
	}
	
	/** Joint index */
	const Configspace::Index& getIndex() const {
		return index;
	}
	/** Joint index in the chain */
	idx_t getIndexLocal() const {
		return indexLocal;
	}

	/** Returns minimum value of extended coordinate of a joint */
	const GenCoord& getMin() const {
		return min;
	}
	/** Returns minimum value with safety offset of extended coordinate of a joint */
	const GenCoord& getMinOffset() const {
		return minOffset;
	}

	/** Returns maximum value of extended coordinate of a joint */
	const GenCoord& getMax() const {
		return max;
	}
	/** Returns maximum value with safety offset of extended coordinate of a joint */
	const GenCoord& getMaxOffset() const {
		return maxOffset;
	}

	/** Returns minimum and maximum limits' offset */
	const GenCoord& getOffset() const {
		return offset;
	}

	/** Exponential coordinates of the joint */
	const ExpCoord::Seq& getTrnSeq() const {
		return trnSeq;
	}

	/** exponential coordinates of the joint */
	const ExpCoord& getTrn() const {
		return trnSeq[0];
	}

	/** Returns exponential coordinates defining initial frame transformation */
	const Mat34& getTrnInit() const {
		return trnInit;
	}

	/** Chain owner object */
	const golem::Chain& getChain() const {
		return chain;
	}
	/** Chain owner object */
	golem::Chain& getChain() {
		return chain;
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CTRL_JOINT_H_*/
