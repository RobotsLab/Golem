//------------------------------------------------------------------------------
//               Simultaneous Perception And Manipulation (SPAM)
//------------------------------------------------------------------------------
//
// Copyright (c) 2010 University of Birmingham.
// All rights reserved.
//
// Intelligence Robot Lab.
// School of Computer Science
// University of Birmingham
// Edgbaston, Brimingham. UK.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// with the Software without restriction, including without limitation the 
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
// sell copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
//     * Redistributions of source code must retain the above copyright 
//       notice, this list of conditions and the following disclaimers.
//     * Redistributions in binary form must reproduce the above copyright 
//       notice, this list of conditions and the following disclaimers in 
//       the documentation and/or other materials provided with the 
//       distribution.
//     * Neither the names of the Simultaneous Perception And Manipulation 
//		 (SPAM), University of Birmingham, nor the names of its contributors  
//       may be used to endorse or promote products derived from this Software 
//       without specific prior written permission.
//
// The software is provided "as is", without warranty of any kind, express or 
// implied, including but not limited to the warranties of merchantability, 
// fitness for a particular purpose and noninfringement.  In no event shall the 
// contributors or copyright holders be liable for any claim, damages or other 
// liability, whether in an action of contract, tort or otherwise, arising 
// from, out of or in connection with the software or the use of other dealings 
// with the software.
//
//------------------------------------------------------------------------------
//! @Author:   Claudio Zito
//! @Date:     27/10/2012
//------------------------------------------------------------------------------
#pragma once
#ifndef _GOLEM_HBPLANNER_HYPOTHESIS_H_
#define _GOLEM_HBPLANNER_HYPOTHESIS_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/RBPose.h>
#include <Golem/HBPlanner/Collision.h>

//------------------------------------------------------------------------------

namespace flann {
	template <typename T> struct L2_Simple;
};

namespace pcl {
	struct PointXYZ;
	template <typename T, typename Dist> class KdTreeFLANN;
	struct PolygonMesh;
};

namespace grasp {
	class Manipulator;
};

namespace spam {
	class FTDrivenHeuristic;
	class Belief;
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Hypothesis over object poses */
class GOLEM_LIBRARY_DECLDIR Hypothesis {
public:
	friend class HBHeuristic;
	friend class Belief;
	typedef shared_ptr<Hypothesis> Ptr;
	typedef std::map<U32, Ptr> Map;
	typedef std::vector<Ptr> Seq;

	/** Bounds Appearance */
	class GOLEM_LIBRARY_DECLDIR BoundsAppearance {
	public:
		/** Show bounds solid */
		bool showSolid;
		/** Show bounds wire frames */
		bool showWire;
		/** Bounds solid colour */
		RGBA solidColour;
		/** Bounds wire colour */
		RGBA wireColour;
		/** Bounds wireframe thickness */
		Real wireWidth;

		/** Constructs from description object */
		BoundsAppearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showSolid = false;
			showWire = true;
			solidColour = RGBA(U8(255), U8(255), U8(0), U8(127));
			wireColour = RGBA(U8(255), U8(255), U8(0), U8(127));
			wireWidth = Real(1.0);
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (wireWidth <= REAL_ZERO)
				return false;
			return true;
		}

		/** Draw bounds */
		void draw(const Bounds::Seq& bounds, DebugRenderer& renderer) const;
	};

	/** Appearance */
	class GOLEM_LIBRARY_DECLDIR Appearance {
	public:
		/** Show frame */
		bool showFrames;
		/** Show point cloud */
		bool showPoints;
		/** Frame size of the sample */
		Vec3 frameSize;
		/** clolour of the point cloud */
		RGBA colour;

		/** Bounds colour */
		BoundsAppearance bounds;

		/** Constructs from description object */
		Appearance() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			showFrames = true;
			showPoints = true;
			frameSize.set(Real(0.02));
			colour = RGBA::MAGENTA;
			bounds.setToDefault();
		}
		/** Checks if the description is valid. */
		bool isValid() const {
			if (!bounds.isValid())
				return false;
			if (!frameSize.isPositive())
				return false;
			return true;
		}
	};

	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		typedef shared_ptr<Desc> Ptr;

		Appearance appearance;
		HBCollision::Desc::Ptr collisionDescPtr;

		/** Constructs description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Nothing to do here */
		virtual ~Desc() {}
		/** Creates the object from the description. */
		virtual Hypothesis::Ptr create(const Manipulator& manipulator) const {
			return Hypothesis::Ptr(new Hypothesis(manipulator, *this));
		}
		/** Sets description to default values */
		void setToDefault() {
			appearance.setToDefault();
			collisionDescPtr.reset(new HBCollision::Desc());
		}
		/** Checks if the description is valid. */
		virtual bool isValid() const {
			if (!appearance.isValid())
				return false;
			if (collisionDescPtr != nullptr && !collisionDescPtr->isValid())
				return false;
			return true;
		}

	};

	/** Create */
	virtual void create(const U32 idx, const Mat34 &trn, const RBPose::Sample &s, Rand& rand, const Cloud::PointSeq& points);

	/** Returns this sample in model frame **/
	inline RBPose::Sample toRBPoseSample() const { return sample; };
	/** Returns this sample in global frame (default: robot frame) **/
	inline RBPose::Sample toRBPoseSampleGF() const { return RBPose::Sample(sample.toMat34() * modelFrame, sample.weight, sample.cdf); };
	/** Returns the point cloud in global frame */
	inline Cloud::PointSeq getCloud() const { return points; };

	/** Collision detection at a given waypoint */
	inline bool checkNN(const HBCollision::FlannDesc& desc, const Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->checkNN(desc, config, debug);
	}

	/** Collision detection at a given waypoint */
	inline virtual Real estimate(const HBCollision::FlannDesc& desc, const Manipulator::Config& config, Real maxDist = REAL_MAX, bool debug = false) const {
		return collisionPtr->estimate(desc, config, maxDist, debug);
	}

	/** Collision likelihood estimation at a given waypoint */
	inline Real evaluate(const HBCollision::Waypoint& waypoint, const Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->evaluate(waypoint, config, debug);
	}
	/** Collision likelihood estimation at a given waypoint */
	inline Real evaluate(const HBCollision::FlannDesc& desc, const Manipulator::Config& config, bool debug = false) const {
		return collisionPtr->evaluate(desc, config, debug);
	}

	/** Return seq of bounds */
	Bounds::Seq bounds();

	/** Prints global pose of the hypothesis */
	std::string str() const;

	/** Draw hypotheses */
	void draw(DebugRenderer &renderer) const;

	/** Draw collisions */
	void draw(const HBCollision::Waypoint &waypoint, const Manipulator::Config& config, DebugRenderer& renderer) const;
	/** Draw collision using kdtree */
	void draw(DebugRenderer& renderer, const Rand& rand, const Manipulator::Config& config) const;
	/** Draw estimate */
	void draw(DebugRenderer& renderer, const Manipulator::Config& config, const HBCollision::FlannDesc& desc) const {
		collisionPtr->draw(renderer, config, desc);
	}
	/** Draw simulate */
	void draw(DebugRenderer& renderer, const Manipulator::Config& config, std::vector<Configspace::Index> &joints, RealSeq &forces, const HBCollision::FlannDesc& desc) const {
		collisionPtr->draw(renderer, config, joints, forces, desc);
	}
	Appearance appearance;

protected:
	/** Identifier */
	U32 index;
	/** Model frame **/
	Mat34 modelFrame;
	/** Hypothesis. NOTE: contains the query (or sample) frame w.r.t model frame **/
	RBPose::Sample sample;
	/** Point cloud */
	Cloud::PointSeq points;

	/** Bounding box desc for the object */
	BoundingBox::Desc boundsDesc;


	/** Manipulator */
	const Manipulator& manipulator;
	/** Description */
	const Desc desc;

	/** Collision detection pointer */
	HBCollision::Ptr collisionPtr;

	/** Create */
	Hypothesis(const Manipulator& manipulator, const Desc& desc);
};

//------------------------------------------------------------------------------

void XMLData(Hypothesis::Desc& val, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _GOLEM_HBPLANNER_HYPOTHESIS_H_ */