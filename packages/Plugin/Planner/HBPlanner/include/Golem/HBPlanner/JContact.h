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
#ifndef _GOLEM_HBPLANNER_JCONTACT_H_
#define _GOLEM_HBPLANNER_JCONTACT_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Manipulator.h>
#include <Golem/Ctrl/Data.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Type of guards for Justin */
enum FTGuardTypes {
	/** Absolute values */
	FTGUARD_ABS = 0,
	/** Less than */
	FTGUARD_LESSTHAN,
	/**Greater than */
	FTGUARD_GREATERTHAN,
};
/** HandChains */
enum HandChain {
	HC_UNKNOWN = 0,
	THUMB,
	INDEX,
	MIDDLE,
	RING,
	PINKY,
};
/** Mode */
enum Mode {
	DISABLE = 0,
	ENABLE,
	INCONTACT,
};
/** Face */
enum Face {
	UNKNOWN = 0,
	FRONT, // FT sensor face
	RIGHT, // from the nail
	LEFT,
	BACK,
	TIP,
	TOP,
};


//------------------------------------------------------------------------------
// base interface for guards
class Guard {
public:
	typedef std::vector<Guard> Seq;

	/** Force/torque threshold for the guard */
	Real threshold;
	/** Force/torque measured */
	Real force;

	class Desc {
	public:
		std::string name;
		
	};
};
typedef std::vector<Face> FaceSeq;

/** Force/torque guards for Justin and BHAM robots */
class GOLEM_LIBRARY_DECLDIR FTGuard {
public:
	typedef std::vector<FTGuard> Seq;
	typedef std::vector<FTGuard*> SeqPtr;

	/** FT sensors in 6 dimensions */
	static const size_t DIM = 6;
	/** Arm joints */
	static const size_t ARM = 7;
	/** Joint per finger */
	static const size_t JOINTSPERFINGER = 4;

	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		/** Type of guard {|>,<,>} */
		FTGuardTypes type;
		/** Force/torque threshold for the guard */
		RealSeq limits;
		/** Hand chain */
		HandChain chain;
		/** Initial mode */
		Mode mode;

		/** Joint index at which the sensor is attached */
		Configspace::Index jointIdx;

		Desc() {
			setToDefault();
		}

		void setToDefault() {
			type = FTGuardTypes::FTGUARD_ABS;
			limits.assign(DIM, Real(0.1));
			chain = HandChain::HC_UNKNOWN;
			jointIdx = Configspace::Index(0);
			mode = Mode::ENABLE;
		}

		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(limits.size() != DIM, ac, "limits dimention does not agree");
			for (auto i = limits.begin(); i != limits.end(); ++i)
				Assert::valid(std::isfinite(*i), ac, "elements in limit is not finite");
		}
		/** Creates the object from the description. */
		virtual FTGuard* create() const;

		inline void setLimits(Real* v) {
			for (size_t i = 0; i < DIM; ++i)
				limits[i] = v[i];
		}
	};

	/** Mode of the ft guard */
	Mode mode;
	/** Wrench treshold */
	RealSeq limits;
	/** Current FT sensor wrench */
	Twist wrench;
	/** Joint index at which the sensor is attached */
	Configspace::Index jointIdx;
	/** Faces in contact */
	FaceSeq faces;

	/** Enable guard after a contact */
	inline void unlock() { this->mode = Mode::ENABLE; }
	/** Check for disable mode */
	inline bool isDisable() const { return this->mode == Mode::DISABLE; }
	/** Check for enable mode */
	inline bool isEnable() const { return this->mode == Mode::ENABLE; }
	/** Check for contact mode */
	inline bool isInContact() const { return this->mode == Mode::INCONTACT; };
	/** Check for contact mode */
	bool checkContacts();


	/** Chain name */
	static const std::string ChainName[];
	/** Mode name */
	static const char* ModeName[];
	/** Face name */
	static const char* FaceName[];

	/** Set mode */
	inline void setMode(const Mode& mode) {
		this->mode = mode;
	}

	inline U32 getHandJoint() const {
		return 3;
	}
	// Get chain from [0,..,2]
	inline U32 getHandChain() const { return chain - 1; }

	/** Guard in string */
	void str(Context& context) const;

	std::string strForces() const;


	/** Check a list of sensors for contacts */
	static inline U32 triggered(FTGuard::SeqPtr& seq) {
		U32 contacts = U32(0);
		for (auto i = seq.begin(); i != seq.end(); ++i)
			if ((*i)->isInContact())
				++contacts;
		return contacts;
	}

	inline void set(const Vec3& v, const Vec3& w) {
		wrench.setV(v);
		wrench.setW(w);
	}

	inline void getColumn6(Real* v) const {
		v[0] = wrench.getV().x;
		v[1] = wrench.getV().y;
		v[2] = wrench.getV().z;
		v[3] = wrench.getW().x;
		v[4] = wrench.getW().y;
		v[5] = wrench.getW().z;
	}

	inline void setLimits(Real* v) {
		for (size_t i = 0; i < DIM; ++i)
			limits[i] = v[i];
	}

	/** Writes 3 consecutive values from the ptr passed
	*	@param	v	Array to read elements from.
	*/
	inline void setColumn6(const Real* v) {
		wrench.setV(v[0], v[1], v[2]);
		wrench.setW(v[3], v[4], v[5]);
	}


protected:
	/** Type of guard {|>,<,>} */
	FTGuardTypes type;

	/** Finger on which the FT sensor is attached */
	HandChain chain;

	/** Creates from a description file */
	void create(const Desc& desc);

	/** C'ctor */
	FTGuard();
};
/** Reads/writes guards from/to a given XML context */
//void XMLData(FTGuard::Desc& val, XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _GOLEM_HBPLANNER_JCONTACT_H_ */
