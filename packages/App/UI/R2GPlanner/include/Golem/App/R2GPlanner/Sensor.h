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
#ifndef _GOLEM_APP_R2GPLANNER_SENSOR_H_
#define _GOLEM_APP_R2GPLANNER_SENSOR_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Manipulator.h>
#include <Golem/Tools/FT.h>
#include <Golem/HBPlanner/Collision.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* graspDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR SensorBundle : public golem::Runnable  {
public:
	/** Pointer */
	typedef golem::shared_ptr<SensorBundle> Ptr;
	/** Force reader. */
	typedef std::function<golem::RealSeq(golem::RealSeq&)> ForceReader;
	/** Force filter.  */
	typedef std::function<golem::RealSeq(void)> ForceFilter;
	/** FT Sequence */
	typedef std::vector<golem::FT*> FTSensorSeq;

	/** Modes */
	enum ForceMode {
		/** Inactive */
		FORCE_MODE_DISABLED = 0,
		/** Torque */
		FORCE_MODE_FT,
		/** Force */
		FORCE_MODE_JOINT,
	};

	/** Read raw input forces */
	class GOLEM_LIBRARY_DECLDIR Reader {
	public:
		/** Pointer */
		typedef golem::shared_ptr<Reader> Ptr;

		class GOLEM_LIBRARY_DECLDIR Desc {
		public:
			/** Pointer */
			typedef golem::shared_ptr<Desc> Ptr;

			/** Reader implementation */
			ForceReader forceReader;
			
			Desc() {
				setToDefault();
			}

			void setToDefault() {
				forceReader = nullptr;
			}

			virtual void assertValid(const golem::Assert::Context &ac) const {
			}

			/** Load descritpion from xml context. */
			virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
			/** Creates the object from the description. */
			GOLEM_CREATE_FROM_OBJECT_DESC1(Reader, Reader::Ptr, golem::Context&)
		};

		golem::RealSeq read(golem::RealSeq& force);

		/** Force filter */
		void setForceReader(ForceReader reader) {
			golem::CriticalSectionWrapper csw(cs);
			forceReader = reader;
		};

	protected:
		/** Context */
		golem::Context context;
		/** Critical sections */
		golem::CriticalSection cs;
		
		/** Reader implementation */
		ForceReader forceReader, defaultForceReader;

		/** Create from description */
		void create(const Desc& desc);
		/** C'tor */
		Reader(golem::Context& context);
	};

	/** Filter the noisy input readings */
	class GOLEM_LIBRARY_DECLDIR Filter {
	public: 
		/** Pointer */
		typedef golem::shared_ptr<Filter> Ptr;

		class GOLEM_LIBRARY_DECLDIR Desc {
		public:
			/** Pointer */
			typedef golem::shared_ptr<Desc> Ptr;

			/** Filter implementation */
			ForceFilter forceFilter;

			/** Dimensionality of the filter (DoFs)*/
			golem::U32 dimensionality;

			/** Size of reading collected */
			golem::U32 windowSize;
			/** Initial step */
			golem::I32 steps;
			/** Incrememnt */
			golem::Real delta;
			/** gaussian filter mask */
			golem::RealSeq mask;
			/** Variance for the gaussian filter */
			golem::Real sigma;

			Desc() {
				setToDefault();
			}

			void setToDefault() {
				forceFilter = nullptr;

				dimensionality = 18;

				steps = 0;
				windowSize = 40;
				delta = .1;
				mask.assign(windowSize, golem::REAL_ZERO);
				sigma = golem::Real(2.65);
			}

			virtual void assertValid(const golem::Assert::Context &ac) const {
				golem::Assert::valid(dimensionality > golem::U32(0), ac, "Sensor Filter description: invalid dimensionality");
				golem::Assert::valid(windowSize > golem::U32(0), ac, "Sensor Filter description: invalid window size");
				golem::Assert::valid(delta > golem::REAL_ZERO, ac, "Sensor Filter description: invalid sigma");
				golem::Assert::valid(sigma > golem::REAL_ZERO, ac, "Sensor Filter description: invalid sigma");
			}
			/** Load descritpion from xml context. */
			virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);
			/** Creates the object from the description. */
			GOLEM_CREATE_FROM_OBJECT_DESC1(Filter, Filter::Ptr, golem::Context&)
		};

		// Return hand DOFs */
		inline size_t dimensions() const { return dimensionality; }// (size_t)handInfo.getJoints().size();

		golem::RealSeq filter(const golem::RealSeq& force);

		/** Force filter */
		void setForceFilter(ForceFilter filter) {
			golem::CriticalSectionWrapper csw(cs);
			forceFilter = filter;
		};

	protected:
		/** Context */
		golem::Context context;
		/** Critical sections */
		golem::CriticalSection cs, csInp;

		/** Filter implementation */
		ForceFilter forceFilter, defaultForceFilter;

		/** Dimensionality of the filter (DoFs)*/
		golem::U32 dimensionality;

		/** Number of averaging steps before stopping collecting readings */
		golem::U32 windowSize;
		golem::I32 steps;
		golem::Real delta;
		// gaussian filter mask
		golem::RealSeq mask;
		/** Variance for the gaussian filter */
		golem::Real sigma;

		/** Input force at sensor, sequence */
		std::vector<golem::RealSeq> forceInpSensorSeq;

		template <typename _Real> inline _Real N(const _Real x, const _Real stdev) {
			const _Real norm = golem::numeric_const<_Real>::ONE / (stdev*golem::Math::sqrt(2 * golem::numeric_const<_Real>::PI));
			return norm*golem::Math::exp(-.5*golem::Math::sqr(_Real(x) / _Real(stdev))); // gaussian
		};
		// computes guassian on a vector
		template <typename _Ptr, typename _Real> inline std::vector<_Real> N(_Ptr begin, _Ptr end, const size_t dim, const _Real stdev) {
			std::vector<_Real> output;
			output.assign(dim, golem::numeric_const<_Real>::ZERO);
			size_t idx = 0;
			for (_Ptr i = begin; i != end; ++i) {
				output[idx++] = N(*i, stdev);
			}
			return output;
		};

		/** Create from description */
		void create(const Desc& desc);
		/** C'tor */
		Filter(golem::Context& context) : context(context) {};
	};

	class GOLEM_LIBRARY_DECLDIR Desc {
	public:
		/** Pointer */
		typedef golem::shared_ptr<Desc> Ptr;

		Reader::Desc::Ptr readerDescPtr;
		Filter::Desc::Ptr filterDescPtr;

		std::string path, sepField, ext;

		/** Working thread priority */
		golem::Thread::Priority threadPriority;
		/** Cycle time */
		golem::SecTmReal tCycle;
		/** Idle time */
		golem::SecTmReal tIdle;

		/** Collision description file. Used for collision with the ground truth */
		golem::HBCollision::Desc::Ptr objCollisionDescPtr;

		Desc() {
			setToDefault();
		}

		void setToDefault() {
			readerDescPtr.reset(new Reader::Desc);
			filterDescPtr.reset(new Filter::Desc);

			path = "./data/boris/experiments/ftsensors/";
			sepField = "-";
			ext = ".txt";

			threadPriority = golem::Thread::HIGHEST;
			tCycle = golem::SecTmReal(0.02);
			tIdle = golem::SecTmReal(0.01);

			objCollisionDescPtr.reset(new golem::HBCollision::Desc());
		}

		virtual void assertValid(const golem::Assert::Context &ac) const {
			readerDescPtr->assertValid(ac);
			filterDescPtr->assertValid(ac);
			golem::Assert::valid(objCollisionDescPtr != nullptr, ac, "SensorBundle Descritor: Collision description invalid");
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC2(SensorBundle, SensorBundle::Ptr, golem::Controller&, golem::Manipulator&)
	};

	/** Set the sequence of sensors */
	void setSensorSeq(const golem::Sensor::Seq& sensorSeq);

	inline golem::RealSeq getFilteredForces() {
		golem::CriticalSectionWrapper csw(cs);
		return filteredForce;
	}

	inline void stop() {
		bTerminate = true;
		release();
	}

	inline void setManipulator(golem::Manipulator* manipulator) {
		this->manipulator.reset(manipulator);
		collisionPtr = desc.objCollisionDescPtr->create(*this->manipulator.get());
	}

	inline golem::HBCollision::Ptr& getCollisionPtr() {
		return collisionPtr;
	}

	inline void enable() {
		mode = forceMode;
	}

	inline void disable() {
		mode = ForceMode::FORCE_MODE_DISABLED;
	}

	/** Render */
	void render() {
		golem::CriticalSectionWrapper csw(csRender);
		debugRenderer.render();
	}

	inline golem::HBCollision::FlannDesc getFlannDesc() {
		return desc.objCollisionDescPtr->flannDesc;
	}

protected:
	/** Context */
	golem::Context context;
	/** Genrator of pseudo-random numbers */
	golem::Rand rand;
	/** Pointer to controller */
	golem::Controller* controller;
	/** Manipulator interface */
	golem::Manipulator::Ptr manipulator;
	/** Debug renderer */
	golem::DebugRenderer debugRenderer;
	/** Description file */
	Desc desc;

	/** Force reader */
	Reader::Ptr reader;
	/** Filter */
	Filter::Ptr filter;

	/** Thread terminate condition */
	volatile bool bTerminate;
	/** I/O thread */
	golem::Thread thread;
	/** I/O thread priority */
	golem::Thread::Priority threadPriority;
	/** Inter-thread signalling time out */
	golem::MSecTmU32 threadTimeOut;

	/** Mode */
	ForceMode forceMode, mode;
	/** Time stamp of the reading forces */
	golem::U32 idx;

	/** Pointer to collision detection with the ground truth */
	golem::HBCollision::Ptr collisionPtr;

	/** FT Sensors */
	golem::Sensor::Seq sensorSeq;
	/** Sequence of FT sensors */
	FTSensorSeq ftSensorSeq;

	/** force reader access cs */
	golem::CriticalSection cs, csRender;

	/** Filtered forces for the hand */
	golem::RealSeq filteredForce;

	/** Communication function working in a separate thread. */
	virtual void run();

	/** Shuts down control thread and releases related resources */
	void release();

	/** User create is called just before launching I/O thread. */
	virtual void userCreate() {}

	/** I/O simulator timer */
	golem::shared_ptr<golem::Sleep> sleep;
	/** Time */
	golem::SecTmReal tRead;
	/** Cycle time */
	golem::SecTmReal tCycle;
	/** Idle time */
	golem::SecTmReal tIdle;

	void create(const Desc& desc);

	/** C'tor */
	SensorBundle(golem::Controller& ctrl, golem::Manipulator& manipulator);
};

//------------------------------------------------------------------------------

}; /** namespace */

#endif /** _GOLEM_APP_R2GPLANNER_SENSOR_H_ */
