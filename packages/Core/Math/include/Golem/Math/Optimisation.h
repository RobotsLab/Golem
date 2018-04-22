/** @file Optimisation.h
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
#ifndef _GOLEM_GOLEM_OPTIMISATION_H_
#define _GOLEM_GOLEM_OPTIMISATION_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>
#include <Golem/Math/Rand.h>
#include <Golem/Defs/Pointers.h>
#include <Golem/Sys/Context.h>
#include <Golem/Sys/XMLParser.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <vector>
#include <functional>

//------------------------------------------------------------------------------

namespace golem {

/** Optimisation heuristic. */
template <typename _ThreadData, typename _Vec, typename _Type> class DEHeuristic {
public:
	/** Thread data */
	typedef _ThreadData ThreadData;
	/** Vector type */
	typedef _Vec Vec;
	/** Vector element type */
	typedef _Type Type;

	/** Initialisation */
	DEHeuristic(Context& context, size_t size = 0) : context(context), _size(size) {
	}

	/** Vector size */
	inline size_t size() const {
		return _size;
	}
	/** Vector size */
	inline void resize(size_t _size) {
		this->_size = _size;
	}

	/** Thread initialisation */
	inline void init(ThreadData& threadData, Rand& rand) const {
	}
	/** Vector sampling */
	inline bool sample(Rand& rand, Vec& vector, Type& value) {
		return true;
	}
	/** Vector processing */
	inline void process(Vec& vector, ThreadData& threadData) const {
	}
	/** Objective function */
	inline Type value(Vec& vector, ThreadData& threadData) const {
		return numeric_const<Type>::ZERO;
	}
	/** Collision function */
	inline bool collides(Vec& vector, ThreadData& threadData) const {
		return false;
	}
	/** Default distance function implemented as Euclidean distance */
	inline Type distance(const Vec& a, const Vec& b) const {
		return numeric_const<Type>::ZERO;
	}

	/** Thread context */
	inline ThreadData threadData() const {
		return ThreadData();
	}

	/** Context */
	inline Context& getContext() {
		return context;
	}
	/** Context */
	inline const Context& getContext() const {
		return context;
	}

protected:
	/** Context */
	Context& context;
	/** Vector size */
	size_t _size;
};

//------------------------------------------------------------------------------

/** Optimisation heuristic with callback functions. */
template <typename _ThreadData, typename _Vec, typename _Type> class DEHeuristicFunc : public golem::DEHeuristic<_ThreadData, _Vec, _Type> {
public:
	/** Heuristic base class */
	typedef golem::DEHeuristic<_ThreadData, _Vec, _Type> Heuristic;

	/** Thread data */
	typedef typename Heuristic::ThreadData ThreadData;
	/** Vector type */
	typedef typename Heuristic::Vec Vec;
	/** Value type */
	typedef typename Heuristic::Type Type;

	/** Sample function */
	typedef std::function<void(ThreadData&, golem::Rand&)> Init;
	/** Sample function */
	typedef std::function<bool(golem::Rand&, Vec&, Type&)> Sample;
	/** Process function */
	typedef std::function<void(Vec&, ThreadData&)> Process;
	/** Objective function */
	typedef std::function<Type(const Vec&, ThreadData&)> Value;
	/** Distance function */
	typedef std::function<Type(const Vec&, const Vec&)> Distance;

	/** Init function */
	Init pInit;
	/** Sample function */
	Sample pSample;
	/** Process function */
	Process pProcess;
	/** Objective function */
	Value pValue;
	/** Distance function */
	Distance pDistance;

	/** Initialisation */
	DEHeuristicFunc(Context& context, size_t size = 0) : Heuristic(context, size) {
	}

	/** Thread initialisation */
	inline void init(ThreadData& threadData, Rand& rand) const {
		if (pInit) pInit(threadData, rand);
	}
	/** Vector sampling */
	inline bool sample(golem::Rand& rand, Vec& vector, Type& value) {
		return !pSample || pSample(rand, vector, value);
	}
	/** Vector processing */
	inline void process(Vec& vector, ThreadData& threadData) const {
		if (pProcess) pProcess(vector, threadData);
	}
	/** Objective function */
	inline Type value(Vec& vector, ThreadData& threadData) const {
		return pValue ? pValue(vector, threadData) : golem::numeric_const<Type>::ZERO;
	}
	/** Default distance function implemented as Euclidean distance */
	inline Type distance(const Vec& a, const Vec& b) const {
		return pDistance ? pDistance(a, b) : golem::numeric_const<Type>::ZERO;
	}
};

//------------------------------------------------------------------------------

/** Global optimisation tool which uses population of solutions based on differential evolution. */
template <typename _Heuristic> class DEOptimisation : protected Runnable {
public:
	/** Pointer */
	typedef shared_ptr<DEOptimisation> Ptr;

	/** Heuristic type */
	typedef _Heuristic Heuristic;
	/** Context type */
	typedef typename _Heuristic::ThreadData ThreadData;
	/** Vector type */
	typedef typename _Heuristic::Vec Vec;
	/** Value type */
	typedef typename _Heuristic::Type Type;

	/** Sequence of vectors*/
	typedef std::vector<Vec> VecSeq;
	/** Sequence of vectors value */
	typedef std::vector<Type> ValSeq;

	/** Description */
	class Desc {
	public:
		/** Search for minimum/maximum value */
		bool minimum;
		/** population size */
		size_t populationSize;
		/** number of generations */
		size_t generationsNum;

		/** Vector difference factor */
		Type diffFac;
		/** Crossover probability */
		Type crossProb;

		/** convergence test value */
		bool testValue;
		/** number of generations between convergence tests */
		size_t testGenerations;
		/** number of pairs used in convergence tests */
		size_t testPairs;
		/** convergence test variance threshold */
		Type testVariance;

		/** Number of threads */
		size_t numOfThreads;
		/** Working threads priority */
		Thread::Priority priority;

		/** Constructs object. */
		Desc() {
			setToDefault();
		}
		/** Sets the parameters to the default values. */
		void setToDefault() {
			minimum = true;
			populationSize = 100;
			generationsNum = 10000000;
			diffFac = Type(0.5);
			crossProb = Type(0.1);
			testValue = false;
			testGenerations = 100;// -1 to disable
			testPairs = 100;
			testVariance = numeric_const<Type>::EPS;
			numOfThreads = -1;
			priority = Thread::LOWEST;
		}
		/** Checks if the parameters are valid. */
		bool isValid() const {
			if (populationSize < 4 || generationsNum < 1)
				return false;
			if (diffFac <= numeric_const<Type>::ZERO)
				return false;
			if (crossProb < numeric_const<Type>::ZERO || crossProb > numeric_const<Type>::ONE)
				return false;
			if (testGenerations <= 0 || !Math::isPositive(testVariance))
				return false;
			return true;
		}
		/** Reads parameters from a given XML context */
		void load(XMLContext* xmlcontext) {
			XMLData("minimum", minimum, xmlcontext);
			XMLData("population_size", populationSize, xmlcontext);
			XMLData("generations_num", generationsNum, xmlcontext);
			XMLData("de_diff_fac", diffFac, xmlcontext);
			XMLData("de_cross_prob", crossProb, xmlcontext);
			XMLData("test_value", testValue, xmlcontext);
			XMLData("test_generations", testGenerations, xmlcontext);
			XMLData("test_pairs", testPairs, xmlcontext);
			XMLData("test_variance", testVariance, xmlcontext);
			XMLData("num_of_threads", numOfThreads, xmlcontext);
			XMLData("thread_priority", priority, xmlcontext);
		}
	};

	/** Create optimisation */
	DEOptimisation(Heuristic& heuristic) : heuristic(heuristic), context(heuristic.getContext()), pParallels(heuristic.getContext().getParallels()) {}

	/** Start search */
	void start(const Desc& desc, MSecTmU32 timeOut = MSEC_TM_U32_INF) {
		// check Parallels
		if (pParallels == NULL)
			throw Message(Message::LEVEL_CRIT, "DEOptimisation::start(): Parallels required");
		// copy description
		if (!desc.isValid())
			throw Message(Message::LEVEL_ERROR, "DEOptimisation::start(): Invalid description");
		this->desc = desc;
		// initialise computation
		vectorSeq.resize(desc.populationSize);
		valueSeq.resize(desc.populationSize);
		vectorVar = numeric_const<Type>::MAX;
		generation = 0;
		runStop = timeOut < MSEC_TM_U32_INF ? heuristic.getContext().getTimer().elapsed() + MSecToSec(timeOut) : SEC_TM_REAL_MAX;
		bStop = false;
		// launch Parallels
		parallels(true);
	}
	/** Stop search, return solution index */
	size_t stop(bool wait = true) {
		// finish Parallels
		parallels(false, wait);
		// population size
		const size_t size = std::min(vectorSeq.size(), generation);
		if (size <= 0)
			throw Message(Message::LEVEL_ERROR, "DEOptimisation::stop(): No valid solutions");
		// find best solution
		size_t solution = 0;
		for (size_t j = 1; j < size; ++j)
			if (accept(valueSeq[solution], valueSeq[j]))
				solution = j;
		// done!
		return solution;
	}

	/** Description */
	inline const Desc& getDesc() const {
		return desc;
	}
	/** Population */
	inline const VecSeq& getVectors() const {
		return vectorSeq;
	}
	/** Population objective values */
	inline const ValSeq& getValues() const {
		return valueSeq;
	}
	/** Population variance */
	inline Type getVariance() const {
		return vectorVar;
	}
	/** Current generation */
	inline size_t getGenerations() const {
		return generation;
	}

protected:
	/** Heuristic */
	Heuristic& heuristic;
	/** Context */
	Context& context;
	/** Parallels */
	Parallels *pParallels;
	/** Description */
	Desc desc;
	/** Population */
	VecSeq vectorSeq;
	/** Population objective values */
	ValSeq valueSeq;
	/** Population variance */
	Type vectorVar;
	/** Current generation */
	size_t generation;
	/** Time stop condition */
	SecTmReal runStop;
	/** Stop condition */
	bool bStop;
	/** Thread cs */
	CriticalSection cs;

	/** Generates test vector */
	inline void generate(Rand& rand, const VecSeq& vectorSeq, Vec& nextVector, size_t& nextIndex) const {
		// choose random set of vectors (use random permutation for small population size)
		size_t index[4];
		for (size_t i = 0; i < 4;) {
		REPEAT:
			index[i] = rand.next()%vectorSeq.size();
			for (size_t j = 0; j < i; ++j)
				if (index[j] == index[i])
					goto REPEAT;
			++i;
		}
		nextIndex = index[0];

		// generate a candidate solution
		const size_t crossPoint = rand.next()%heuristic.size();
		bool bCross = true;
		for (size_t i = 0; i < heuristic.size(); ++i) {
			const size_t j = (crossPoint + i)%heuristic.size();
			if (bCross) {
				// apply crossover accessor
				nextVector[j] = vectorSeq[index[1]][j] + desc.diffFac*(vectorSeq[index[2]][j] - vectorSeq[index[3]][j]);
				// decide when to stop crossover
				bCross = desc.crossProb < rand.nextUniform<Type>();
			}
			else {
				// just copy if crossover or coordinate is disabled
				nextVector[j] = vectorSeq[index[0]][j];
			}
		}
	}
	/** Solution acceptance */
	inline bool accept(Type prevValue, Type nextValue) const {
		return desc.minimum ? prevValue > nextValue : prevValue < nextValue;
	}
	/** Find population value variance */
	inline Type variance(const ValSeq& valueSeq) const {
		// population variance
		Variable<Type> variable(valueSeq.begin(), valueSeq.end());
		return variable.getVarianceN();
	}
	/** Find population vector variance */
	inline Type variance(Rand& rand, const VecSeq& vectorSeq) const {
		// population size
		const size_t size = std::min(vectorSeq.size(), generation);
		// population variance
		Variable<Type> variable;
		for (size_t i = 0; i < desc.testPairs; ++i) {
			const size_t j = rand.next()%size;
			const size_t k = (j + 1 + rand.next()%(size - 1))%size;
			variable.update(heuristic.distance(vectorSeq[j], vectorSeq[k]));
		}
		return variable.getVarianceN();
	}
	/** Stop condition */
	inline bool stop(Rand& rand, const ValSeq& valueSeq, const VecSeq& vectorSeq, Type& vectorVar) const {
		if (generation >= desc.generationsNum || generation%desc.testGenerations == 0)
			return (vectorVar = desc.testValue ? variance(valueSeq) : variance(rand, vectorSeq)) < desc.testVariance || generation >= desc.generationsNum;
		return heuristic.getContext().getTimer().elapsed() > runStop;
	}

	/** Runs working threads */
	void run() {
		Vec vector; // no initialisation - make sure that both sample() and generate() initialise *ALL* coordinates (including disabled ones!)
		Type value;
		size_t index;
		ThreadData threadData(heuristic.threadData());
		Rand rand(RandSeed(context.getRandSeed()._U32[0] + (desc.numOfThreads > 0 ? pParallels->getCurrentJob()->getJobId() : (U32)0), (U32)0));
		
		// thread initialisation
		heuristic.init(threadData, rand);

		for (bool bAccept = false;;) {
			{
				CriticalSectionWrapper csw(cs, desc.numOfThreads > 0);

				// increase generation index
				if (generation >= vectorSeq.size() || bAccept)
					generation += 1;

				// check if the next solution is better than the previous one
				if (bAccept) {
					vectorSeq[index] = vector;
					valueSeq[index] = value;
					bAccept = false;
				}

				// stop condition - the maximum number of generations or passed test
				if (bStop || generation > 0 && stop(rand, valueSeq, vectorSeq, vectorVar)) {
					bStop = true; // stop other threads
					break;
				}

				if (generation < vectorSeq.size()) {
					// sample test vector
					bAccept = heuristic.sample(rand, vector, value);
					index = generation;
				}
				else {
					// generate test vector
					generate(rand, vectorSeq, vector, index);
					value = valueSeq[index]; // get old value corresponding to index
				}
			}

			// compute value of test vector
			heuristic.process(vector, threadData);
			const Type nextValue = heuristic.value(vector, threadData);
			// accept if there is no collision
			if (!bAccept && accept(value, nextValue) && !heuristic.collides(vector, threadData))
				bAccept = true;
			// always update here!
			value = nextValue;
		}
	}

	/** Starts/stops Parallels */
	void parallels(bool start, bool wait = true) {
		// no Parallels - run in the current thread with blocking
		if (desc.numOfThreads <= 0) {
			if (start)
				run();
			return;
		}

		if (start) {
			// launch Parallels
			for (size_t t = std::min(vectorSeq.size(), std::min(desc.numOfThreads, (size_t)pParallels->getNumOfThreads())); t > 0; --t) {
				Job* pJob = pParallels->startJob(this);
				if (!pJob)
					throw Message(Message::LEVEL_CRIT, "DEOptimisation::parallels(): Unable to start job");
				pJob->setThreadPriority(desc.priority);
				// TODO store run ID to identify threads launched by other objects
			}
		}
		else {
			// wait for completion or stop now
			bStop = !wait;
			if (!pParallels->joinJobs()) {
				bStop = true;
				(void)pParallels->joinJobs();
			}
		}
	}
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_GOLEM_OPTIMISATION_H_*/
