/** @file OptimisationEvalCPU.h
 *
 * Contact model and config model evaluation routines for CPU
 *
 * @author Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CONTACT_OPTIMISATIONEVALCPU_H_
#define _GOLEM_CONTACT_OPTIMISATIONEVALCPU_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Configuration.h>
#include <Golem/Contact/Query.h>

//------------------------------------------------------------------------------

// Optimisation debug
//#define _GOLEM_CONTACT_OPTIMISATION_DEBUG

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Configuration */
template <typename _Real> class EvalConfigCPU {
public:
	_Real config[golem::Configspace::DIM];

	EvalConfigCPU() {}
	template <typename _RealConfig> EvalConfigCPU(size_t begin, size_t end, const _RealConfig* config) {
		for (size_t i = begin; i < end; ++i)
			this->config[i - begin] = static_cast<_Real>(config[i]);
	}

	inline _Real distance(size_t size, const EvalConfigCPU& config, const EvalConfigCPU& dist) const {
		_Real d = golem::numeric_const<_Real>::ZERO;
		for (size_t i = 0; i < size; ++i)
			d += dist.config[i] * golem::Math::sqr(this->config[i] - config.config[i]);
		return d;
	}
};

/** Configuration kernel */
template <typename _Real, typename _RealEval> class EvalConfigKernelCPU : public EvalConfigCPU<_Real> {
public:
	typedef std::vector<EvalConfigKernelCPU> Seq;

	_RealEval weight;

	EvalConfigKernelCPU() {}
	EvalConfigKernelCPU(size_t begin, size_t end, const Configuration::Kernel& k) :
		EvalConfigCPU<_Real>(begin, end, k.config.data()), weight(static_cast<_RealEval>(k.weight))
	{}
};

/** Configuration model */
template <typename _Real, typename _RealEval> class EvalConfigModelCPU {
public:
	typedef std::vector<EvalConfigModelCPU> Seq;
	typedef EvalConfigCPU<_Real> Config;
	typedef EvalConfigKernelCPU<_Real, _RealEval> Kernel;

	size_t begin, end, size;
	typename Kernel::Seq kernels;
	Config dist;
	_Real distMax;
	_RealEval weight;

	EvalConfigModelCPU() {}

	void create(const Configuration& configuration, const Configuration::Space& space) {
		begin = (size_t)*configuration.getManipulator().getHandInfo().getJoints().begin();
		end = (size_t)*configuration.getManipulator().getHandInfo().getJoints().end();
		size = end - begin;

		const golem::Configuration::Kernel::Seq& kernels = space.configs;
		if (kernels.size() <= 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "EvalConfigModelCPU::create(): invalid number of kernels %u", kernels.size());
		this->kernels.clear();
		this->kernels.reserve(kernels.size());
		for (Configuration::Kernel::Seq::const_iterator i = kernels.begin(); i != kernels.end(); ++i)
			this->kernels.push_back(Kernel(begin, end, *i));

		if (kernels.back().cdf < golem::numeric_const<golem::Real>::EPS)
			throw golem::Message(golem::Message::LEVEL_ERROR, "EvalConfigModelCPU::create(): kernels are not normalised");
		dist = Config(begin, end, space.desc.configCovInv.data());
		distMax = static_cast<_Real>(golem::Math::sqr(space.desc.distanceStdDevMax));
		weight = static_cast<_RealEval>(golem::numeric_const<golem::Real>::ONE / kernels.back().cdf);
	}

	inline _RealEval evaluate(const Config& config) const {
		_RealEval likelihood = golem::numeric_const<_RealEval>::ZERO, c = golem::numeric_const<_RealEval>::ZERO;
		for (const Kernel *i = kernels.data(), *end = i + kernels.size(); i < end; ++i) {
			const _Real distance = i->distance(size, config, dist);
			if (distance < distMax) {
				const _RealEval sampleLikelihood = i->weight*golem::Math::exp(-_RealEval(distance));
				golem::kahanSum(likelihood, c, sampleLikelihood);
			}
		}
		return weight*likelihood;
	}
};

//------------------------------------------------------------------------------

/** Rigid body kernel */
template <typename _Real, typename _RealEval> class EvalRBCoordKernelCPU : public _RBCoord<_Real>, public _RBCoordDist<_Real> {
public:
	typedef _RBCoord<_Real> RBCoord;
	//typedef RBCoordKernelCPU<_Real, _RealEval> RBCoordKernelCPU;
	typedef std::vector<EvalRBCoordKernelCPU> Seq;
	/** Dimensions per transformation */
	//static const size_t DIM = _RBCoord<_Real>::N + 1;// variable bandwidth
	static const size_t DIM = _RBCoord<_Real>::LIN_N;

	_RealEval weight;

	EvalRBCoordKernelCPU()
	{}
	EvalRBCoordKernelCPU(const golem::Query::Pose& pose) : _RBCoord<_Real>(pose), _RBCoordDist<_Real>(pose), weight((_RealEval)pose.weight)
	{}
	EvalRBCoordKernelCPU(const _RBCoord<_Real>& coord) : _RBCoord<_Real>(coord)
	{}
};

/** Contact */
template <typename _Real, typename _RealEval> class EvalContactModelCPU {
public:
	typedef _Real Real;
	typedef _RealEval RealEval;
	typedef _RBCoord<_Real> RBCoord;
	typedef EvalRBCoordKernelCPU<_Real, _RealEval> Kernel;
	typedef std::vector<EvalContactModelCPU> Seq;
	typedef _RBDist<_Real> RBDist;

	/** KD-tree based nn search */
	struct NNDist {
		typedef bool is_kdtree_distance;
		typedef _Real ElementType;
		typedef _Real ResultType;

		template <typename _Iter1, typename _Iter2> _Real operator() (_Iter1 a, _Iter2 b, size_t size = Kernel::DIM, _Real worst_dist = -golem::numeric_const<_Real>::ONE) const {
			// test vector: a[]
			// kernels: b[] - see: kdtree_single_index.h:604 distance_
			// _RBCoord<_Real>::p: b[0], b[1], b[2]
			// _RBCoord<_Real>::q: b[3], b[4], b[5], b[6]
			// distLin: b[7]
			//return _Real(b[7] * (golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2])));// variable bandwidth
			return _Real((golem::Math::sqr(a[0] - b[0]) + golem::Math::sqr(a[1] - b[1]) + golem::Math::sqr(a[2] - b[2])));
		}
		inline _Real accum_dist(const _Real& a, const _Real& b, int) const {
			return _Real(golem::Math::sqr(a - b));
		}
	};

	struct NNData {
		typedef std::vector<NNData> Seq;

		void resize(size_t size) {
			indices.resize(size);
			distances.resize(size);
		}
		size_t size() const {
			return std::min(indices.size(), distances.size());
		}

		NNSearch::IndexSeq indices;
		std::vector<_Real> distances;

#ifdef _GOLEM_CONTACT_OPTIMISATION_DEBUG
		golem::Real evalDiff;
		int evals;
		int cnt, cntDiff;
		int cntBF, cntNNSearch;
#endif
	};

	typename Kernel::Seq kernels;
	_RealEval distLinMax; // used by NN-search only
	_RealEval weight;

	KDTreeDesc nnSearchDesc;
	mutable typename NNData::Seq nnData;
	typename NNSearch::Ptr nnSearch;

	EvalContactModelCPU() {}

	void create(const Query::Contact3DDesc::TypeDesc& desc, const golem::Query::Pose::Seq& poses, size_t threads = 0) {

		if (poses.size() <= 0)
			throw golem::Message(golem::Message::LEVEL_ERROR, "EvalContactModelCPU::create(): invalid number of kernels %u", poses.size());
		kernels.clear();
		kernels.reserve(poses.size());
		for (golem::Query::Pose::Seq::const_iterator i = poses.begin(); i != poses.end(); ++i)
			kernels.push_back(*i);
		distLinMax = (_RealEval)golem::Math::sqr(desc.second.poseStdDevMax);
		weight = golem::numeric_const<_RealEval>::ONE;// (_RealEval)desc.second.weight;

		if (threads > 0 && desc.second.nnNeighbours > 0) {
			nnSearchDesc = desc.second.nnSearchDesc;
			nnData.resize(threads);
			for (typename NNData::Seq::iterator i = nnData.begin(); i != nnData.end(); ++i) {
				i->resize(desc.second.nnNeighbours);
#ifdef _GOLEM_CONTACT_OPTIMISATION_DEBUG
				i->evalDiff = golem::REAL_ZERO;
				i->evals = 0;
				i->cnt = 0;
				i->cntDiff = 0;
#endif
			}
		}

	}

	inline bool empty() const {
		return kernels.empty();
	}

	inline _RealEval evaluateBF(golem::U32 jobId, const RBCoord& coord) const {
		_RealEval likelihood = golem::numeric_const<_RealEval>::ZERO, c = golem::numeric_const<_RealEval>::ZERO;
		for (const Kernel *i = kernels.data(), *end = i + kernels.size(); i < end; ++i) {
			const Kernel k = *i;
			const _Real dlin = k.p.distanceSqr(coord.p);
			if (dlin < k.distMax.lin) {
				const _Real dang = k.q.distance(coord.q);
				if (dang < k.distMax.ang) {
#ifdef _GOLEM_CONTACT_OPTIMISATION_DEBUG
					++this->nnData[jobId].cntBF;
#endif
					const _Real distance = k.covInv.lin*dlin + k.covInv.ang*dang;
					const _RealEval sampleLikelihood = k.weight*golem::Math::exp(-_RealEval(distance));
					golem::kahanSum(likelihood, c, sampleLikelihood);
				}
			}
		}
		return weight*likelihood;
	}
	_RealEval evaluateNNSearch(golem::U32 jobId, const RBCoord& coord) const;

	inline _RealEval evaluate(golem::U32 jobId, const RBCoord& coord) const {
#ifdef _GOLEM_CONTACT_OPTIMISATION_DEBUG
		NNData& data = this->nnData[jobId];
		data.cntBF = data.cntNNSearch = 0;
		const golem::Real evalBF = evaluateBF(jobId, coord);
		const golem::Real evalNNSearch = evaluateNNSearch(jobId, coord);
		data.evalDiff += evalBF > golem::REAL_EPS ? (evalBF - evalNNSearch)/evalBF : golem::REAL_ZERO;
		++data.evals;
		data.cntDiff += data.cntBF - data.cntNNSearch;
		data.cnt += data.cntBF;
		return _RealEval(evalBF);
#else
		return nnSearch != nullptr ? evaluateNNSearch(jobId, coord) : evaluateBF(jobId, coord);
#endif
	}
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_OPTIMISATIONEVALCPU_H_*/
