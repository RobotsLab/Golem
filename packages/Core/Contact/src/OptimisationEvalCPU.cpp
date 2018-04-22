/** @file OptimisationEvalCPU.cpp
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

//------------------------------------------------------------------------------

#include <Golem/Contact/OptimisationEvalCPU.h>
#include <Golem/Sys/XMLData.h>
#ifdef WIN32
#pragma warning (push)
#pragma warning (disable : 4291 4244 4996 4305 4267)
#endif
#include <flann/flann.hpp>
#ifdef WIN32
#pragma warning (pop)
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

namespace golem {
	template <> golem::F32 EvalContactModelCPU<F32, F32>::evaluateNNSearch(golem::U32 jobId, const RBCoord& coord) const {
		typedef EvalConfigModelCPU<golem::F32, golem::F32> ConfigModel;
		typedef EvalContactModelCPU<golem::F32, golem::F32> ContactModel;

		// KD-tree based nn search
		NNData& data = nnData[jobId];
		F32 likelihood = numeric_const<F32>::ZERO, c = numeric_const<F32>::ZERO;

		//nnSearch->radiusSearch(coord.data(), 1, sizeof(ContactModel::Kernel), data.indices.size(), distMax.lin, data.indices.data(), data.distances.data());// variable bandwidth
		nnSearch->radiusSearch(coord.data(), 1, sizeof(ContactModel::Kernel), data.indices.size(), distLinMax/kernels[0].covInv.lin, data.indices.data(), data.distances.data());
		for (size_t i = 0; i < data.indices.size(); ++i) {
			const int index = data.indices[i];
			if (index < 0)
				break;
			ContactModel::Kernel kernel = kernels[index];
			const F32 dang = kernel.q.distance(coord.q);
			if (dang < kernel.distMax.ang) {
#ifdef _GOLEM_CONTACT_OPTIMISATION_DEBUG
				++data.cntNNSearch;
#endif
				// data.distances[i] is already multiplied by kernel.distLin
				//const F32 distance = data.distances[i] + kernel.distAng*dang;// variable bandwidth
				const F32 distance = kernel.covInv.lin*data.distances[i] + kernel.covInv.ang*dang;
				const F32 sampleLikelihood = kernels[index].weight*golem::Math::exp(-distance);
				golem::kahanSum(likelihood, c, sampleLikelihood);
			}
		}

		return weight*likelihood;
	}
};

//------------------------------------------------------------------------------
