/** @file Search.h
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
#ifndef _GOLEM_TOOLS_SEARCH_H_
#define _GOLEM_TOOLS_SEARCH_H_

//------------------------------------------------------------------------------

#include <Golem/Tools/Defs.h>
#ifdef __APPLE__
#include <flann/flann.hpp>
#endif

//------------------------------------------------------------------------------

namespace flann {
	template <typename Distance> class Index;
	template <typename T> class Matrix;
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class XMLContext;

//------------------------------------------------------------------------------

/** Nearest neighbour search base */
class NNSearch {
public:
	/** Pointer */
	typedef golem::shared_ptr<NNSearch> Ptr;

	/** Indices */
	typedef std::vector<int> IndexSeq;
	/** Distances */
	typedef std::vector<golem::F32> DistanceF32Seq;
	/** Distances */
	typedef std::vector<golem::F64> DistanceF64Seq;

	/** Virtual descructor */
	virtual ~NNSearch() {}

	/** knnSearch: generic data, F32 */
	virtual int knnSearch(const golem::F32* data, size_t size, size_t stride, size_t neighbours, int* indices, golem::F32* distances) const {throw golem::Message(golem::Message::LEVEL_CRIT, "NNSearch::knnSearch(): not implemented!");}
	/** knnSearch: generic data, F64 */
	virtual int knnSearch(const golem::F64* data, size_t size, size_t stride, size_t neighbours, int* indices, golem::F64* distances) const {throw golem::Message(golem::Message::LEVEL_CRIT, "NNSearch::knnSearch(): not implemented!");}
	
	/** knnSearch: generic data */
	template <typename _Data, typename _Real> int knnSearch(const std::vector<_Data>& data, size_t neighbours, int* indices, _Real* distances) const {
		return knnSearch(reinterpret_cast<const _Real*>(&data.front()), data.size(), sizeof(_Data), neighbours, indices, distances);
	}
	/** knnSearch: vector */
	template <typename _Data, typename _Real> int knnSearch(const std::vector<_Data>& data, size_t neighbours, IndexSeq& indices, std::vector<_Real>& distances) const {
		allocate(data.size(), neighbours, indices, distances);
		return knnSearch(reinterpret_cast<const _Real*>(&data.front()), data.size(), sizeof(_Data), neighbours, indices.data(), distances.data());
	}
	/** knnSearch: single element */
	template <typename _Data, typename _Real> int knnSearch(const _Data& data, size_t neighbours, IndexSeq& indices, std::vector<_Real>& distances) const {
		allocate(1, neighbours, indices, distances);
		return knnSearch(reinterpret_cast<const _Real*>(&data), 1, sizeof(_Data), neighbours, indices.data(), distances.data());
	}
	/** knnSearch: single element, one neighbour */
	template <typename _Data, typename _Real> int knnSearch(const _Data& data, int& index, _Real& distance) const {
		return knnSearch(reinterpret_cast<const _Real*>(&data), 1, sizeof(_Data), 1, &index, &distance);
	}

	/** radiusSearch: generic data, F32 */
	virtual int radiusSearch(const golem::F32* data, size_t size, size_t stride, size_t neighbours, golem::F32 radius, int* indices, golem::F32* distances) const {throw golem::Message(golem::Message::LEVEL_CRIT, "NNSearch::radiusSearch(): not implemented!");}
	/** radiusSearch: generic data, F64 */
	virtual int radiusSearch(const golem::F64* data, size_t size, size_t stride, size_t neighbours, golem::F64 radius, int* indices, golem::F64* distances) const {throw golem::Message(golem::Message::LEVEL_CRIT, "NNSearch::radiusSearch(): not implemented!");}
	
	/** radiusSearch: generic data */
	template <typename _Data, typename _Real> int radiusSearch(const std::vector<_Data>& data, size_t neighbours, _Real radius, int* indices, _Real* distances) const {
		return radiusSearch(reinterpret_cast<const _Real*>(&data.front()), data.size(), sizeof(_Data), neighbours, radius, indices, distances);
	}
	/** radiusSearch: vector */
	template <typename _Data, typename _Real> int radiusSearch(const std::vector<_Data>& data, size_t neighbours, _Real radius, IndexSeq& indices, std::vector<_Real>& distances) const {
		allocate(data.size(), neighbours, indices, distances);
		return radiusSearch(reinterpret_cast<const _Real*>(&data.front()), data.size(), sizeof(_Data), neighbours, radius, indices.data(), distances.data());
	}
	/** radiusSearch: single element */
	template <typename _Data, typename _Real> int radiusSearch(const _Data& data, size_t neighbours, _Real radius, IndexSeq& indices, std::vector<_Real>& distances) const {
		allocate(1, neighbours, indices, distances);
		return radiusSearch(reinterpret_cast<const _Real*>(&data), 1, sizeof(_Data), neighbours, radius, indices.data(), distances.data());
	}
	/** radiusSearch: single element, one neighbour */
	template <typename _Data, typename _Real> int radiusSearch(const _Data& data, _Real radius, int& index, _Real& distance) const {
		return radiusSearch(reinterpret_cast<const _Real*>(&data), 1, sizeof(_Data), 1, radius, &index, &distance);
	}

	/** Allocate */
	template <typename _Data> static void allocate(size_t size, size_t neighbours, IndexSeq& indices, std::vector<_Data>& distances) {
		indices.resize(size*neighbours);
		distances.resize(size*neighbours);
	}
};

//------------------------------------------------------------------------------

/** KDTree description */
class KDTreeDesc {
public:
	/** Maximum leafs to visit when searching for neighbouring features */
	golem::U32 searchChecks;
	/** Number of randomized kd-trees which will be searched in parallel */
	golem::U32 searchKDTrees;
	/** The maximum number of points to have in a leaf for not branching the tree any more */
	golem::U32 searchLeafMaxSize;
	/** The branching factor to use for the hierarchical k-means tree */
	golem::U32 searchBranching;
	/** The maximum number of iterations to use in the k-means clustering */
	golem::U32 searchIterations;

	KDTreeDesc() {
		setToDefault();
	}
	/** Sets the parameters to the default values */
	void setToDefault() {
		searchChecks = 32;
		searchKDTrees = 4;
		searchLeafMaxSize = 10;
		searchBranching = 32;
		searchIterations = 11;
	}
	/** Assert that the object is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(searchChecks > 0, ac, "searchChecks: must be greater than 0");
		Assert::valid(searchKDTrees > 0, ac, "searchKDTrees: must be greater than 0");
		Assert::valid(searchLeafMaxSize > 0, ac, "searchLeafMaxSize: must be greater than 0");
		Assert::valid(searchBranching > 0, ac, "searchBranching: must be greater than 0");
		Assert::valid(searchIterations > 0, ac, "searchIterations: must be greater than 0");
	}

	/** flann::SearchParams */
	template <typename _SearchParams> void getSearchParams(_SearchParams& search) const {
		search.checks = (int)searchChecks;
		search.sorted = false;
		search.cores = 1;
	}
	/** flann::KDTreeIndex */
	template <typename _SearchParams, typename _KDTreeIndexParams> void getKDTreeIndex(_SearchParams& search, _KDTreeIndexParams& index) const {
		getSearchParams(search);
		index = _KDTreeIndexParams((int)searchKDTrees);
	}
	/** flann::KDTreeSingleIndex */
	template <typename _SearchParams, typename _KDTreeSingleIndexParams> void getKDTreeSingleIndex(_SearchParams& search, _KDTreeSingleIndexParams& index) const {
		getSearchParams(search);
		index = _KDTreeSingleIndexParams((int)searchLeafMaxSize);
	}
	/** flann::KMeansIndex */
	template <typename _SearchParams, typename _KMeansIndexParams> void getKMeansIndex(_SearchParams& search, _KMeansIndexParams& index) const {
		getSearchParams(search);
		index = _KMeansIndexParams((int)searchBranching, (int)searchIterations);
	}
};

/** Flann wrapper */
template <typename _Real, typename _Dist, typename _SearchParams> class KDTree : public NNSearch, public flann::Index<_Dist> {
public:
	/** Distances */
	typedef std::vector<_Real> DistanceSeq;

	/** Construct kd-trees: generic data */
	template <typename _IndexParams> KDTree(const _SearchParams& searchParams, const _IndexParams& indexParams, const _Real* data, size_t size, size_t stride, size_t dim, _Dist dist = _Dist()) :
		flann::Index<_Dist>(flann::Matrix<_Real>(const_cast<_Real*>(data), size, dim, stride), indexParams, dist), searchParams(searchParams), dim(dim)
	{
		flann::Index<_Dist>::buildIndex();
	}
	/** Construct kd-trees: vector */
	template <typename _Data, typename _IndexParams> KDTree(const _SearchParams& searchParams, const _IndexParams& indexParams, const std::vector<_Data>& data, size_t dim, _Dist dist = _Dist()) :
		flann::Index<_Dist>(flann::Matrix<_Real>(reinterpret_cast<_Real*>(const_cast<_Data*>(&data.front())), data.size(), dim, sizeof(_Data)), indexParams, dist), searchParams(searchParams), dim(dim)
	{
		flann::Index<_Dist>::buildIndex();
	}

	/** knnSearch */
	virtual int knnSearch(const _Real* data, size_t size, size_t stride, size_t neighbours, int* indices, _Real* distances) const {
		// prepare data
		flann::Matrix<int> flannIndices(indices, size, neighbours);
		flann::Matrix<_Real> flannDistances(distances, size, neighbours);
		// knn search
		return ((flann::Index<_Dist>*)const_cast<KDTree*>(this))->knnSearch(flann::Matrix<_Real>(const_cast<_Real*>(data), size, dim, stride), flannIndices, flannDistances, neighbours, searchParams);
	}

	/** knnSearch */
	virtual int radiusSearch(const _Real* data, size_t size, size_t stride, size_t neighbours, _Real radius, int* indices, _Real* distances) const {
		// prepare data
		flann::Matrix<int> flannIndices(indices, size, neighbours);
		flann::Matrix<_Real> flannDistances(distances, size, neighbours);
		// knn search
		return ((flann::Index<_Dist>*)const_cast<KDTree*>(this))->radiusSearch(flann::Matrix<_Real>(const_cast<_Real*>(data), size, dim, stride), flannIndices, flannDistances, (float)radius, searchParams);
	}

private:
	/** Search params */
	const _SearchParams searchParams;
	/** Dimensions */
	const  size_t dim;
};

void XMLData(KDTreeDesc& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------


#endif /*_GOLEM_TOOLS_SEARCH_H_*/
