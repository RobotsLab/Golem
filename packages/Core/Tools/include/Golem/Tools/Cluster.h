/** @file Cluster.h
 * 
 * Clustering
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
#ifndef _GOLEM_TOOLS_CLUSTER_H_
#define _GOLEM_TOOLS_CLUSTER_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/Sys/Stream.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Clustering description */
class ClusteringDesc {
public:
	/** affinity propagation clustering enabled */
	bool affinityEnabled;
	/** affinity propagation clustering steps */
	golem::U32 affinitySteps;
	/** affinity propagation clustering convergence steps */
	golem::U32 affinityConvergenceSteps;
	/** affinity propagation clustering convergence cycles */
	golem::U32 affinityConvergenceCycles;
	/** affinity propagation clustering damping factor */
	golem::Real affinityLambda;
	/** affinity propagation clustering preference gain */
	golem::Real affinityPreferenceGain;
	/** affinity propagation clustering preference offset */
	golem::Real affinityPreferenceOffset;

	/** PAM clustering enabled */
	bool pamEnabled;
	/** PAM clustering steps */
	golem::U32 pamSteps;
	/** PAM clustering max clusters */
	golem::Real pamClustersMin;
	/** PAM clustering min clusters */
	golem::Real pamClustersMax;
	/** PAM clustering F-test confidence */
	golem::Real pamFTest;

	/** PAM clustering enabled */
	bool opticsEnabled;
	/** Clustering search distance radius */
	golem::Real opticsRadius;
	/** Clustering min neighbours */
	golem::U32 opticsDensity;

	/** Constructs description object */
	ClusteringDesc() {
		ClusteringDesc::setToDefault();
	}
	/** Sets the parameters to the default values. */
	void setToDefault() {
		affinityEnabled = true;
		affinitySteps = 1000000;
		affinityLambda = golem::Real(0.999);
		affinityConvergenceSteps = 1000;
		affinityConvergenceCycles = 10;
		affinityPreferenceGain = golem::Real(1.0);
		affinityPreferenceOffset = golem::Real(0.0);

		pamEnabled = false;
		pamSteps = 1000000;
		pamClustersMin = golem::Real(0.05);
		pamClustersMax = golem::Real(0.5);
		pamFTest = golem::Real(0.95);

		opticsEnabled = false;
		opticsRadius = golem::Real(1.0);
		opticsDensity = 5;
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		Assert::valid(affinitySteps > 0, ac, "affinitySteps: 0");
		Assert::valid(affinityLambda >= golem::REAL_ZERO && affinityLambda <= golem::REAL_ONE, ac, "affinityLambda: not in [0, 1]");
		Assert::valid(affinityConvergenceSteps > 0, ac, "affinityConvergenceSteps: <= 0");
		Assert::valid(affinityConvergenceCycles > 0, ac, "affinityConvergenceCycles: <= 0");
		Assert::valid(Math::isFinite(affinityPreferenceGain), ac, "affinityPreferenceGain: invalid");
		Assert::valid(Math::isFinite(affinityPreferenceOffset), ac, "affinityPreferenceGain: invalid");

		Assert::valid(pamSteps > 0, ac, "pamSteps: 0");
		Assert::valid(pamClustersMin >= golem::REAL_ZERO && pamClustersMin <= golem::REAL_ONE, ac, "pamClustersMin: not in [0, 1]");
		Assert::valid(pamClustersMax >= golem::REAL_ZERO && pamClustersMax <= golem::REAL_ONE, ac, "pamClustersMax: not in [0, 1]");
		Assert::valid(pamFTest >= golem::REAL_ZERO && pamFTest <= golem::REAL_ONE, ac, "pamFTest: not in [0, 1]");

		Assert::valid(opticsRadius > golem::REAL_EPS, ac, "opticsRadius: < eps");
		Assert::valid(opticsDensity > 0, ac, "opticsDensity: < 1");
	}
	/** Load descritpion from xml context. */
	void load(const golem::XMLContext* xmlcontext);
};

//------------------------------------------------------------------------------

namespace data {

/** Cluster 3D interface.
*/
class Cluster3D {
public:
	/** Index */
	typedef golem::U32 Index;
	/** Index sequence */
	typedef std::vector<Index> IndexSeq;
	/** Index set */
	typedef std::set<Index> IndexSet;
	/** Selection map */
	typedef std::map<std::string, IndexSet> IndexMap;

	/** Cluster set name: processing */
	static const std::string CLUSTER_PROCESSING;
	/** Cluster set name: collision */
	static const std::string CLUSTER_COLLISIONS;

	/** File extension: cloud clusters */
	static const std::string FILE_EXT_CLUSTER;

	/** Cluster3D: Indices sorted along clusters, clusters indices pointers. */
	virtual void getClusters(IndexSeq& indices, IndexSeq& clusters) const = 0;
	/** Cluster3D: Cluster selection. */
	virtual void getClustersSelection(IndexMap& selection) const = 0;

	/** Select indices */
	template <typename _Set> static void getIndices(const IndexSeq& indices, const IndexSeq& clusters, Index selection, _Set& set) {
		if (selection < (Index)clusters.size()) {
			const Index begin = clusters[selection];
			const Index end = selection + 1 < (Index)clusters.size() ? clusters[selection + 1] : (Index)indices.size();
			for (Index i = begin; i < end; ++i)
				set.insert(set.end(), indices[i]);
		}
	}
	/** Select indices */
	template <typename _Set> static void getIndices(const std::string& name, const IndexSeq& indices, const IndexSeq& clusters, const IndexMap& selection, _Set& set) {
		const IndexMap::const_iterator i = selection.find(name);
		if (i == selection.end())
			return;
		for (IndexSet::const_iterator j = i->second.begin(); j != i->second.end(); ++j)
			getIndices(indices, clusters, *j, set);
	}
	/** Select indices */
	template <typename _Cluster3D, typename _Set> static void getIndices(const std::string& name, const _Cluster3D* _cluster3D, _Set& set) {
		const Cluster3D* cluster3D = is<const Cluster3D>(_cluster3D);
		if (!cluster3D)
			return;
		
		IndexSeq indices;
		IndexSeq clusters;
		cluster3D->getClusters(indices, clusters);
		IndexMap selection;
		cluster3D->getClustersSelection(selection);
		
		getIndices(name, indices, clusters, selection, set);
	}
};

/** Region 3D interface.
*/
class Region3D {
public:
	/** Index */
	typedef golem::U32 Index;
	/** Index sequence */
	typedef std::vector<Index> IndexSeq;

	/** File extension: cloud clusters */
	static const std::string FILE_EXT_REGION;

	/** Cluster3D: Indices sorted along clusters, clusters indices pointers. */
	virtual void getRegions(IndexSeq& indices, IndexSeq& clusters) const = 0;
};
};	// namespace

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

namespace golem {

/** Reads/writes object from/to a given XML context */
void XMLData(golem::data::Cluster3D::IndexMap::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

template <> void Stream::read(golem::data::Cluster3D::IndexMap::value_type& value) const;
template <> void Stream::write(const golem::data::Cluster3D::IndexMap::value_type& value);

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_TOOLS_CLUSTER_H_*/
