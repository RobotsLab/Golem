/** @file Cluster.cpp
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

//------------------------------------------------------------------------------

#include <Golem/Tools/Cluster.h>
#include <Golem/Tools/Import.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::ClusteringDesc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("enabled", affinityEnabled, xmlcontext->getContextFirst("affinity"), false);
	golem::XMLData("steps", affinitySteps, xmlcontext->getContextFirst("affinity"), false);
	golem::XMLData("convergence_steps", affinityConvergenceSteps, xmlcontext->getContextFirst("affinity"), false);
	golem::XMLData("convergence_cycles", affinityConvergenceCycles, xmlcontext->getContextFirst("affinity"), false);
	golem::XMLData("lambda", affinityLambda, xmlcontext->getContextFirst("affinity"), false);
	golem::XMLData("preference_gain", affinityPreferenceGain, xmlcontext->getContextFirst("affinity"), false);
	golem::XMLData("preference_offset", affinityPreferenceOffset, xmlcontext->getContextFirst("affinity"), false);

	golem::XMLData("enabled", pamEnabled, xmlcontext->getContextFirst("pam"), false);
	golem::XMLData("steps", pamSteps, xmlcontext->getContextFirst("pam"), false);
	golem::XMLData("clusters_min", pamClustersMin, xmlcontext->getContextFirst("pam"), false);
	golem::XMLData("clusters_max", pamClustersMax, xmlcontext->getContextFirst("pam"), false);
	golem::XMLData("ftest", pamFTest, xmlcontext->getContextFirst("pam"), false);

	golem::XMLData("enabled", opticsEnabled, xmlcontext->getContextFirst("optics"), false);
	golem::XMLData("radius", opticsRadius, xmlcontext->getContextFirst("optics"), false);
	golem::XMLData("density", opticsDensity, xmlcontext->getContextFirst("optics"), false);
}

//------------------------------------------------------------------------------

const std::string golem::data::Cluster3D::CLUSTER_PROCESSING = "processing";
const std::string golem::data::Cluster3D::CLUSTER_COLLISIONS = "collisions";

const std::string golem::data::Cluster3D::FILE_EXT_CLUSTER = ".cluster";

//------------------------------------------------------------------------------

void golem::XMLData(data::Cluster3D::IndexMap::value_type &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData("name", const_cast<std::string&>(val.first), xmlcontext, create);
	std::string clusters;
	if (create && !val.second.empty()) {
		for (data::Cluster3D::IndexSet::const_iterator i = val.second.begin();;) {
			clusters.append(std::to_string(*i));
			if (++i == val.second.end()) break;
			clusters.append(",");
		}
	}
	golem::XMLData("clusters", clusters, xmlcontext, create);
	if (!create) {
		val.second.clear();
		std::stringstream sstream(clusters + " "); // HACK std::stringstream::read(&c, 1) reads one character too much without reporting eof
		IStream istream(sstream, "\t,; ");
		while (!istream.eos()) {
			const std::string cluster(istream.next<const char*>());
			std::istringstream iss(cluster, std::istringstream::in);
			data::Cluster3D::Index index;
			iss >> index;
			if (iss.fail())
				throw MsgXMLParser(Message::LEVEL_ERROR, "golem::XMLData(data::Cluster3D::IndexMap): Value parse error at: %s", cluster.c_str());
			val.second.insert(index);
		}
	}
}

//------------------------------------------------------------------------------

template <> void golem::Stream::read(golem::data::Cluster3D::IndexMap::value_type& value) const {
	read(const_cast<std::string&>(value.first));
	read(value.second, value.second.begin());
}
template <> void golem::Stream::write(const golem::data::Cluster3D::IndexMap::value_type& value) {
	write(value.first);
	write(value.second.begin(), value.second.end());
}

//------------------------------------------------------------------------------

const std::string golem::data::Region3D::FILE_EXT_REGION = ".region";

//------------------------------------------------------------------------------

