/** @file CollisionBounds.h
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
#ifndef _GOLEM_TOOLS_CTRL_H_
#define _GOLEM_TOOLS_CTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Planner/Planner.h>
#include <Golem/Plugin/Renderer.h>
#include <Golem/Tools/Cluster.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Collision detection */
class CollisionBounds : public golem::Planner::CallbackDataSync {
public:
	typedef golem::shared_ptr<CollisionBounds> Ptr;

	/** create */
	CollisionBounds(golem::Planner& planner, const golem::Bounds::Seq& bounds, golem::DebugRenderer* renderer = nullptr, golem::CriticalSection* cs = nullptr, const golem::RGBA& colourSolid = golem::RGBA(0, 0, 255, 64), const golem::RGBA& colourWire = golem::RGBA(0, 0, 255, 255));
	
	/** collision testing */
	bool collides(const golem::ConfigspaceCoord& c, golem::Configspace::Index joint);
	/** local bounds collision test only */
	void setLocal(bool local = true);

	/** Point cloud bounding box */
	template <typename _Points> static golem::BoundingBox::Desc getBoundingBox(_Points points) {
		golem::BoundingBox::Desc boundingBoxDesc;
		boundingBoxDesc.pose.R = orientation(covariance(points));
		golem::Vec3 v[3];
		for (size_t j = 0; j < 3; ++j) {
			v[j].setZero();
			v[j][j] = golem::REAL_ONE;
			v[j] = boundingBoxDesc.pose.R * v[j];
		}
		golem::Vec3 point, min(golem::REAL_MAX), max(-golem::REAL_MAX);
		for (size_t i = 0; points(i, point); ++i) {
			for (size_t j = 0; j < 3; ++j) {
				const golem::Real proj = point.dot(v[j]);
				min[j] = std::min(min[j], proj);
				max[j] = std::max(max[j], proj);
			}
		}
		for (size_t j = 0; j < 3; ++j) {
			boundingBoxDesc.dimensions[j] = golem::REAL_HALF*(max[j] - min[j]);
			boundingBoxDesc.pose.p[j] = min[j] + boundingBoxDesc.dimensions[j];
		}
		boundingBoxDesc.pose.p = boundingBoxDesc.pose.R * boundingBoxDesc.pose.p;
		return boundingBoxDesc;
	}

	/** Point cloud bounding boxes from clusters */
	template <typename _Points> static golem::Bounds::Seq getBounds(_Points points, const data::Cluster3D::IndexSeq& indices, const data::Cluster3D::IndexSeq& clusters, const data::Cluster3D::IndexMap& selection, const std::string& name = data::Cluster3D::CLUSTER_COLLISIONS) {
		golem::Bounds::Seq bounds;

		data::Cluster3D::IndexMap::const_iterator k = selection.find(name);
		if (k == selection.end())
			bounds.push_back(getBoundingBox(points).create());
		else {
			for (data::Cluster3D::IndexSet::const_iterator j = k->second.begin(); j != k->second.end(); ++j) {
				data::Cluster3D::IndexSeq seq;
				data::Cluster3D::getIndices(indices, clusters, *j, seq);
				bounds.push_back(getBoundingBox([=] (size_t i, golem::Vec3& p) -> bool { (void)points(seq[i], p); return i < seq.size(); }).create());
			}
		}

		return bounds;
	}

	/** Point cloud bounding boxes from interfaces */
	template <typename _Points> static golem::Bounds::Seq getBounds(_Points points, const data::Cluster3D* clusters3D = nullptr, const std::string& name = data::Cluster3D::CLUSTER_COLLISIONS) {
		golem::Bounds::Seq bounds;
		
		if (clusters3D) {
			data::Cluster3D::IndexSeq indices;
			data::Cluster3D::IndexSeq clusters;
			data::Cluster3D::IndexMap selection;
			clusters3D->getClusters(indices, clusters);
			clusters3D->getClustersSelection(selection);
			bounds = getBounds(points, indices, clusters, selection, name);
		}
		else
			bounds.push_back(getBoundingBox(points).create());

		return bounds;
	}

	/** golem::Planner::CallbackDataSync */
	virtual void syncCollisionBounds();
	/** golem::Planner::CallbackDataSync */
	virtual void syncFindTrajectory(golem::Controller::Trajectory::const_iterator begin, golem::Controller::Trajectory::const_iterator end, const golem::GenWorkspaceChainState* wend = NULL);

	/** uninstall callback and reset rendering */
	virtual ~CollisionBounds();

protected:
	golem::Planner& planner;
	golem::Planner::CallbackDataSync* pCallback;
	golem::Bounds::Seq bounds;

	golem::DebugRenderer* renderer;
	golem::CriticalSection* cs;
	golem::RGBA colourSolid, colourWire;

	bool local;

	/** Covariance */
	template <typename _Points> static golem::Mat33 covariance(const _Points& points) {
		golem::Vec3 point;
		size_t size = 0;
		// mean
		golem::Vec3 mean = golem::Vec3::zero();
		for (size_t i = 0; points(i, point); ++i, ++size)
			mean += point;
		// norm
		const golem::Real norm = golem::REAL_ONE / golem::Real(size);
		mean *= norm;
		// covariance
		golem::Mat33 cov = golem::Mat33::zero();
		for (size_t i = 0; points(i, point); ++i) {
			const golem::Vec3 d = point - mean;
			cov += golem::Mat33(d.x*d.x, d.x*d.y, d.x*d.z, d.y*d.x, d.y*d.y, d.y*d.z, d.z*d.x, d.z*d.y, d.z*d.z);
		}
		return cov * norm;
	}
	/** Orientation */
	static golem::Mat33 orientation(const golem::Mat33& cov);

	/** add bounds */
	void addCollisionBounds();
	/** Draw bounds */
	void draw(const golem::Heuristic::BoundsSet* jointBoundsSet = nullptr, const golem::Mat34* frame = nullptr);
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_TOOLS_CTRL_H_*/
