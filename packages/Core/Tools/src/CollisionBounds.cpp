/** @file CollisionBounds.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#ifndef EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#endif // EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Eigenvalues>
#include <Golem/Tools/CollisionBounds.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

CollisionBounds::CollisionBounds(golem::Planner& planner, const golem::Bounds::Seq& bounds, golem::DebugRenderer* renderer, golem::CriticalSection* cs, const golem::RGBA& colourSolid, const golem::RGBA& colourWire) :
	planner(planner), bounds(bounds), pCallback(planner.getCallbackDataSync()), renderer(renderer), cs(cs), colourSolid(colourSolid), colourWire(colourWire), local(false)
{
	//if (bounds.empty())
	//	throw golem::Message(golem::Message::LEVEL_ERROR, "CollisionBounds(): empty collision bounds");
	for (golem::Bounds::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
		if (*i == nullptr)
			throw golem::Message(golem::Message::LEVEL_ERROR, "CollisionBounds(): invalid collision bounds");
	// install callback
	planner.setCallbackDataSync(this);
	// draw
	draw();
}

CollisionBounds::~CollisionBounds() {
	// restore old callback
	planner.setCallbackDataSync(pCallback);
	// clear bounds
	if (renderer && cs) {
		golem::CriticalSectionWrapper csw(*cs);
		renderer->reset();
	}
}

void CollisionBounds::addCollisionBounds() {
	for (golem::Bounds::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i)
		planner.getHeuristic().addCollisionBounds(**i);
};

void CollisionBounds::syncCollisionBounds() {
	// sync bounds
	pCallback->syncCollisionBounds();
	if (!local) addCollisionBounds();
};

void CollisionBounds::syncFindTrajectory(golem::Controller::Trajectory::const_iterator begin, golem::Controller::Trajectory::const_iterator end, const golem::GenWorkspaceChainState* wend) {
	pCallback->syncFindTrajectory(begin, end, wend);
};

bool CollisionBounds::collides(const golem::ConfigspaceCoord& c, golem::Configspace::Index joint) {
	// sync bounds
	pCallback->syncCollisionBounds();
	addCollisionBounds();
	
	// display bounds
	const golem::Waypoint waypoint(planner.getController(), c);
	draw(&planner.getHeuristic().getJointBounds()[joint], &waypoint.wposex[joint]);
	
	// compute collisions
	return !bounds.empty() && planner.getHeuristic().collides(waypoint);
}

void CollisionBounds::setLocal(bool local) {
	this->local = local;
}

void CollisionBounds::draw(const golem::Heuristic::BoundsSet* jointBoundsSet, const golem::Mat34* frame) {
	// display bounds
	if (renderer && cs) {
		golem::CriticalSectionWrapper csw(*cs);

		renderer->reset();
		for (golem::Bounds::Seq::const_iterator i = bounds.begin(); i != bounds.end(); ++i) {
			renderer->setColour(colourSolid);
			renderer->addSolid(**i);
			renderer->setColour(colourWire);
			renderer->addWire(**i);
		}
		
		if (jointBoundsSet && frame)
			for (golem::Heuristic::BoundsSet::const_iterator i = jointBoundsSet->begin(); i != jointBoundsSet->end(); ++i) {
				golem::Bounds::Ptr jointBounds = (*i)->create();
				jointBounds->multiplyPose(*frame, jointBounds->getPose());
				renderer->setColour(colourWire);
				renderer->addWire(*jointBounds);
			}
	}
}

//------------------------------------------------------------------------------

golem::Mat33 golem::CollisionBounds::orientation(const golem::Mat33& cov) {
	typedef Eigen::Matrix<golem::Real, 3, 3> EigenMat33;
	EigenMat33 ecov;
	for (size_t i = 0; i < 3; ++i)
		for (size_t j = 0; j < 3; ++j)
			ecov(i, j) = cov.m[i][j];
	Eigen::EigenSolver<EigenMat33> solver(ecov);

	typedef std::pair<golem::Real, golem::Vec3> EigenPair;
	EigenPair epairs[3];
	for (size_t i = 0; i < 3; ++i) {
		epairs[i] = std::make_pair(solver.pseudoEigenvalueMatrix().diagonal()(i,i), Vec3(solver.pseudoEigenvectors()(0, i), solver.pseudoEigenvectors()(1, i), solver.pseudoEigenvectors()(2, i)));
		epairs[i].second.normalise();
		//printf("Eigen: e=%f, l=%f -> (%f, %f, %f), (%f, %f, %f)\n", epairs[i].first, epairs[i].second.magnitude(), epairs[i].second.x, epairs[i].second.y, epairs[i].second.z, epairs[i].second.dot(epairs[0].second), epairs[i].second.dot(epairs[1].second), epairs[i].second.dot(epairs[2].second));
	}
	std::sort(epairs, epairs + 3, [] (const EigenPair& l, const EigenPair& r) -> bool { return l.first > r.first; });

	// for stability in case of thin surfaces - do not use last eigen vector, but rather cross product of the first two
	return Mat33(epairs[0].second, epairs[1].second, epairs[0].second.cross(epairs[1].second));
}

//------------------------------------------------------------------------------
