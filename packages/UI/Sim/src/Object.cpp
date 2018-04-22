/** @file Object.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sim/Object.h>
#include <Golem/Sim/Universe.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Plugin/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void Object::Desc::load(const golem::XMLContext* xmlcontext) {
}

Object::Object(Scene &scene) : scene(scene), universe(scene.getUniverse()), context(scene.getContext()) {
}

void Object::create(const Object::Desc& desc) {
	if (!desc.isValid())
		throw MsgObjectInvalidDesc(Message::LEVEL_CRIT, "Object::create(): Invalid description");
}

Object::~Object() {
}

void Object::release() {
}

//------------------------------------------------------------------------------

void Actor::Desc::load(const golem::XMLContext* xmlcontext) {
	Object::Desc::load(xmlcontext);

	golem::XMLData(pose, xmlcontext->getContextFirst("pose"), false);
	boundsDescSeq.clear();
	golem::XMLData(boundsDescSeq, boundsDescSeq.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "bounds", false);
	golem::XMLData(appearance, xmlcontext->getContextFirst("appearance"), false);
}

Actor::BoundsData::BoundsData() {
	pShapeDesc = pShape = NULL;
}

//------------------------------------------------------------------------------

Actor::Actor(Scene &scene) : Object(scene) {
}

Actor::~Actor() {
}

void Actor::create(const Actor::Desc& desc) {
	Object::create(desc); // throws

	pose = desc.pose;
	appearance = desc.appearance;
	kinematic = desc.kinematic;

	try {
		boundsDataMap.clear();
		boundsConstSeq.clear();

		for (Bounds::Desc::Seq::const_iterator i = desc.boundsDescSeq.begin(); i != desc.boundsDescSeq.end(); ++i)
			(void)addBounds(*i); // throws
	}
	catch (const std::exception& ex) {
		throw MsgActor(Message::LEVEL_CRIT, "Actor::create(): %s", ex.what()); // LEVEL_ERROR ==> LEVEL_CRIT
	}
}

void Actor::release() {
	{
		// main thread
		CriticalSectionWrapper csw(universe.getCS());
		
		boundsConstSeq.clear();
		boundsDataMap.clear();
	}
	Object::release();
}

//------------------------------------------------------------------------------

Actor::BoundsData* Actor::addBounds(Bounds::Desc::Ptr pDesc) {
	if (pDesc == NULL || !pDesc->isValid())
		throw MsgActorBounds(Message::LEVEL_ERROR, "Actor::addBounds(): invalid bounds");

	try {
		BoundsData data;

		data.pBoundsDesc = pDesc;
		data.pBounds = pDesc->create(); // throws

		boundsConstSeq.push_back(data.pBounds.get());
		return &boundsDataMap.insert(boundsDataMap.end(), std::make_pair(data.pBounds.get(), data))->second;
	}
	catch (const std::exception& ex) {
		throw MsgActorBounds(Message::LEVEL_ERROR, "Actor::addBounds(): %s", ex.what());
	}
}

Actor::BoundsData Actor::removeBounds(const Bounds* pBounds) {
	BoundsData::Map::iterator pos = boundsDataMap.find(pBounds);
	if (pos == boundsDataMap.end())
		throw MsgActorBounds(Message::LEVEL_ERROR, "Actor::removeBounds(): unable to find bounds");

	const BoundsData data = pos->second;

	boundsDataMap.erase(pos);
	boundsConstSeq.erase(std::find(boundsConstSeq.begin(), boundsConstSeq.end(), pBounds));// linear search

	return data;
}

//------------------------------------------------------------------------------

Mat34 Actor::getPoseSync() const {
	return pose;
}

void Actor::setPoseSync(const Mat34& pose, bool bMove) {
	this->pose = pose;
}

Mat34 Actor::getPose() const {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	return getPoseSync();
}

void Actor::setPose(const Mat34& pose, bool bMove) {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	setPoseSync(pose, bMove);
}

//------------------------------------------------------------------------------

const Bounds* Actor::createBounds(Bounds::Desc::Ptr pDesc) {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	return addBounds(pDesc)->pBounds.get();
}

void Actor::releaseBounds(const Bounds& bounds) {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	(void)removeBounds(&bounds);
}

Bounds::SeqPtr Actor::getLocalBoundsSeq(U32 group) const {
	Bounds::SeqPtr pBoundsSeq = Bounds::clone(boundsConstSeq.begin(), boundsConstSeq.end(), group);
	return pBoundsSeq;
}

Bounds::SeqPtr Actor::getGlobalBoundsSeq(U32 group) const {
	Bounds::SeqPtr pBoundsSeq;
	pBoundsSeq = Bounds::clone(boundsConstSeq.begin(), boundsConstSeq.end(), group);
	
	const Mat34 pose = getPose();
	Bounds::multiplyPose(pose, pBoundsSeq->begin(), pBoundsSeq->end());
	
	return pBoundsSeq;
}

const Bounds::Desc* Actor::getBoundsDesc(const Bounds& bounds) const {
	BoundsData::Map::const_iterator pos = boundsDataMap.find(&bounds);
	if (pos == boundsDataMap.end())
		throw MsgActorBounds(Message::LEVEL_ERROR, "Actor::getBoundsDesc(): unable to find bounds");

	return pos->second.pBoundsDesc.get();
}

void Actor::setBoundsGroup(const Bounds& bounds, U32 group) {
	BoundsData::Map::iterator pos = boundsDataMap.find(&bounds);
	if (pos == boundsDataMap.end()) {
		context.error("Actor::setBoundsGroup(): Unable to find specified bounds\n");
		return;
	}
	
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	
	pos->second.pBounds->setGroup(group);
}

void Actor::setBoundsGroup(U32 group) {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	
	for (BoundsData::Map::iterator i = boundsDataMap.begin(); i != boundsDataMap.end(); ++i)
		i->second.pBounds->setGroup(group);
}

//------------------------------------------------------------------------------

void Actor::setAppearance(const Appearance &appearance) {
	// main thread
	CriticalSectionWrapper csw(universe.getCS());
	
	this->appearance = appearance;
}

//------------------------------------------------------------------------------

void Actor::render() const {
	if (appearance.invisible)
		return;
	if (boundsConstSeq.empty())
		return;
	
	boundsRenderer.setMat(getPoseSync());
	
	const U32 draw = scene.getDraw();

	if (draw & OpenGL::DRAW_DEFAULT_SOLID) {
		boundsRenderer.setSolidColour(appearance.solidColour);
		boundsRenderer.renderSolid(boundsConstSeq.begin(), boundsConstSeq.end());
	}
	if (draw & OpenGL::DRAW_DEFAULT_WIRE) {
		boundsRenderer.setWireColour(appearance.wireColour);
		boundsRenderer.setLineWidth(appearance.lineWidth);
		boundsRenderer.renderWire(boundsConstSeq.begin(), boundsConstSeq.end());
	}
	if (draw & OpenGL::DRAW_DEFAULT_SHADOW) {
		boundsRenderer.setShadowColour(appearance.shadowColour);
		boundsRenderer.renderShadow(boundsConstSeq.begin(), boundsConstSeq.end());
	}
}

//------------------------------------------------------------------------------
