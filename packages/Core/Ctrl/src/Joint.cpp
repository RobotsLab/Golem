/** @file Arm.cpp
 * 
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sys/Context.h>
#include <Golem/Ctrl/Joint.h>
#include <Golem/Ctrl/Chain.h>
#include <Golem/Ctrl/Controller.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

Joint::Joint(Chain& chain) : chain(chain), context(chain.getController().getContext()) {
}

void Joint::create(const Desc& desc) {
	if (!desc.isValid())
		throw MsgJointInvalidDesc(Message::LEVEL_CRIT, "Joint::create(): Invalid description");
	
	name = desc.name;
	index = desc.index;
	indexLocal = desc.indexLocal;

	min = desc.min;
	minOffset = desc.min + desc.offset;
	
	max = desc.max;
	maxOffset = desc.max - desc.offset;
	
	offset = desc.offset;

	trnSeq = desc.trnSeq;
	trnInit = desc.trnInit;

	boundsDescSeq.clear();	
	for (Bounds::Desc::Seq::const_iterator i = desc.bounds.begin(); i != desc.bounds.end(); ++i)
		if (*i != NULL)
			boundsDescSeq.push_back((*i)->clone());
}

bool Joint::addBoundsDesc(Bounds::Desc::Ptr pDesc) {
	{
		CriticalSectionWrapper csw(csData);
		boundsDescSeq.push_back(pDesc);
	}

	chain.getController().getCallbackDataSync()->syncJointBoundsDesc(this);

	return true;
}

bool Joint::removeBoundsDesc(const Bounds::Desc* pDesc) {
	Bounds::Desc::Seq::iterator pos = std::find(boundsDescSeq.begin(), boundsDescSeq.end(), pDesc);
	if (pos == boundsDescSeq.end()) {
		context.error("Joint::removeBoundsDesc(): Unable to find bounds description\n");
		return false;
	}
	{
		CriticalSectionWrapper csw(csData);
		boundsDescSeq.erase(pos);
	}

	chain.getController().getCallbackDataSync()->syncJointBoundsDesc(this);

	return true;
}

Bounds::Desc::SeqPtr Joint::getBoundsDescSeq() const {
	CriticalSectionWrapper csw(csData);
	return Bounds::Desc::clone(boundsDescSeq.begin(), boundsDescSeq.end());
}

bool Joint::hasBoundsDesc() const {
	CriticalSectionWrapper csw(csData);
	return !boundsDescSeq.empty();
}

//------------------------------------------------------------------------------
