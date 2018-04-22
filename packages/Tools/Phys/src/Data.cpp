/** @file Data.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Phys/Data.h>
#include <Golem/Math/Data.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(PhysScene::Physics& desc, XMLContext* context, bool create) {
	XMLData("restitution", desc.restitution, context, create);
	XMLData("static_friction", desc.staticFriction, context, create);
	XMLData("dynamic_friction", desc.dynamicFriction, context, create);
	Vec3 tmp;
	tmp.setColumn3(&desc.nxSceneDesc.gravity.x);
	XMLData(tmp, context->getContextFirst("gravity"), create);
	tmp.getColumn3(&desc.nxSceneDesc.gravity.x);
}

//------------------------------------------------------------------------------
