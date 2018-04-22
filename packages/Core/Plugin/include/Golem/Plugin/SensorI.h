/** @file SensorI.h
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
#ifndef _GOLEM_PLUGIN_SENSORI_H_
#define _GOLEM_PLUGIN_SENSORI_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Sensor.h>
#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/Defs.h>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Captures snapshot or sequence from sensor.
*	(Optionally) Implemented by Handler.
*/
class Capture {
public:
	/** Sensor control with sync start and stop condition. */
	typedef std::function<bool(const golem::TimeStamp*)> Control;

	/** Captures snapshot or sequence from sensor. */
	virtual Item::Ptr capture(golem::Sensor& sensor, Control control) = 0;
};

/** Estimates model pose.
*	(Optionally) Implemented by Item.
*/
class Model {
public:
	/** Estimates model pose. */
	virtual void model(golem::ConfigMat34& config, golem::Mat34& frame, Vec3Seq* vertices = nullptr, TriangleSeq* triangles = nullptr) = 0;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_SENSORI_H_*/
