/** @file ContactQueryInterop.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2018 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_APP_GRASP_GRASP_INTEROP_H_ // if #pragma once is not supported
#define _GOLEM_APP_GRASP_GRASP_INTEROP_H_

//------------------------------------------------------------------------------

#include "GolemInteropContactDefs.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {
	
	/** Communication defined as request/response service - request types. */
	enum GraspInterop {
		/** Input point clouds
		*	request: std::uint32_t(GRASP_INTEROP_INPUT_CLOUD)
		*	response: serialised golem::interop::Point3DCloud::Seq
		*/
		GRASP_INTEROP_INPUT_CLOUD = 0,
		/** Training
		*	request: std::uint32_t(GRASP_INTEROP_TRAINING), serialised golem::interop::Point3DCloud::Seq, serialised golem::interop::Path::Seq
		*	response: serialised golem::interop::Path::Seq
		*/
		GRASP_INTEROP_TRAINING,
		/** Inference
		*	request: std::uint32_t(GRASP_INTEROP_INFERENCE), serialised golem::interop::Point3DCloud::Seq, serialised golem::interop::Path::Seq
		*	response: serialised golem::interop::Path::Seq
		*/
		GRASP_INTEROP_INFERENCE,
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_APP_GRASP_GRASP_INTEROP_H_