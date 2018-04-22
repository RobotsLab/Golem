/** @file Part3DHoPInterop.h
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
#ifndef _GOLEM_DATA_PART_3D_HOP_PART_3D_HOP_INTEROP_H_ // if #pragma once is not supported
#define _GOLEM_DATA_PART_3D_HOP_PART_3D_HOP_INTEROP_H_

//------------------------------------------------------------------------------

#include "GolemInteropDefs.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {
/** Golem part3d name space */
namespace part3d {

	/*************************************************************************
	*
	* Part 3D interface
	*
	**************************************************************************/

	/** Index */
	typedef std::uint64_t Index;
	/** Index set */
	typedef std::set<Index> IndexSet;
	/** Index sequence map */
	typedef std::map<Index, IndexSet> IndexSetMap;

	/** Part 3D realisation. */
	class Part3D : public Sample {
	public:
		/** Sequence */
		typedef std::map<Index, Part3D> Map;

		/** Model */
		Index model;
		/** Frame */
		Mat34 frame;

		/** Default constructor sets the default values. */
		Part3D() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			model = 0;
			frame.clear();
		}
	};

	/** HOP communication defined as request/response service - request types. */
	enum HopReq {
		/** Query training cloud list represented by StringSeq
		*	request: std::uint32_t(HOP_REQ_TRAINING_CLOUD_LIST)
		*	response: std::uint32_t(HOP_RES_XXX), serialised StringSeq
		*/
		HOP_REQ_TRAINING_CLOUD_LIST = 0,

		/** Query part model graph represented by one to many map IndexSetMap (part model -> part sub-models)
		*	request: std::uint32_t(HOP_REQ_PART_MODEL_LIST)
		*	response: std::uint32_t(HOP_RES_XXX), serialised IndexSetMap
		*/
		HOP_REQ_PART_MODEL_LIST = 1,
		
		/** Query trining cloud represented by: cloud (Point3DCloud), part realisation graph (IndexSetMap), part realisations with frames (Part3D::Map), part realisation - point cloud indices mapping (IndexSetMap)
		*	request: std::uint32_t(HOP_REQ_TRAINING_DATA), serialised std::string
		*	response: std::uint32_t(HOP_RES_XXX), serialised Point3DCloud, serialised IndexSetMap, serialised Part3D::Map, serialised IndexSetMap
		*/
		HOP_REQ_TRAINING_DATA = 2,

		/** Query test cloud represented by: cloud (Point3DCloud), part realisation graph (IndexSetMap), part realisations with frames (Part3D::Map), part realisation - point cloud indices mapping (IndexSetMap)
		*	request: std::uint32_t(HOP_REQ_TEST_DATA), serialised Point3DCloud
		*	response: std::uint32_t(HOP_RES_XXX), serialised Point3DCloud, serialised IndexSetMap, serialised Part3D::Map, serialised IndexSetMap
		*/
		HOP_REQ_TEST_DATA = 3,
	};

}; // namespace part3d
}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_DATA_PART_3D_HOP_PART_3D_HOP_INTEROP_H_