/** @file GolemInteropContactInterface.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_CONTACT_CONTACT_INTERFACE_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_CONTACT_CONTACT_INTERFACE_H_

//------------------------------------------------------------------------------

#include "GolemInteropInterface.h"
#include "GolemInteropContactDefs.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Contact interface
	*
	**************************************************************************/

	/** Contact interface */
	class Contact : virtual public Interface {
	public:
		/** Training and test: Process a sequence of point clouds
		*	@param[in]	inp				point cloud sequence to be processed
		*	@param[out]	out				set of features
		*/
		virtual void findFeatures(const Point3DCloud::Seq& inp, Feature3D::Seq& out) = 0;

		/** Training: Creates contact models from training data - a set of trajectories and point cloud features
		*	@param[in]	training		training data
		*	@param[out]	models			contact and configuration models
		*/
		virtual void findModel(const Training3D::Map& training, Model3D::Map& models) = 0;

		/** Test: Finds path hypotheses from contact models and point cloud features
		*	@param[in]	models			contact and configuration models
		*	@param[in]	features		point cloud features
		*	@param[out]	query			query data
		*/
		virtual void findQuery(const Model3D::Map& models, const Feature3D::Seq& features, Query& query) = 0;

		/** Test: Finds path hypotheses from default contact models and point cloud features
		*	@param[in]	features		point cloud features
		*	@param[out]	query			query data
		*/
		virtual void findQuery(const Feature3D::Seq& features, Query& query) = 0;

		/** Selection: Selects most likely path hypothesis and computes collision-free trajectory
		*	@param[in]	query			query data
		*	@param[out]	trajectory		most likely feasible trajectory
		*/
		virtual void selectTrajectory(const Query& query, Trajectory& trajectory) = 0;
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_CONTACT_CONTACT_INTERFACE_H_