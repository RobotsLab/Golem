/** @file GolemInteropContactDefs.h
*
* @author	Marek Kopicki
*
*/

#pragma once
#ifndef _GOLEM_INTEROP_CONTACT_CONTACT_DEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_CONTACT_CONTACT_DEFS_H_

//------------------------------------------------------------------------------

#include "GolemInteropDefs.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Contact data structures
	*
	**************************************************************************/

	/** Contact manifold */
	class Manifold {
	public:
		/** Frame */
		Mat34 frame;
		/** Frame deviation */
		Twist frameDev;

		/** Set to the default values. */
		void clear() {
			frame.clear();
			frameDev.clear();
		}
	};

	/** Visual feature type, max dim = Feature::SIZE */
	typedef Vec<float_t, 36> Feature;

	/** 3D point with feature vector and weight. */
	class Feature3D : public Point3D, public Sample {
	public:
		/** Sequence */
		typedef std::vector<Feature3D> Seq;

		/** Feature surface (principal) direction. */
		Vec3 direction;

		/** Feature type. */
		std::uint32_t featureType;
		/** Feature size. */
		std::uint32_t featureSize;
		/** Feature data */
		Feature featureData;

		/** Default constructor sets the default values. */
		Feature3D() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			Point3D::clear();
			Sample::clear();
			direction.clear();
			featureType = 0;
			featureSize = 0;
			featureData.clear();
		}
	};

	/** Training data */
	class Training3D {
	public:
		/** Map contact type -> data */
		typedef std::map<std::string, Training3D> Map;

		/** Trajectory */
		Config::Seq trajectory;
		/** Features */
		Feature3D::Seq features;

		/** Set to the default values. */
		void clear() {
			trajectory.clear();
			features.clear();
		}
	};

	/** Configuration model represents robot configuration density at contact  */
	class ConfigModel : public Sample {
	public:
		/** Sequence representing configuration model density */
		typedef std::vector<ConfigModel> Seq;

		/** Robot configuration */
		ConfigspaceCoord config;
		/** Robot end-effector frame */
		Frame3D frame;

		/** Set to the default values. */
		void clear() {
			Sample::clear();
			config.clear();
			frame.clear();
		}
	};

	/** Robot path model */
	class PathModel : public ConfigModel::Seq, public Sample {
	public:
		/** Weighted sequence of path models */
		typedef std::vector<PathModel> Seq;

		/** Set to the default values. */
		void clear() {
			ConfigModel::Seq::clear();
			Sample::clear();
		}
	};

	/** Configuration sub-space */
	class ConfigSpace {
	public:
		/** Set of sub-spaces */
		typedef std::vector<ConfigSpace> Seq;

		/** Space name */
		std::string name;
		/** Paths leading to contact (last waypoint) */
		PathModel::Seq paths;
		/** Config model at contact */
		ConfigModel::Seq configs;

		/** Space reserved area */
		Buffer reserved;

		/** Set to the default values. */
		void clear() {
			name.clear();
			paths.clear();
			configs.clear();
			reserved.clear();
		}
	};

	/** Feature/parts-link contact */
	class ContactModel3D : public Sample {
	public:
		/** Sequence representing contact model density */
		typedef std::vector<ContactModel3D> Seq;

		/** Sequence representing contact model density */
		class Data {
		public:
			/** Map link name -> contact model density */
			typedef std::map<std::uint32_t, Data> Map;

			/** Type */
			std::uint32_t type;
			/** Model */
			Seq model;

			/** Reserved area */
			Buffer reserved;

			/** Default constructor sets the default values. */
			Data() {
				clear();
			}
			/** Set to the default values. */
			void clear() {
				type = 0;
				model.clear();
				reserved.clear();
			}
		};

		/** Reserved area */
		typedef Vec<std::uint8_t, 128> Reserved;

		/** Feature/parts global frame */
		Frame3D global;
		/** Feature/parts-link local frame */
		Frame3D local;

		/** Feature size. */
		std::uint32_t featureSize;
		/** Feature vector */
		Feature feature;
		/** Part model */
		std::uint64_t model;

		/** Reserved area */
		Reserved reserved;

		/** Default constructor sets the default values. */
		ContactModel3D() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			Sample::clear();
			global.clear();
			local.clear();
			featureSize = 0;
			feature.clear();
			model = 0;
			reserved.clear();
		}
	};

	/** Contact 3D view */
	class ContactView3D : public Sample {
	public:
		/** Collection of views */
		typedef std::vector<ContactView3D> Seq;

		/** Contact model pointer */
		class ModelPtr {
		public:
			/** Contact model multimap with one or more models per link */
			typedef std::multimap<std::uint32_t, ModelPtr> Map;

			/** Index */
			std::uint32_t index;
			/** Weight */
			float_t weight;
			/** Local frame */
			Mat34 frame;

			/** Default constructor sets the default values. */
			ModelPtr() {
				clear();
			}
			/** Set to the default values. */
			void clear() {
				index = 0;
				weight = float_t(1.); // equally weighted
				frame.clear();
			}
		};

		/** Space index */
		std::uint32_t space;
		/** Contact model map */
		ModelPtr::Map models;

		/** View reserved area */
		Buffer reserved;

		/** Default constructor sets the default values. */
		ContactView3D() {
			clear();
		}
		/** Set to the default values. */
		void clear() {
			Sample::clear();
			space = 0;
			models.clear();
			reserved.clear();
		}
	};

	/** Contact and configuration model data */
	class Model3D {
	public:
		/** Map contact type -> data */
		typedef std::map<std::string, Model3D> Map;

		/** Contact models */
		ContactModel3D::Data::Map contacts;
		/** Contact views */
		ContactView3D::Seq views;
		/** Configuration sub-spaces */
		ConfigSpace::Seq spaces;

		/** Model reserved area */
		Buffer reserved;

		/** Set to the default values. */
		void clear() {
			contacts.clear();
			views.clear();
			spaces.clear();
			reserved.clear();
		}
	};

	/** Robot path hypothesis */
	class Path : public Sample {
	public:
		/** Weighted sequence of paths */
		typedef std::vector<Path> Seq;

		/** Reserved area */
		typedef Vec<std::uint8_t, 1024> Reserved;

		/** Path type */
		std::string type;
		/** Path view index */
		std::uint32_t view;
		/** Path space index */
		std::uint32_t space;

		/** Manifold */
		Manifold manifold;

		/** Path waypoints */
		ConfigModel::Seq path;

		/** Reserved area */
		Reserved reserved;

		/** Set to the default values. */
		void clear() {
			Sample::clear();
			type.clear();
			view = 0;
			space = 0;
			manifold.clear();
			path.clear();
		}
	};

	/** Query data */
	class Query {
	public:
		/** Robot path hypothesis */
		Path::Seq paths;

		/** Query reserved area */
		Buffer reserved;

		/** Set to the default values. */
		void clear() {
			paths.clear();
			reserved.clear();
		}
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_CONTACT_CONTACT_DEFS_H_