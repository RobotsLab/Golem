/** @file Data.h
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
#ifndef _GOLEM_CONTACT_DATA_H_
#define _GOLEM_CONTACT_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Ctrl.h>
#include <Golem/Plugin/Point.h>
#include <Golem/Contact/Model.h>
#include <Golem/Contact/Query.h>
#include <Golem/Contact/Configuration.h>
#include <Golem/Contact/Contact.h>
#include <vector>
#include <map>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Contact model data interface.
*	(Optionally) Implemented by Item.
*/
class ContactModel {
public:
	/** Model index */
	//typedef ::golem::U32 Index;

	/** Contains two types of data: (1) single contact type, object, view and trajectory; (2) single contact type, combined data
	*/
	class Data {
	public:
		/** Data collection: contact type -> data */
		typedef std::multimap<std::string, Data> Map;

		/** contact model re-mapping: old index -> new index */
		typedef std::map<golem::U32, golem::U32> ContactReMap;

		/** Type */
		enum Type {
			/** Default: ready for inference */
			TYPE_DEFAULT = 0,
			/** Debug: visualisation */
			TYPE_DEBUG,
		};

		/** Data operation description */
		class Desc {
		public:
			typedef golem::shared_ptr<Desc> Ptr;

			/** Copy points */
			bool copyPoints;

			/** Contact3D properties description */
			Contact3D::Data::Desc contact3DDesc;

			/** Constructs description object */
			Desc() {
				Desc::setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				copyPoints = false;
				contact3DDesc.setToDefault();
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				contact3DDesc.assertValid(Assert::Context(ac, "feature3DDesc."));
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Appearance */
		class Appearance {
		public:
			/** Show path */
			bool pathShow;
			/** Path appearance */
			Configuration::Path::Appearance path;

			/** Contacts appearance */
			Contact3D::Appearance::Map contacts;
			/** Show contact relations */
			Contact3D::Relation relation;

			/** Show point */
			bool pointShow;
			/** Point colour */
			Point3D::Appearance point;

			/** Manifold appearance */
			ManifoldCtrl::Appearance manifold;

			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				pathShow = true;
				path.setToDefault();
				contacts.insert(std::make_pair(Manipulator::Link::getName(Manipulator::Link::TYPE_ANY), Contact3D::Appearance()));
				relation = Contact3D::RELATION_DFLT;
				pointShow = true;
				point.setToDefault();
				manifold.setToDefault();
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				path.assertValid(Assert::Context(ac, "path."));

				Assert::valid(!contacts.empty(), ac, "contacts: empty");
				for (Contact3D::Appearance::Map::const_iterator i = contacts.begin(); i != contacts.end(); ++i)
					i->second.assertValid(Assert::Context(ac, "contacts[]."));

				point.assertValid(Assert::Context(ac, "point."));

				manifold.assertValid(Assert::Context(ac, "manifold->"));
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Header */
		class Header : public golem::Header {
		public:
			/** Version */
			static const Version VERSION;
			/** Version */
			virtual Version getVersion() const { return VERSION; }
			/** Id */
			static const std::string ID;
			/** Id */
			virtual const std::string& getId() const { return ID; }
		} header;

		/** Contact models */
		Contact3D::Data::Map contacts;
		/** Contact views */
		Contact::View::Seq views;
		/** Configuration sub-spaces */
		Configuration::Space::Seq spaces;

		/** Query density selector map */
		Contact::SelectorMap selectorMap;

		/** Type */
		::golem::U32 type;

		/** add */
		void add(const Desc& desc, const golem::Configuration& configuration, const Contact3D::Data::Map& contacts, const Contact::View::Seq& views, const Configuration::Space::Seq& spaces, const Contact::SelectorMap& selectorMap);
		/** Merging views */
		void add(const Desc& desc, const golem::Configuration& configuration, const Data& data);
		/** Merging contact model data bundles */
		void add(const Desc& desc, const golem::Configuration& configuration, golem::data::ContactModel::Data::Map::const_iterator begin, golem::data::ContactModel::Data::Map::const_iterator end);
		/** clear */
		void clear();
		/** is empty */
		bool empty();

		/** Clamp indices */
		void clamp(golem::U32& viewIndex, golem::U32& pathIndex) const;

		/** Draw training data */
		void draw(const Configuration& configuration, const Appearance& appearance, golem::U32 viewIndex, golem::U32 pathIndex, golem::DebugRenderer& renderer, golem::DebugRenderer* rendererPoint = nullptr) const;

		/** Initialisation */
		Data(Type type = TYPE_DEFAULT) : type(type) {}
	};

	/** Sets contact model data. */
	virtual void setData(const ContactModel::Data::Map& dataMap) = 0;
	/** Returns contact model data. */
	virtual const ContactModel::Data::Map& getData() const = 0;
};

//------------------------------------------------------------------------------

/** Contact query data interface.
*	(Optionally) Implemented by Item.
*/
class ContactQuery {
public:
	class Data {
	public:
		/** Appearance */
		class Appearance {
		public:
			/** Config appearance */
			Contact::Config::Appearance config;

			/** Show point */
			bool pointShow;
			/** Point colour */
			Point3D::Appearance point;

			/** Constructs description. */
			Appearance() {
				setToDefault();
			}
			/** Sets the parameters to the default values. */
			void setToDefault() {
				config.setToDefault();
				pointShow = true;
				point.setToDefault();
			}
			/** Assert that the description is valid. */
			void assertValid(const Assert::Context& ac) const {
				config.assertValid(Assert::Context(ac, "config."));
				point.assertValid(Assert::Context(ac, "point."));
			}
			/** Load descritpion from xml context. */
			void load(const golem::XMLContext* xmlcontext);
		};

		/** Header */
		class Header : public golem::Header {
		public:
			/** Version */
			static const Version VERSION;
			/** Version */
			virtual Version getVersion() const { return VERSION; }
			/** Id */
			static const std::string ID;
			/** Id */
			virtual const std::string& getId() const { return ID; }
		} header;

			/** Configs */
		Contact::Config::Seq configs;
		/** Clusters */
		Contact::Config::Cluster::Seq clusters;

		/** Points (visualisation) */
		F32Vec3Seq points;

		/** Cluster3D indices */
		Cluster3D::IndexSeq pointsIndices;
		/** Cluster3D clusters */
		Cluster3D::IndexSeq pointsClusters;
		/** Cluster3D selection */
		Cluster3D::IndexMap pointsSelection;

		/** clear */
		void clear();
	};

	/** ContactQuery: sets contact query data. */
	virtual void setData(const ContactQuery::Data& data) = 0;
	/** ContactQuery: returns contact query data. */
	virtual const ContactQuery::Data& getData() const = 0;

	/** ContactQuery: get current config */
	virtual Contact::Config::Seq::const_iterator getConfig() const = 0;
	/** ContactQuery: set current config */
	virtual void setConfig(Contact::Config::Seq::const_iterator config) = 0;

	/** ContactQuery: get config cluster */
	virtual Contact::Config::SetConstPtr getConfigCluster() const = 0;
	/** ContactQuery: get config selection */
	virtual Contact::Config::SetConstPtr getConfigSelection() const = 0;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::data::ContactModel::Data::Map::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::data::ContactModel::Data::Map::value_type& value);

	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::data::ContactQuery::Data& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::data::ContactQuery::Data& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_DATA_H_*/
