/** @file GolemInteropContactStreamDefs.h
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
#ifndef _GOLEM_INTEROP_CONTACT_STREAM_DEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_CONTACT_STREAM_DEFS_H_

//------------------------------------------------------------------------------

#include "GolemInteropContactDefs.h"
#include "GolemInteropStreamDefs.h"

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Serialisation of data structures
	*
	**************************************************************************/

	template <> void StreamWrite(Stream& stream, const Feature3D& data) {
		StreamWrite(stream, (const Sample&)data);
		StreamWrite(stream, (const Point3D&)data);
		StreamWrite(stream, data.direction);
		StreamWrite(stream, data.featureType);
		StreamWrite(stream, data.featureSize);
		//StreamWrite(stream, data.featureData);
		stream.write(data.featureSize * sizeof(Feature::TYPE), data.featureData.v);
	}
	template <> void StreamRead(Stream& stream, Feature3D& data) {
		StreamRead(stream, (Sample&)data);
		StreamRead(stream, (Point3D&)data);
		StreamRead(stream, data.direction);
		StreamRead(stream, data.featureType);
		StreamRead(stream, data.featureSize);
		//StreamRead(stream, data.featureData);
		stream.read(data.featureSize * sizeof(Feature::TYPE), data.featureData.v);
	}

	template <> void StreamWrite(Stream& stream, const Training3D& data) {
		StreamWrite(stream, data.features);
		StreamWrite(stream, data.trajectory);
	}
	template <> void StreamRead(Stream& stream, Training3D& data) {
		StreamRead(stream, data.features);
		StreamRead(stream, data.trajectory);
	}

	template <> void StreamWrite(Stream& stream, const ConfigModel& data) {
		StreamWrite(stream, (const Sample&)data);
		StreamWrite(stream, data.config);
		StreamWrite(stream, data.frame);
	}
	template <> void StreamRead(Stream& stream, ConfigModel& data) {
		StreamRead(stream, (Sample&)data);
		StreamRead(stream, data.config);
		StreamRead(stream, data.frame);
	}

	template <> void StreamWrite(Stream& stream, const PathModel& data) {
		StreamWrite(stream, (const Sample&)data);
		StreamWrite(stream, (const ConfigModel::Seq&)data);
	}
	template <> void StreamRead(Stream& stream, PathModel& data) {
		StreamRead(stream, (Sample&)data);
		StreamRead(stream, (ConfigModel::Seq&)data);
	}

	template <> void StreamWrite(Stream& stream, const ConfigSpace& data) {
		StreamWrite(stream, data.name);
		StreamWrite(stream, data.paths);
		StreamWrite(stream, data.configs);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, ConfigSpace& data) {
		StreamRead(stream, data.name);
		StreamRead(stream, data.paths);
		StreamRead(stream, data.configs);
		StreamRead(stream, data.reserved);
	}

	template <> void StreamWrite(Stream& stream, const ContactModel3D& data) {
		StreamWrite(stream, (const Sample&)data);
		StreamWrite(stream, data.global);
		StreamWrite(stream, data.local);
		StreamWrite(stream, data.featureSize);
		//StreamWrite(stream, data.feature);
		stream.write(data.featureSize * sizeof(Feature::TYPE), data.feature.v);
		StreamWrite(stream, data.model);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, ContactModel3D& data) {
		StreamRead(stream, (Sample&)data);
		StreamRead(stream, data.global);
		StreamRead(stream, data.local);
		StreamRead(stream, data.featureSize);
		//StreamRead(stream, data.feature);
		stream.read(data.featureSize * sizeof(Feature::TYPE), data.feature.v);
		StreamRead(stream, data.model);
		StreamRead(stream, data.reserved);
	}

	template <> void StreamWrite(Stream& stream, const ContactModel3D::Data& data) {
		StreamWrite(stream, data.type);
		StreamWrite(stream, data.model);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, ContactModel3D::Data& data) {
		StreamRead(stream, data.type);
		StreamRead(stream, data.model);
		StreamRead(stream, data.reserved);
	}

	template <> void StreamWrite(Stream& stream, const ContactView3D::ModelPtr& data) {
		StreamWrite(stream, data.index);
		StreamWrite(stream, data.weight);
		StreamWrite(stream, data.frame);
	}
	template <> void StreamRead(Stream& stream, ContactView3D::ModelPtr& data) {
		StreamRead(stream, data.index);
		StreamRead(stream, data.weight);
		StreamRead(stream, data.frame);
	}

	template <> void StreamWrite(Stream& stream, const ContactView3D& data) {
		StreamWrite(stream, (const Sample&)data);
		StreamWrite(stream, data.space);
		StreamWrite(stream, data.models);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, ContactView3D& data) {
		StreamRead(stream, (Sample&)data);
		StreamRead(stream, data.space);
		StreamRead(stream, data.models);
		StreamRead(stream, data.reserved);
	}

	template <> void StreamWrite(Stream& stream, const Model3D& data) {
		StreamWrite(stream, data.contacts);
		StreamWrite(stream, data.views);
		StreamWrite(stream, data.spaces);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, Model3D& data) {
		StreamRead(stream, data.contacts);
		StreamRead(stream, data.views);
		StreamRead(stream, data.spaces);
		StreamRead(stream, data.reserved);
	}

	template <> void StreamWrite(Stream& stream, const Path& data) {
		StreamWrite(stream, (const Sample&)data);
		StreamWrite(stream, data.type);
		StreamWrite(stream, data.view);
		StreamWrite(stream, data.space);
		StreamWrite(stream, data.path);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, Path& data) {
		StreamRead(stream, (Sample&)data);
		StreamRead(stream, data.type);
		StreamRead(stream, data.view);
		StreamRead(stream, data.space);
		StreamRead(stream, data.path);
		StreamRead(stream, data.reserved);
	}

	template <> void StreamWrite(Stream& stream, const Query& data) {
		StreamWrite(stream, data.paths);
		StreamWrite(stream, data.reserved);
	}
	template <> void StreamRead(Stream& stream, Query& data) {
		StreamRead(stream, data.paths);
		StreamRead(stream, data.reserved);
	}

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_CONTACT_STREAM_DEFS_H_