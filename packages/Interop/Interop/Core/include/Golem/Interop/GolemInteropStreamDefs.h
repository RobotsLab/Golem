/** @file GolemInteropStreamDefs.h
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
#ifndef _GOLEM_INTEROP_INTEROP_STREAM_DEFS_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_STREAM_DEFS_H_

//------------------------------------------------------------------------------

#include "GolemInteropDefs.h"
#include "GolemInteropStream.h"

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

	template <typename _Type> void StreamWrite(Stream& stream, const _Type& data) {
		stream.write(sizeof(_Type), &data);
	}
	template <typename _Type> void StreamRead(Stream& stream, _Type& data) {
		stream.read(sizeof(_Type), &data);
	}

	template <> void StreamWrite(Stream& stream, const std::string& data) {
		const std::uint32_t size = static_cast<std::uint32_t>(data.size());
		stream.write(sizeof(std::uint32_t), &size);
		stream.write(size*sizeof(char), data.data());
	}
	template <> void StreamRead(Stream& stream, std::string& data) {
		std::uint32_t size = 0;
		stream.read(sizeof(std::uint32_t), &size);
		data.resize(size);
		stream.read(size*sizeof(char), const_cast<char*>(data.data()));
	}

	template <typename _Type> void StreamWrite(Stream& stream, const std::vector<_Type>& data) {
		const std::uint32_t size = static_cast<std::uint32_t>(data.size());
		stream.write(sizeof(std::uint32_t), &size);
		for (typename std::vector<_Type>::const_iterator i = data.begin(); i != data.end(); ++i)
			StreamWrite(stream, *i);
	}
	template <typename _Type> void StreamRead(Stream& stream, std::vector<_Type>& data) {
		std::uint32_t size = 0;
		stream.read(sizeof(std::uint32_t), &size);
		data.clear();
		data.reserve(size);
		for (; size > 0; --size) {
			_Type val;
			StreamRead(stream, val);
			data.insert(data.end(), val);
		}
	}

	template <typename _Type> void StreamWrite(Stream& stream, const std::set<_Type>& data) {
		const std::uint32_t size = static_cast<std::uint32_t>(data.size());
		stream.write(sizeof(std::uint32_t), &size);
		for (typename std::set<_Type>::const_iterator i = data.begin(); i != data.end(); ++i)
			StreamWrite(stream, *i);
	}
	template <typename _Type> void StreamRead(Stream& stream, std::set<_Type>& data) {
		std::uint32_t size = 0;
		stream.read(sizeof(std::uint32_t), &size);
		data.clear();
		for (; size > 0; --size) {
			_Type val;
			StreamRead(stream, val);
			data.insert(data.end(), val);
		}
	}

	template <typename _Key, typename _Type> void StreamWrite(Stream& stream, const std::map<_Key, _Type>& data) {
		const std::uint32_t size = static_cast<std::uint32_t>(data.size());
		stream.write(sizeof(std::uint32_t), &size);
		for (typename std::map<_Key, _Type>::const_iterator i = data.begin(); i != data.end(); ++i) {
			StreamWrite(stream, i->first);
			StreamWrite(stream, i->second);
		}
	}
	template <typename _Key, typename _Type> void StreamRead(Stream& stream, std::map<_Key, _Type>& data) {
		std::uint32_t size = 0;
		stream.read(sizeof(std::uint32_t), &size);
		data.clear();
		for (std::uint32_t i = 0; i < size; ++i) {
			typename std::map<_Key, _Type>::value_type value;
			StreamRead(stream, const_cast<_Key&>(value.first));
			StreamRead(stream, value.second);
			data.insert(data.end(), value);
		}
	}

	template <typename _Key, typename _Type> void StreamWrite(Stream& stream, const std::multimap<_Key, _Type>& data) {
		const std::uint32_t size = static_cast<std::uint32_t>(data.size());
		stream.write(sizeof(std::uint32_t), &size);
		for (typename std::map<_Key, _Type>::const_iterator i = data.begin(); i != data.end(); ++i) {
			StreamWrite(stream, i->first);
			StreamWrite(stream, i->second);
		}
	}
	template <typename _Key, typename _Type> void StreamRead(Stream& stream, std::multimap<_Key, _Type>& data) {
		std::uint32_t size = 0;
		stream.read(sizeof(std::uint32_t), &size);
		data.clear();
		for (std::uint32_t i = 0; i < size; ++i) {
			typename std::map<_Key, _Type>::value_type value;
			StreamRead(stream, const_cast<_Key&>(value.first));
			StreamRead(stream, value.second);
			data.insert(data.end(), value);
		}
	}

	template <typename _Point> void StreamWrite(Stream& stream, const Cloud<_Point>& data) {
		StreamWrite(stream, data.width);
		StreamWrite(stream, data.height);
		StreamWrite(stream, data.frame);
		StreamWrite(stream, (const std::vector<_Point>&)data);
	}
	template <typename _Point> void StreamRead(Stream& stream, Cloud<_Point>& data) {
		StreamRead(stream, data.width);
		StreamRead(stream, data.height);
		StreamRead(stream, data.frame);
		StreamRead(stream, (std::vector<_Point>&)data);
	}

	template <> void StreamWrite(Stream& stream, const Trajectory& data) {
		StreamWrite(stream, data.type);
		StreamWrite(stream, data.trajectory);
		StreamWrite(stream, data.error);
	}
	template <> void StreamRead(Stream& stream, Trajectory& data) {
		StreamRead(stream, data.type);
		StreamRead(stream, data.trajectory);
		StreamRead(stream, data.error);
	}

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_STREAM_DEFS_H_