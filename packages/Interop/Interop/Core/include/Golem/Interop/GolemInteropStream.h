/** @file GolemInteropStream.h
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
#ifndef _GOLEM_INTEROP_INTEROP_STREAM_H_ // if #pragma once is not supported
#define _GOLEM_INTEROP_INTEROP_STREAM_H_

//------------------------------------------------------------------------------

#include <memory>

//------------------------------------------------------------------------------

/** Golem name space */
namespace golem {
/** Golem interoperability name space */
namespace interop {

	/*************************************************************************
	*
	* Marshalling/serialisation interface
	*
	**************************************************************************/

	/** Marshalling/serialisation interface */
	class Stream {
	public:
		/** Pointer */
		typedef std::shared_ptr<Stream> Ptr;
		
		/** Read from stream */
		virtual void read(size_t size, void* data) = 0;
		/** Write to stream */
		virtual void write(size_t size, const void* data) = 0;
	};

}; // namespace interop
}; // namespace golem

//------------------------------------------------------------------------------

#endif // _GOLEM_INTEROP_INTEROP_STREAM_H_