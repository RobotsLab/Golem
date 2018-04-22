/** @file DataI.h
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
#ifndef _GOLEM_PLUGIN_DATAI_H_
#define _GOLEM_PLUGIN_DATAI_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Data.h>
#include <Golem/Plugin/Defs.h>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Generate creates multiple items.
*	(Optionally) Implemented by Handler.
*/
class Generate {
public:
	/** Generate: map of items */
	virtual void generate(Item::Map& ouput) = 0;
};

/** Transform creates Item from single or multiple Items of the same or different type.
*	(Optionally) Implemented by Handler.
*/
class Transform {
public:
	/** Transform: transform items */
	virtual Item::Ptr transform(const Item::List& input) = 0;
	/** Transform: return available interfaces */
	virtual const StringSeq& getTransformInterfaces() const = 0;
	/** Transform: is supported by the interface */
	virtual bool isTransformSupported(const Item& item) const = 0;
};

/** Convert creates from the current Item another Item of a given output type (determined by Handler).
*	(Optionally) Implemented by Item.
*/
class Convert {
public:
	/** Convert: convert current item */
	virtual Item::Ptr convert(const Handler& handler) = 0;
	/** Convert: return available interfaces */
	virtual const StringSeq& getConvertInterfaces() const = 0;
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const Handler& handler) const = 0;
};

/** Imports Item from external file.
*	(Optionally) Implemented by Handler.
*/
class Import {
public:
	/** Import from file */
	virtual Item::Ptr import(const std::string& path) = 0;
	/** Available file types */
	virtual const StringSeq& getImportFileTypes() const = 0;
};

/** Exports Item to external file.
*	(Optionally) Implemented by Item.
*/
class Export {
public:
	/** Export to file */
	virtual void exportt(const std::string& path) const = 0;
	/** Available file types */
	virtual const StringSeq& getExportFileTypes() const = 0;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_DATAI_H_*/
