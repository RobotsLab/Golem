/** @file Data.h
 * 
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
#ifndef _GOLEM_CTRL_KUKA_DATA_H_
#define _GOLEM_CTRL_KUKA_DATA_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/Kuka/KukaKR5Sixx.h>
#include <Golem/Ctrl/Kuka/KukaLWR.h>
#include <Golem/Ctrl/Kuka/KukaIIWA.h>
#include <Golem/Sys/XMLParser.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(KukaKR5SixxChain::ChainModel &val, XMLContext* context, bool create = false);
void XMLData(KukaKR5SixxChain::Desc &val, XMLContext* context, bool create = false);
void XMLData(KukaKR5Sixx::Desc &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(KukaLWRChain::ChainModel &val, XMLContext* context, bool create = false);
void XMLData(KukaLWRChain::Desc &val, XMLContext* context, bool create = false);
void XMLData(KukaLWR::Desc &val, XMLContext* context, bool create = false);

/** Reads/writes object from/to a given XML context */
void XMLData(KukaIIWAChain::ChainModel &val, XMLContext* context, bool create = false);
void XMLData(KukaIIWAChain::Desc &val, XMLContext* context, bool create = false);
void XMLData(KukaIIWA::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKA_DATA_H_*/
