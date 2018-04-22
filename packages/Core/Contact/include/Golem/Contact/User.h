/** @file User.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2017 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#pragma once
#ifndef _GOLEM_CONTACT_USER_H_
#define _GOLEM_CONTACT_USER_H_

//------------------------------------------------------------------------------

#include <Golem/Contact/Manipulator.h>

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

/** Contact query callback interface.
*	(Optionally) Implemented by Handler.
*/
class ContactQueryCallback {
public:
	/** Contact evaluation expert */
	class ContactEval {
	public:
		/** ContactQueryCallback::ContactEval: Contact evaluation expert */
		virtual Real contactEval(const Manipulator::Waypoint::Seq& path) = 0;
	};

	/** ContactQueryCallback: Set contact evaluation expert */
	virtual void setContactEval(ContactEval* contactEval = nullptr) = 0;
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_CONTACT_USER_H_*/
