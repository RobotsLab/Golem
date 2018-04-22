/** @file Menu.h
 * 
 * User interface library
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
#ifndef _GOLEM_APP_MENU_H_
#define _GOLEM_APP_MENU_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/Defs.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Sys/Context.h>
#include <sstream>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Interactive text menu tools.
*/
class Menu {
public:
	typedef golem::shared_ptr<Menu> Ptr;
	/** Read value */
	typedef std::function<void(std::string&)> ReadValue;
	/** Read string value from list */
	typedef std::function<StringSeq(const std::string&)> ReadList;

	/** Menu command */
	typedef golem::Function MenuCmd;
	/** Menu command set */
	typedef std::map<std::string, MenuCmd> MenuCmdMap;
	/** Menu control */
	typedef std::function<void(MenuCmdMap&, std::string&)> MenuCtrl;
	/** Menu control set */
	typedef std::map<std::string, MenuCtrl> MenuCtrlMap;

	/** Read value template */
	template <typename _Type> void read(const char* name, _Type& val, const std::string& chars, ReadValue readValue = nullptr) const {
		std::ostringstream ostr;
		ostr << val;
		std::string str(ostr.str()), clr;
		size_t clrn = 0;

		for (;;) {
			clr.assign(clrn, ' ');
			contextMenu.write("\r%s%s%s", name, str.c_str(), clr.c_str());

			const int key = callback.waitKey();

			if (key & golem::UIKeyboardMouseCallback::KEY_SPECIAL)
				continue;
			else if (key == 27) {  // <Esc>
				throw Cancel("\nCancelled");
			}
			else if (key == 13) { // <Enter>
				contextMenu.write("\n");
				break;
			}
			else if (key == 8) {  // <Bkspace>
				if (str.length() > 0) {
					str.erase(str.length() - 1);
					clrn = 1;
				}
			}
			else if (key == 9 && readValue != nullptr) { // <Tab>
				clrn = str.length();
				readValue(str);
				clrn = std::max(0, (int)clrn - (int)str.length());
			}
			else if (chars.find((char)key) != std::string::npos) { // numbers
				str += (char)key;
			}
			else {
				contextMenu.write(
					"\n<Bkspace>        delete last digit\n"
					"<Tab>            expand\n"
					"<Esc>            cancel\n"
					"<Enter>          accept and exit\n"
					);
			}
		}

		std::istringstream(str) >> val;
	}
	/** Read numeric value */
	template <typename _Type> void readNumber(const char* name, _Type& val, ReadValue readValue = nullptr) const {
		read(name, val, "-0123456789.", readValue);
	}
	/** Read string value */
	template <typename _Type> void readString(const char* name, _Type& val, ReadValue readValue = nullptr) const {
		read(name, val, "0123456789qwertyuiopasdfghjklzxcvbnmQWERTYUIOPASDFGHJKLZXCVBNM_-+. \\/:@$&^()=", readValue);
	}

	/** Select serial access item */
	template <typename _Ptr, typename _Iter, typename _Info> void select(_Ptr& ptr, _Iter begin, _Iter end, const char* name, _Info info) const {
		golem::U32 size = 0, index = 0;
		contextMenu.write("%s", name);
		for (_Iter i = begin; i != end; ++i) {
			++size;
			if (i == ptr) index = size;
			contextMenu.write("  [%u]  %s\n", size, info(i).c_str());
		}

		index = std::min(size, index);
		readNumber("Enter index: ", index);
		if (index < 1 || index > size)
			throw golem::Message(golem::Message::LEVEL_ERROR, "Invalid index #%u", index);
		for (ptr = begin; ptr != end && index != 1; ++ptr, --index);
	}

	/** Select index */
	template <typename _Seq, typename _Index> void selectIndex(const _Seq& seq, _Index& index, const std::string& name) {
		if (seq.empty())
			throw Cancel("Empty!");
		// select index within the range
		std::stringstream str;
		golem::Math::clamp(index, (_Index)1, (_Index)seq.size());
		str << "Enter " << name << " index <1.." << seq.size() << ">: ";
		readNumber(str.str().c_str(), index);
		if (size_t(index) < 1 || size_t(index) > seq.size())
			throw Cancel("Invalid index");
	}

	/** Read string value from list */
	virtual void readList(const char* name, std::string& val, ReadList readList) const;
	/** Read path */
	virtual void readPath(const char* name, std::string& val, const StringSeq& extSeq = StringSeq()) const;
	/** Read path */
	virtual void readPath(const char* name, std::string& val, const char* ext) const;

	/** Option */
	virtual size_t option(size_t index, const char* name, const StringSeq& seq) const;

	/** Read a key from keys, display msg */
	virtual int option(const std::string& keys, const char* str, bool silent = false) const;

	/** User interface: menu function */
	virtual void menu(const MenuCtrlMap& menuCtrlMap, MenuCmdMap& menuCmdMap, golem::U32& menuLevel) const;

	/** Init with callback interface */
	Menu(golem::Context& context, golem::UIKeyboardMouseCallback& callback);

	/** Required */
	virtual ~Menu() {}

private:
	/** Program context */
	golem::Context& contextMenu;
	/** golem::UIKeyboardMouseCallback */
	golem::UIKeyboardMouseCallback& callback;
};

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------


#endif /*_GOLEM_APP_MENU_H_*/
