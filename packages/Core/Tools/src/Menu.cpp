/** @file UI.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/Menu.h>
#ifdef LINUX
#define BOOST_NO_CXX11_SCOPED_ENUMS
#define BOOST_NO_SCOPED_ENUMS
#endif
#include <boost/filesystem.hpp>
#ifdef WIN32
#include <boost/algorithm/string.hpp>
#endif

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

golem::Menu::Menu(golem::Context& context, golem::UIKeyboardMouseCallback& callback) : contextMenu(context), callback(callback) {
}

//------------------------------------------------------------------------------

void golem::Menu::readList(const char* name, std::string& val, ReadList readList) const {
	readString(name, val, [=] (std::string& val) {
		const StringSeq list = readList(val);
		StringSeq::const_iterator begin = list.begin(), end = list.end();
		bool isFound = false;
		for (StringSeq::const_iterator i = begin; i != list.end(); ++i) {
			const bool equals = i->compare(0, val.length(), val) == 0;
			if (!isFound && equals) {
				begin = i;
				isFound = true;
			}
			if (isFound && equals)
				end = i + 1;
		}
		if (isFound)
			for (size_t k = val.length();; ++k)
				for (StringSeq::const_iterator j = begin; j != end; ++j)
					if (k >= begin->length() || k >= j->length() || (*begin)[k] != (*j)[k]) {
						val.assign(*begin, 0, k);
						goto END;
					}
		END:
		if (!isFound)
			return;
		if (begin != end)
			contextMenu.write("\n");
		const std::string space(strlen(name), ' ');
		while (begin != end)
			contextMenu.write("%s%s\n", space.c_str(), (begin++)->c_str());
	});
}

void golem::Menu::readPath(const char* name, std::string& val, const StringSeq& extSeq) const {
	const std::string sep = "/";
	readList(name, val, [=] (const std::string& val) -> StringSeq {
		std::string dir;
		const size_t pos = val.rfind(sep);
		val.length() > 0 ? pos == std::string::npos ? dir.assign(val) : dir.assign(val, 0, pos + 1) : dir.assign("." + sep);
#ifdef WIN32
		boost::replace_all(dir, sep, "\\");
#endif
		StringSeq list;
		if (boost::filesystem::exists(dir)) {
			for (boost::filesystem::directory_iterator i(dir), end; i != end; ++i) {
				std::string path = i->path().string();
#ifdef WIN32
				boost::replace_all(path, "\\", sep);
#endif
				if (boost::filesystem::is_directory(i->status()))
					list.push_back(path + sep);
				else if (extSeq.empty() || std::find(extSeq.begin(), extSeq.end(), getExt(path)) != extSeq.end())
					list.push_back(path);
			}
		}
		return list;
	});
}

void golem::Menu::readPath(const char* name, std::string& val, const char* ext) const {
	StringSeq extSeq;
	extSeq.push_back(ext);
	readPath(name, val, extSeq);
}

//------------------------------------------------------------------------------

size_t golem::Menu::option(size_t index, const char* name, const StringSeq& seq) const {
	index = std::min(index, seq.size());
	size_t next = index;
	std::string clr;

	for (;;) {
		clr.assign(std::max(0, (int)seq[index].length() - (int)seq[next].length()), ' ');
		index = next;
		contextMenu.write("\r%s%s%s", name, seq[index].c_str(), clr.c_str());

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
		else if (key == 32 || key == 9) // <Space>
			next = next < seq.size() - 1 ? next + 1 : 0;
		else if (key == 8)  // <Bkspace>
			next = next > 0 ? next - 1 : seq.size() - 1;
		else {
			contextMenu.write(
				"\n<Space>/<Tab>    forward\n"
				"<Bkspace>        backward\n"
				"<Esc>            cancel\n"
				"<Enter>          accept and exit\n"
			);
		}
	}

	return index;
}

int golem::Menu::option(const std::string& keys, const char* str, bool silent) const {
	if (str != nullptr && std::strlen(str) > 0)
		contextMenu.write("%s", str);
	else
		silent = true;

	for (;;) {
		const int key = callback.waitKey();
		if (key == 27) {// <Esc>
			if (!silent) contextMenu.write("\n");
			throw Cancel("Cancelled");
		}
		if (keys.find(char(key)) != std::string::npos) {
			if (!silent) contextMenu.write(" )%c(\n", (char)key);
			return key;
		}
	}
	
	return -1;
}

//------------------------------------------------------------------------------

void golem::Menu::menu(const MenuCtrlMap& menuCtrlMap, MenuCmdMap& menuCmdMap, golem::U32& menuLevel) const {
	if (menuCmdMap.empty())
		throw Message(Message::LEVEL_CRIT, "golem::Menu::menu(): empty menu");

	std::string menuCmd = menuCmdMap.begin()->first.substr(0, menuLevel);
	const MenuCtrlMap::const_iterator ctrl = menuCtrlMap.find(menuCmd);

	if (ctrl == menuCtrlMap.end())
		throw Message(Message::LEVEL_CRIT, "golem::Menu::menu(): unable to find menu control \"%s\"", menuCmd.c_str());

	std::string desc;
	ctrl->second(menuCmdMap, desc);

	std::string keys;
	for (MenuCmdMap::const_iterator i = menuCmdMap.begin(); i != menuCmdMap.end(); ++i) {
		if (i->first.length() > menuCmd.length())
			keys += i->first[menuCmd.length()];
		else
			throw Message(Message::LEVEL_CRIT, "golem::Menu::menu(): invalid menu length");
	}

	menuCmd += (char)option(keys, desc.c_str());

	++menuLevel;
	ScopeGuard guard([&] () { --menuLevel; });

	MenuCmdMap::const_iterator cmd = menuCmdMap.find(menuCmd);
	if (cmd != menuCmdMap.end()) {
		cmd->second();
		return;
	}

	menuCmdMap.erase(menuCmdMap.begin(), menuCmdMap.upper_bound(menuCmd));
	MenuCmdMap::const_iterator end = menuCmdMap.begin();
	for (; end != menuCmdMap.end() && end->first.find(menuCmd) == 0; ++end);
	menuCmdMap.erase(end, menuCmdMap.end());

	menu(menuCtrlMap, menuCmdMap, menuLevel);
}

//------------------------------------------------------------------------------

