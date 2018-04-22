/** @file Manager.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Manager.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <boost/algorithm/string.hpp>
#include <sstream>

using namespace golem;

//------------------------------------------------------------------------------

golem::Manager::Data::Selection::Selection() : item(0) {}

golem::Manager::Data::Selection::Selection(const data::Item::Map& itemMap, const View& view) : item(0) {
	create(itemMap, view.ptrLabel, view.ptrItem);
}

golem::Manager::Data::Selection::Selection(const data::Item::Map& itemMap, const Selection& selection) : item(0) {
	create(itemMap, selection.getLabel(), selection.getItem());
}

void golem::Manager::Data::Selection::assertValid() const {
	if (!isValid())
		throw Message(Message::LEVEL_ERROR, "Manager::Data::Selection::assertValid(): invalid selection");
}

void golem::Manager::Data::Selection::create(const data::Item::Map& itemMap, const std::string& label, golem::U32 item) {
	this->label = label;
	this->item = item;
	this->type.clear();
	this->ptr = Data::View::getItem<data::Item::Map::const_iterator>(itemMap, this->label, this->item, true);
	if (this->ptr != itemMap.end() && this->item == item)
		type = this->ptr->second->getHandler().getType();
}

bool golem::Manager::Data::Selection::add(const Selection& selection, List& list) {
	selection.assertValid();
	list.push_back(selection);
	return true;
}

bool golem::Manager::Data::Selection::add(const Selection& selection, List& list, Map& map) {
	selection.assertValid();
	Map::const_iterator ptr = map.find(selection.getPtr()->second.get());
	if (ptr == map.end()) {
		map.insert(std::make_pair(selection.getPtr()->second.get(), list.insert(list.end(), selection)));
		return true;
	}
	return false;
}

bool golem::Manager::Data::Selection::add(const data::Item::Map& itemMap, const std::string& label, List& list, Map& map) {
	return false;
}

bool golem::Manager::Data::Selection::remove(const Selection& selection, List& list) {
	selection.assertValid();
	if (list.empty()/* || selection.getPtr()->second.get() != list.back().getPtr()->second.get()*/)
		return false;
	list.pop_back();
	return true;
}

bool golem::Manager::Data::Selection::remove(const Selection& selection, List& list, Map& map) {
	selection.assertValid();
	Map::const_iterator ptr = map.find(selection.getPtr()->second.get());
	if (ptr != map.end()) {
		list.erase(ptr->second);
		map.erase(ptr);
		return true;
	}
	return false;
}

bool golem::Manager::Data::Selection::remove(const data::Item::Map& itemMap, const std::string& label, List& list, Map& map) {
	return false;
}

void golem::Manager::Data::Selection::print(List::const_iterator begin, List::const_iterator end, golem::Context& context) {
	std::stringstream str;
	//for (List::const_iterator i = begin, j = begin, k = begin; begin != end;) {
	//	++k;
	//	if (k != end && j->getHandler() == k->getHandler() && j->getPtr()->first == k->getPtr()->first && j->getItem() == k->getItem() + 1)
	//		++j;
	//	else {
	//		str << "(" << i->getPtr()->first << ", " << (i->getItem() + 1);
	//		if (i != j) str << "-" << (j->getItem() + 1);
	//		str << ") ";
	//		i = j = k;
	//	}
	//	if (k == end)
	//		break;
	//}
	for (List::const_iterator i = begin; i != end; ++i)
		str << "(" << i->getPtr()->first << "/" << i->getLabel() << ", " << (i->getItem() + 1) << ") ";
	
	if (begin == end)
		context.write("()");
	else for (size_t ptr = 0; str.str().size() > ptr*Message::MAX_SIZE; ++ptr)
		context.write("%s", str.str().data() + ptr*Message::MAX_SIZE);
	context.write("\n");
}

//------------------------------------------------------------------------------

const std::string golem::Manager::Data::LABEL_DEFAULT = "DEFAULT";

golem::data::Data::Ptr golem::Manager::Data::Desc::create(golem::Context &context) const {
	golem::data::Data::Ptr data(new golem::Manager::Data(context));
	static_cast<golem::Manager::Data*>(data.get())->create(*this);
	return data;
}

golem::Manager::Data::Data(golem::Context &context) : data::Data(context), owner(nullptr) {
}

void golem::Manager::Data::create(const Desc& desc) {
	data::Data::create(desc);
}

void golem::Manager::Data::setOwner(Manager* owner) {
	this->owner = owner;
}

void golem::Manager::Data::createRender() {
	const data::Item::Map::const_iterator ptr = getItem<data::Item::Map::const_iterator>();
	if (ptr != itemMap.end() && getView().bItems)
		ptr->second->createRender();
}

//------------------------------------------------------------------------------

void golem::Manager::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	golem::XMLContext* pxmlcontext = xmlcontext->getContextFirst("manager");

	try {
		handlers.clear();
		golem::XMLData(handlers, handlers.max_size(), pxmlcontext, "handler");
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}

	dataDesc->load(pxmlcontext->getContextFirst("data_template"));
	golem::XMLData("path", dataPath, pxmlcontext->getContextFirst("data_template"));
	golem::XMLData("ext", dataExt, pxmlcontext->getContextFirst("data_template"));

	try {
		dataSeq.clear();
		XMLGetValue(dataSeq, "data", "path", pxmlcontext);
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}

	try {
		poseMap.clear();
		XMLData(poseMap, poseMap.max_size(), const_cast<golem::XMLContext*>(xmlcontext), "pose");
	}
	catch (const golem::MsgXMLParserNameNotFound&) {}
}

void golem::Manager::Desc::run(int argc, char *argv[]) {
	// Setup application
	load(*context(), xmlcontext());

	Manager *pManager = dynamic_cast<Manager*>(scene()->createObject(*this)); // throws
	if (!pManager)
		throw Message(Message::LEVEL_CRIT, "golem::Manager::Desc::run(): Unable to cast to Manager");

	// Random number generator seed
	context()->info("Random number generator seed %d\n", context()->getRandSeed()._U32[0]);

	try {
		pManager->main();
	}
	catch (...) {
	}
	context()->info("Good bye!\n");
	scene()->releaseObject(*pManager);
};

//------------------------------------------------------------------------------

std::string golem::Manager::Data::toString(data::Data::Map::const_iterator data, bool showItem) {
	if (!showItem)
		return golem::makeString("Data %s", data->first.c_str());
	const data::Item::Map::const_iterator item = to<Data>(data)->getItem<data::Item::Map::const_iterator>(false);
	return item != to<Data>(data)->itemMap.end() ?
		golem::makeString("Data %s: handler %s, label %s/%s, item %u", data->first.c_str(), item->second->getHandler().getID().c_str(), item->first.c_str(), to<Data>(data)->getView().ptrLabel.c_str(), to<Data>(data)->getView().ptrItem + 1) :
		golem::makeString("Data %s: no items", data->first.c_str());

}

//------------------------------------------------------------------------------

golem::Manager::Manager(Scene &scene) : Object(scene), Menu(scene.getContext(), *this), menuLevel(0), optionAllData(0) {
	generatePtrIndex = 0;
	convertPtrIndex = 0;
	transformPtrIndex = 0;
	transformProcess = 0;
}
	
void golem::Manager::create(const Desc& desc) {
	desc.assertValid(Assert::Context("Manager::Desc."));

	// create object
	(void)Object::create(desc); // throws if Object::Desc is invalid

	dataDesc = desc.dataDesc;

	// handlers
	handlerMap.clear();
	for (golem::Library::Path::Seq::const_iterator i = desc.handlers.begin(); i != desc.handlers.end(); ++i) {
		data::Handler::Ptr handler = golem::Library::Desc::loadLibrary<data::Handler::Desc>(context, *i)->create(context);
		handler->setFileDesc(dataDesc->file); // overwrite file description
		handlerMap.insert(std::make_pair(handler->getID(), handler));
	}

	// data
	dataPath = desc.dataPath;
	dataImportPath = desc.dataPath;
	dataExportPath = desc.dataPath;
	dataExt = desc.dataExt;
	dataSeq = desc.dataSeq;
	dataMap.clear();
	dataCurrentPtr = dataMap.end(); // safe - rendering not working yet
	dataBundleMode = desc.dataBundleMode;

	// poses
	poseMap = desc.poseMap;

	// top menu help accessible using global key '?'
	scene.getHelp().insert(Scene::StrMapVal("040", "  {}                                      select data\n"));
	scene.getHelp().insert(Scene::StrMapVal("041", "  []                                      select item\n"));
	scene.getHelp().insert(Scene::StrMapVal("050", "  `                                       mode view\n"));
	scene.getHelp().insert(Scene::StrMapVal("051", "  1                                       mode item\n"));
	scene.getHelp().insert(Scene::StrMapVal("052", "  2                                       mode label\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F0", "  D                                       menu data\n"));
	scene.getHelp().insert(Scene::StrMapVal("0F1", "  I                                       menu item\n"));

	// top menu control and commands
	menuCtrlMap.insert(std::make_pair("", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		//desc = "Press a key to: access menu (D)ata/(I)tem, change index of data({})/item([]), change view mode shared(`)/item(1)/label(2)...";
	}));
	menuCmdMap.insert(std::make_pair("`", [&] () {
		dataBundleMode = !dataBundleMode;
		context.write("Mode: %s\n", dataBundleMode ? "data view" : "shared view");
		scene.setOpenGL(to<Data>(dataCurrentPtr)->getView().openGL);
		createRender();
	}));
	menuCmdMap.insert(std::make_pair("1", [&] () {
		Data::View& view = to<Data>(dataCurrentPtr)->getView();
		view.bItems = !view.bItems;
		context.write("Mode: %s\n", view.bItems ? "data items" : "data");
		createRender();
	}));
	menuCmdMap.insert(std::make_pair("2", [&] () {
		Data::View& view = to<Data>(dataCurrentPtr)->getView();
		if (view.bItems) {
			view.bLabels = !view.bLabels;
			context.write("Mode: %s\n", view.bLabels ? "label selection" : "index selection");
			createRender();
		}
	}));
	menuCmdMap.insert(std::make_pair("[", [&] () {
		Data::View& view = to<Data>(dataCurrentPtr)->getView();
		if (view.bItems) {
			RenderBlock renderBlock(*this);
			{
				golem::CriticalSectionWrapper cswData(scene.getCS());
				if (view.bLabels) {
					if (!to<Data>(dataCurrentPtr)->itemMap.empty()) {
						if (view.ptrLabel == Data::LABEL_DEFAULT)
							view.ptrLabel = to<Data>(dataCurrentPtr)->itemMap.begin()->first;
						else {
							const std::pair<data::Item::Map::const_iterator, data::Item::Map::const_iterator> range = to<Data>(dataCurrentPtr)->itemMap.equal_range(view.ptrLabel);
							view.ptrLabel = range.first == range.second ? to<Data>(dataCurrentPtr)->itemMap.begin()->first : range.first == to<Data>(dataCurrentPtr)->itemMap.begin() ? Data::LABEL_DEFAULT : (--const_cast<data::Item::Map::const_iterator&>(range.first))->first;
						}
					}
				}
				else
					view.ptrItem = view.ptrItem > 0 ? view.ptrItem - 1 : view.ptrItem;

				(void)to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
			}
		}
		context.write("%s\n", Data::toString(dataCurrentPtr).c_str());
	}));
	menuCmdMap.insert(std::make_pair("]", [&] () {
		Data::View& view = to<Data>(dataCurrentPtr)->getView();
		if (view.bItems) {
			RenderBlock renderBlock(*this);
			{
				golem::CriticalSectionWrapper cswData(scene.getCS());
				if (view.bLabels) {
					if (!to<Data>(dataCurrentPtr)->itemMap.empty()) {
						if (view.ptrLabel == Data::LABEL_DEFAULT)
							view.ptrLabel = to<Data>(dataCurrentPtr)->itemMap.rbegin()->first;
						else {
							const std::pair<data::Item::Map::const_iterator, data::Item::Map::const_iterator> range = to<Data>(dataCurrentPtr)->itemMap.equal_range(view.ptrLabel);
							view.ptrLabel = range.first == range.second ? to<Data>(dataCurrentPtr)->itemMap.rbegin()->first : range.second == to<Data>(dataCurrentPtr)->itemMap.end() ? Data::LABEL_DEFAULT : range.second->first;
						}
					}
				}
				else
					view.ptrItem = view.ptrItem + 1;

				(void)to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
			}
			context.write("%s\n", Data::toString(dataCurrentPtr).c_str());
		}
	}));
	menuCmdMap.insert(std::make_pair("{", [&] () {
		setCurrentDataPtr(dataCurrentPtr == dataMap.begin() ? --dataMap.end() : --data::Data::Map::iterator(dataCurrentPtr));
		context.write("%s\n", Data::toString(dataCurrentPtr).c_str());
	}));
	menuCmdMap.insert(std::make_pair("}", [&] () {
		setCurrentDataPtr(dataCurrentPtr == --dataMap.end() ? dataMap.begin() : ++data::Data::Map::iterator(dataCurrentPtr));
		context.write("%s\n", Data::toString(dataCurrentPtr).c_str());
	}));
	menuCmdMap.insert(std::make_pair(" ", [&] () {
		createRender();
	}));

	// data menu control and commands
	menuCtrlMap.insert(std::make_pair("D", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		desc = "Press a key to: create (N)ew/(S)ave/(L)oad/(R)emove data, print (D)ata/(H)andlers...";
	}));
	menuCmdMap.insert(std::make_pair("DD", [=] () {
		for (Data::Map::const_iterator i = dataMap.begin(); i != dataMap.end(); ++i)
			context.write("<data path=\"%s\"/>\n", i->first.c_str());
	}));
	menuCmdMap.insert(std::make_pair("DH", [=] () {
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i)
			context.write("<data library_path=\"%s\" config_path=\"%s\"/>\n", i->second->getPath().library.c_str(), i->second->getPath().config.c_str());
	}));
	menuCmdMap.insert(std::make_pair("DN", [&] () {
		readPath("Enter data path to create: ", dataPath);
		if (!isExt(dataPath, dataExt))
			dataPath += dataExt;
		if (dataMap.find(dataPath) != dataMap.end())
			throw Cancel("Non-empty data bundle name");
		Data::Ptr data = createData();
		RenderBlock renderBlock(*this);
		scene.getOpenGL(to<Data>(dataCurrentPtr)->getView().openGL); // set current view
		scene.getOpenGL(to<Data>(data)->getView().openGL); // set view of the new data
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			//dataMap.erase(dataPath);
			dataCurrentPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(dataPath, data));
		}
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("DS", [&] () {
		// save data
		const bool allData = dataMap.size() > 1 && (optionAllData = option(optionAllData, "Save all data: ", {"YES", "NO"})) == 0;
		if (allData) {
			for (Data::Map::iterator i = dataMap.begin(); i != dataMap.end(); ++i) {
				context.write("Saving %s...\n", i->first.c_str());
				i->second->save(i->first);
			}
		}
		else {
			std::string path = dataCurrentPtr->first;
			readPath("Enter data path to save: ", path, dataExt.c_str());
			if (getExt(path).empty())
				path += dataExt;
			to<Data>(dataCurrentPtr)->save(path);
		}
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("DL", [&] () {
		// load data
		readPath("Enter data path or directory to load: ", dataPath, dataExt.c_str());
		std::function<void(const std::string&, const std::string&)> load = [&] (const std::string& path, const std::string& ext) {
			if (!boost::filesystem::is_directory(path)) {
				if (!isExt(path, ext))
					return;
				try {
					dataPath = path;
#ifdef WIN32
					boost::replace_all(dataPath, "\\", "/");
#endif
					context.write("Loading %s...\n", dataPath.c_str());
					Data::Ptr data = createData();
					data->load(dataPath, handlerMap);
					RenderBlock renderBlock(*this);
					scene.getOpenGL(to<Data>(dataCurrentPtr)->getView().openGL); // set current view
					scene.getOpenGL(to<Data>(data)->getView().openGL); // set view of the new data
					{
						golem::CriticalSectionWrapper cswData(scene.getCS());
						dataMap.erase(dataPath);
						dataCurrentPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(dataPath, data));
					}
				}
				catch (const std::exception& ex) {
					context.write("%s\n", ex.what());
				}
				return;
			}
			for (boost::filesystem::directory_iterator i(path), end; i != end; ++i)
				load(i->path().string(), ext);
		};
		load(dataPath, dataExt);
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("DR", [&] () {
		if (dataMap.size() > 1) {
			RenderBlock renderBlock(*this);
			context.write("Removing %s...\n", dataCurrentPtr->first.c_str());
			{
				golem::CriticalSectionWrapper cswData(scene.getCS());
				dataMap.erase(dataCurrentPtr++);
				if (dataCurrentPtr == dataMap.end()) dataCurrentPtr = dataMap.begin();
			}
			scene.setOpenGL(to<Data>(dataCurrentPtr)->getView().openGL);
			context.write("%s\n", Data::toString(dataCurrentPtr).c_str());
		}
	}));

	// item menu control and commands
	menuCtrlMap.insert(std::make_pair("I", [=] (MenuCmdMap& menuCmdMap, std::string& desc) {
		if (to<Data>(dataCurrentPtr)->itemMap.empty()) {
			desc = "Press a key to: (G)enerate/(T)ransform/(I)mport/c(O)py items...";
			menuCmdMap.erase("IC");
			menuCmdMap.erase("IE");
			menuCmdMap.erase("IM");
			menuCmdMap.erase("IR");
		}
		else
			desc = "Press a key to: (G)enerate/(C)onvert/(T)ransform/(I)mport/(E)xport/c(O)py/rena(M)e/(R)emove items...";
	}));
	menuCmdMap.insert(std::make_pair("IG", [&]() {
		// find handlers supporting data::Generate
		typedef std::vector<std::pair<data::Handler*, data::Generate*>> GenerateMap;
		GenerateMap generateMap;
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
			data::Generate* generate = is<data::Generate>(i);
			if (generate) generateMap.push_back(std::make_pair(i->second.get(), generate));
		}
		if (generateMap.empty())
			throw Cancel("No handlers support Generate interface");
		// pick up handler
		GenerateMap::const_iterator generatePtr = generateMap.begin() + generatePtrIndex;
		select(generatePtr, generateMap.begin(), generateMap.end(), "Generate:\n", [](GenerateMap::const_iterator ptr) -> std::string {
			return std::string("Handler: ") + ptr->first->getID();
		});
		generatePtrIndex = U32(generatePtr - generateMap.begin());

		// items' prefix
		std::string dataItemPrefix;
		readString("Enter prefix: ", dataItemPrefix);

		// generate
		RenderBlock renderBlock(*this);
		UI::addCallback(*this, generatePtr->second);
		data::Item::Map items;
		generatePtr->second->generate(items);
		for (data::Item::Map::const_iterator i = items.begin(); i != items.end(); ++i) {
			golem::CriticalSectionWrapper cswData(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemPrefix + i->first, i->second));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// done!
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("IC", [&]() {
		// check if the current item supports Convert
		data::Item::Map::const_iterator input = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		data::Convert* convert = is<data::Convert>(input);
		if (!convert)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Convert interface", Data::toString(dataCurrentPtr).c_str());

		// find matching handlers
		typedef std::vector<data::Handler*> ConvertMap;
		ConvertMap convertMap;
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i)
			if (convert->isConvertSupported(*i->second))
				convertMap.push_back(i->second.get());

		// pick up handler
		ConvertMap::const_iterator convertPtr = convertMap.begin() + convertPtrIndex;
		select(convertPtr, convertMap.begin(), convertMap.end(), "Convert:\n", [] (ConvertMap::const_iterator ptr) -> std::string {
			return std::string("Handler: ") + (*ptr)->getID();
		});
		convertPtrIndex = U32(convertPtr - convertMap.begin());

		readString("Enter label: ", dataItemLabel);

		// convert
		data::Item::Ptr output = convert->convert(**convertPtr);
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemLabel, output));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		// done!
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("IT", [&]() {
		// find handlers supporting data::Transform
		typedef std::vector<std::pair<data::Handler*, data::Transform*>> TransformMap;
		TransformMap transformMap;
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
			data::Transform* transform = is<data::Transform>(i);
			if (transform) transformMap.push_back(std::make_pair(i->second.get(), transform));
		}
		if (transformMap.empty())
			throw Cancel("No handlers support Transform interface");
		
		// pick up handler
		TransformMap::const_iterator transformPtr = transformMap.begin() + transformPtrIndex;
		select(transformPtr, transformMap.begin(), transformMap.end(), "Transform:\n", [] (TransformMap::const_iterator ptr) -> std::string {
			std::stringstream str;
			for (StringSeq::const_iterator i = ptr->second->getTransformInterfaces().begin(); i != ptr->second->getTransformInterfaces().end(); ++i) str << *i << " ";
			return std::string("Handler: ") + ptr->first->getID() + std::string(", Item interfaces: ") + str.str();
		});
		transformPtrIndex = U32(transformPtr - transformMap.begin());

		// Processing options
		transformProcess = (U32)option((size_t)transformProcess, "Process: ", { "All", "Label", "Single" });

		// target label
		if (transformProcess == 0)
			readString("Enter label: ", dataItemLabel);

		// auto save
		const bool autoSave = option(0, "Auto save: ", { "NO", "YES" }) == 1;

		ScopeGuard scopeGuard([&] () {
			UI::addCallback(*this, getCurrentHandler());
			createRender();
		});

		// transform
		processItems([&](const Data::Selection::List& list) {
			// prepare processing list of items
			typedef std::multimap<std::string, data::Item::List> ItemMap;
			ItemMap itemMap;
			for (Data::Selection::List::const_iterator item = list.begin(); item != list.end(); ++item) {
				if (transformProcess == 0) {
					// all items together
					ItemMap::iterator ptr = itemMap.find(dataItemLabel);
					(ptr != itemMap.end() ? ptr : itemMap.insert(itemMap.end(), std::make_pair(dataItemLabel, data::Item::List())))->second.push_back(item->getPtr());
				}
				else if (transformProcess == 1) {
					// label-grouped items
					ItemMap::iterator ptr = itemMap.find(item->getPtr()->first);
					(ptr != itemMap.end() ? ptr : itemMap.insert(itemMap.end(), std::make_pair(item->getPtr()->first, data::Item::List())))->second.push_back(item->getPtr());
				}
				else if (transformProcess == 2) {
					// each item separately
					itemMap.insert(itemMap.end(), std::make_pair(item->getPtr()->first, data::Item::List()))->second.push_back(item->getPtr());
				}
			}

			// transform
			EnableKeyboardMouse enableKeyboardMouse(*this);
			ScopeGuard guard([&] () { setInputEnabled(true); });
			for (ItemMap::const_iterator itemList = itemMap.begin(); itemList != itemMap.end(); ++itemList) {
				RenderBlock renderBlock(*this);
				if (!autoSave)
					UI::addCallback(*this, transformPtr->first);
				if (itemList != itemMap.begin())
					setInputEnabled(false);

				data::Item::Ptr item = transformPtr->second->transform(itemList->second);
				{
					golem::CriticalSectionWrapper cswData(scene.getCS());
					const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemList->first, item));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
				}
			}

			// save
			if (autoSave) {
				context.info("Saving current data bundle to: %s\n", dataCurrentPtr->first.c_str());
				to<Data>(dataCurrentPtr)->save(dataCurrentPtr->first);
			}
		}, [=](const data::Item& item) -> bool { return transformPtr->second->isTransformSupported(item); }, false, false, true);
		// done!
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("II", [&] () {
		// find handlers supporting data::Import
		typedef std::vector<std::pair<data::Handler*, data::Import*>> ImportMap;
		ImportMap importMap;
		for (data::Handler::Map::const_iterator i = handlerMap.begin(); i != handlerMap.end(); ++i) {
			data::Import* import = is<data::Import>(i);
			if (import) importMap.push_back(std::make_pair(i->second.get(), import));
		}
		if (importMap.empty())
			throw Cancel("No handlers support Import interface");
		
		// pick up handler
		ImportMap::const_iterator importPtr = importMap.begin();
		select(importPtr, importMap.begin(), importMap.end(), "Import:\n", [] (ImportMap::const_iterator ptr) -> std::string {
			std::stringstream str;
			for (StringSeq::const_iterator i = ptr->second->getImportFileTypes().begin(); i != ptr->second->getImportFileTypes().end(); ++i) str << *i << " ";
			return std::string("Handler: ") + ptr->first->getID() + std::string(", File types: ") + str.str();
		});

		readPath("Enter file path: ", dataImportPath, importPtr->second->getImportFileTypes());
		readString("Enter label: ", dataItemLabel);

		RenderBlock renderBlock(*this);
		UI::addCallback(*this, importPtr->first);

		// load/import item
		data::Item::Ptr item = importPtr->second->import(dataImportPath);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemLabel, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("IE", [&]() {
		// check if the current item supports Convert
		data::Item::Map::const_iterator item = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);
		data::Export* exportt = is<data::Export>(item);
		if (!exportt)
			throw Message(Message::LEVEL_ERROR, "Item %s does not support Export interface", Data::toString(dataCurrentPtr).c_str());

		// pick up type
		if (!exportt->getExportFileTypes().empty()) {
			StringSeq::const_iterator exportPtr = exportt->getExportFileTypes().begin();
			select(exportPtr, exportt->getExportFileTypes().begin(), exportt->getExportFileTypes().end(), "Export:\n", [](StringSeq::const_iterator ptr) -> std::string {
				return std::string("File type: ") + *ptr;
			});
			readPath("Enter file path: ", dataExportPath, StringSeq(&*exportPtr, &*exportPtr + 1));
		}

		//RenderBlock renderBlock(*this);
		// export item
		exportt->exportt(dataExportPath);
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("IO", [&]() {
		// transform
		processItems([&] (const Data::Selection::List& list) {
			// transform
			for (Data::Selection::List::const_iterator itemList = list.begin(); itemList != list.end(); ++itemList) {
				RenderBlock renderBlock(*this);
				data::Item::Ptr item = itemList->getPtr()->second->clone();
				{
					golem::CriticalSectionWrapper cswData(scene.getCS());
					const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(itemList->getPtr()->first, item));
					Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
				}
			}
		}, nullptr, false, false, true);
		// done!
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("IM", [&]() {
		// check if the current item supports Convert
		data::Item::Map::const_iterator input = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(true);

		readString("Enter label: ", dataItemLabel);

		// rename item
		data::Item::Ptr item = input->second;
		RenderBlock renderBlock(*this);
		{
			golem::CriticalSectionWrapper cswData(scene.getCS());
			to<Data>(dataCurrentPtr)->itemMap.erase(input);
			const data::Item::Map::iterator ptr = to<Data>(dataCurrentPtr)->itemMap.insert(to<Data>(dataCurrentPtr)->itemMap.end(), data::Item::Map::value_type(dataItemLabel, item));
			Data::View::setItem(to<Data>(dataCurrentPtr)->itemMap, ptr, to<Data>(dataCurrentPtr)->getView());
		}
		context.write("Done!\n");
	}));
	menuCmdMap.insert(std::make_pair("IR", [&]() {
		ScopeGuard scopeGuard([&] () {
			UI::addCallback(*this, getCurrentHandler());
			createRender();
		});
		// remove
		processItems([&] (const Data::Selection::List& list) {
			for (Data::Selection::List::const_iterator i = list.begin(); i != list.end(); ++i) {
				context.write("Removing: %s...\n", dataCurrentPtr->first.c_str(), i->getPtr()->second->getHandler().getType().c_str(), i->getPtr()->first.c_str(), i->getLabel().c_str(), (i->getItem() + 1));
				UI::removeCallback(*this, &i->getPtr()->second->getHandler());
				golem::CriticalSectionWrapper cswData(scene.getCS());
				dataCurrentPtr->second->itemMap.erase(i->getPtr());
			}
		});
	}));
}

//------------------------------------------------------------------------------

ConfigMat34::Range golem::Manager::selectPoseRange(const ConfigMat34::Map& poseMap, StringIndexMap::value_type* index) const {
	StringSeq names;
	for (ConfigMat34::Map::const_iterator i = poseMap.begin(); i != poseMap.end(); i = poseMap.upper_bound(i->first))
		names.push_back(i->first);
	if (names.empty())
		throw Cancel("Poses not available!");

	StringSeq::const_iterator namePtr = names.begin() + (index ? index->second : 0);

	select(namePtr, names.begin(), names.end(), "Select pose:\n", [=] (StringSeq::const_iterator ptr) -> std::string {
		const ConfigMat34::Range range = poseMap.equal_range(*ptr);
		return *ptr + " (" + std::to_string(std::distance(range.first, range.second)) + ")";
	});

	if (index) {
		const_cast<std::string&>(index->first) = *namePtr;
		index->second = std::min(StringIndexMap::mapped_type(namePtr - names.begin()), StringIndexMap::mapped_type(names.size() - 1));
	}

	return poseMap.equal_range(*namePtr);
}

//------------------------------------------------------------------------------

data::Data::Ptr golem::Manager::createData() const {
	data::Data::Ptr data(dataDesc->create(context));
	to<Data>(data)->setOwner(const_cast<Manager*>(this));
	return data;
}

//------------------------------------------------------------------------------

bool golem::Manager::isTopMenuLevel() const {
	return menuLevel < 1;
}

data::Item* golem::Manager::getCurrentItem() const {
	if (dataCurrentPtr != dataMap.end() && to<Data>(dataCurrentPtr)->getView().bItems) {
		const data::Item::Map::const_iterator ptr = to<Data>(dataCurrentPtr)->getItem<data::Item::Map::const_iterator>(false);
		if (ptr != to<Data>(dataCurrentPtr)->itemMap.end())
			return ptr->second.get();
	}
	return nullptr;
}

data::Handler* golem::Manager::getCurrentHandler() const {
	data::Item* item = getCurrentItem();
	return item ? &item->getHandler() : nullptr;
}

data::Data::Map::iterator golem::Manager::getCurrentDataPtr() const {
	return dataCurrentPtr;
}

void golem::Manager::setCurrentDataPtr(data::Data::Map::iterator dataPtr) {
	RenderBlock renderBlock(*this);
	scene.getOpenGL(to<Data>(dataCurrentPtr)->getView().openGL); // remember view
	{
		golem::CriticalSectionWrapper cswData(scene.getCS());
		dataCurrentPtr = dataPtr;
	}
	scene.setOpenGL(to<Data>(dataCurrentPtr)->getView().openGL);
}

//------------------------------------------------------------------------------

void golem::Manager::createRender() {
	// create render buffer to be displayed by 
	golem::CriticalSectionWrapper cswData(scene.getCS());
	if (dataCurrentPtr != dataMap.end())
		to<Data>(dataCurrentPtr)->createRender();
}

void golem::Manager::render() const {
	for (RendererSet::const_iterator i = rendererSet.begin(); i != rendererSet.end(); ++i)
		(*i)->render();
}

void golem::Manager::customRender() const {
	for (RendererSet::const_iterator i = rendererSet.begin(); i != rendererSet.end(); ++i)
		(*i)->customRender();
}

//------------------------------------------------------------------------------

void golem::Manager::mouseHandler(int button, int state, int x, int y) {
	// dispatch messages only to the current item/handler providing it supports golem::UIKeyboardMouse interface and only at the top menu level
	golem::UIKeyboardMouse* uiKeyboardMouse = is<golem::UIKeyboardMouse>(getCurrentHandler());
	if (isTopMenuLevel() && uiKeyboardMouse)
		uiKeyboardMouse->mouseHandler(button, state, x, y);
}

void golem::Manager::motionHandler(int x, int y) {
	// dispatch messages only to the current item/handler providing it supports golem::UIKeyboardMouse interface and only at the top menu level
	golem::UIKeyboardMouse* uiKeyboardMouse = is<golem::UIKeyboardMouse>(getCurrentHandler());
	if (isTopMenuLevel() && uiKeyboardMouse)
		uiKeyboardMouse->motionHandler(x, y);
}

void golem::Manager::keyboardHandler(int key, int x, int y) {
	// dispatch messages only to the current item/handler providing it supports golem::UIKeyboardMouse interface and only at the top menu level
	golem::UIKeyboardMouse* uiKeyboardMouse = is<golem::UIKeyboardMouse>(getCurrentHandler());
	if (isTopMenuLevel() && uiKeyboardMouse)
		uiKeyboardMouse->keyboardHandler(key, x, y);
}

//------------------------------------------------------------------------------

int golem::Manager::waitKey(golem::MSecTmU32 timeOut) {
	const int key = universe.waitKey(timeOut);
	if (universe.interrupted())
		throw Exit();
	return key;
}

void golem::Manager::getOpenGL(OpenGL& openGL) const {
	scene.getOpenGL(openGL);
}

void golem::Manager::setOpenGL(const OpenGL& openGL) {
	scene.setOpenGL(openGL);
}

golem::CriticalSection &golem::Manager::getCS() const {
	return scene.getCS();
}

void golem::Manager::clearRenderers() {
	golem::CriticalSectionWrapper cswRenderer(scene.getCS());
	rendererSet.clear();
}

void golem::Manager::addRenderer(const golem::UIRenderer* uiRenderer) {
	// it is called by plugins only via UI
	golem::CriticalSectionWrapper cswRenderer(scene.getCS());
	if (uiRenderer)
		rendererSet.insert(uiRenderer);
}

void golem::Manager::removeRenderer(const golem::UIRenderer* uiRenderer) {
	// it is called by plugins only via UI
	golem::CriticalSectionWrapper cswRenderer(scene.getCS());
	rendererSet.erase(uiRenderer);
}

void golem::Manager::requestRender(const golem::UIRenderer* uiRenderer) {
	// it is called by plugins only via UI
	golem::CriticalSectionWrapper cswRenderer(scene.getCS());
	RendererSet::const_iterator ptr = rendererSet.find(uiRenderer);
	if (ptr != rendererSet.end()) {
		golem::CriticalSectionWrapper cswData(scene.getCS());
		data::Item* item = golem::Manager::getCurrentItem();
		if (uiRenderer == is<golem::UIRenderer>(&item->getHandler()))
			item->createRender();
	}
}

//------------------------------------------------------------------------------

void golem::Manager::processItems(Data::Selection::Process process, TestProcessItems testProcessItems, bool allData, bool enableEmpty, bool enableMultichoice) {
	// protect current view
	//Data::View view = to<Data>(dataCurrentPtr)->getView();
	//ScopeGuard guard([&] {
	//	{
	//		golem::CriticalSectionWrapper cswData(scene.getCS());
	//		to<Data>(dataCurrentPtr)->getView() = view;
	//	}
	//	createRender();
	//});
	// local data
	//if (to<Data>(dataCurrentPtr)->itemMap.empty())
	//	throw Message(Message::LEVEL_ERROR, "Manager::processItems(): No items to select");

	Data::Selection::List list;
	Data::Selection::Map map;
	
	// target data
	const Data::Map::iterator targetDataPtr = dataCurrentPtr;

	// create menu
	bool stop = false;
	MenuCtrlMap menuCtrlMap;
	menuCtrlMap.insert(std::make_pair("", [&] (MenuCmdMap& menuCmdMap, std::string& desc) {}));
	MenuCmdMap menuCmdMap;
	menuCmdMap.insert(std::make_pair("\x0D", [&]() { if (!list.empty() || enableEmpty) stop = true; }));
	menuCmdMap.insert(std::make_pair("+", [&] () {
		if (to<Data>(dataCurrentPtr)->itemMap.empty()) {
			context.write("Manager::processItems(): No items to select\n");
			return;
		}
		Data::Selection selection(to<Data>(dataCurrentPtr)->itemMap, to<Data>(dataCurrentPtr)->getView());
		if (!testProcessItems || testProcessItems(*selection.getPtr()->second)) {
			if (enableMultichoice ? Data::Selection::add(selection, list) : Data::Selection::add(selection, list, map))
				Data::Selection::print(list.begin(), list.end(), context);
		}
		else
			context.write("Unsupported type %s\n", selection.getType().c_str());
	}));
	menuCmdMap.insert(std::make_pair("\x6C", [&] () { // <ins>
		if (to<Data>(dataCurrentPtr)->itemMap.empty()) {
			context.write("Manager::processItems(): No items to select\n");
			return;
		}
		U32 items = 0;
		Data::View view = to<Data>(dataCurrentPtr)->getView();
		for (view.ptrItem = 0; view.ptrItem < U32(to<Data>(dataCurrentPtr)->itemMap.size()); ++view.ptrItem) {
			Data::Selection selection(to<Data>(dataCurrentPtr)->itemMap, view);
			if ((!testProcessItems || testProcessItems(*selection.getPtr()->second)) && (enableMultichoice ? Data::Selection::add(selection, list) : Data::Selection::add(selection, list, map)))
				++items;
		}
		if (items > 0)
			Data::Selection::print(list.begin(), list.end(), context);
	}));
	menuCmdMap.insert(std::make_pair("-", [&] () {
		if (to<Data>(dataCurrentPtr)->itemMap.empty()) {
			context.write("Manager::processItems(): No items to select\n");
			return;
		}
		if (enableMultichoice ? Data::Selection::remove(Data::Selection(to<Data>(dataCurrentPtr)->itemMap, to<Data>(dataCurrentPtr)->getView()), list) : Data::Selection::remove(Data::Selection(to<Data>(dataCurrentPtr)->itemMap, to<Data>(dataCurrentPtr)->getView()), list, map))
			Data::Selection::print(list.begin(), list.end(), context);
	}));
	menuCmdMap.insert(std::make_pair("\x7F", [&] () { // <del>
		list.clear();
		map.clear();
		Data::Selection::print(list.begin(), list.end(), context);
	}));
	menuCmdMap.insert(std::make_pair("2", [&] () { this->Manager::menuCmdMap["2"](); }));
	menuCmdMap.insert(std::make_pair("[", [&]() { this->Manager::menuCmdMap["["](); }));
	menuCmdMap.insert(std::make_pair("]", [&]() { this->Manager::menuCmdMap["]"](); }));

	// select items
	if (allData)
		context.write("Press a key to: accept(<Enter>), item add(+)/remove(-), all items clear(<Del>)/add(<Ins>), change index of item([]), change view mode label(2)...\n");
	else {
		menuCmdMap.insert(std::make_pair("{", [&]() { this->Manager::menuCmdMap["{"](); }));
		menuCmdMap.insert(std::make_pair("}", [&]() { this->Manager::menuCmdMap["}"](); }));
		context.write("Press a key to: accept(<Enter>), item add(+)/remove(-), all items clear(<Del>)/add(<Ins>), change index of data({})/item([]), change view mode label(2)...\n");
	}
	
	// select
	while (!stop) {
		U32 menuLevel = 0;
		golem::Menu::menu(menuCtrlMap, menuCmdMap, menuLevel);
	}
	
	// run
	if (allData) {
		for (Data::Map::iterator data = dataMap.begin(); data != dataMap.end(); ++data) {
			setCurrentDataPtr(data);
			Data::Selection::List outList;
			std::for_each(list.begin(), list.end(), [&](const Data::Selection& inp) {
				const Data::Selection out(dataCurrentPtr->second->itemMap, inp); // re-map selection onto new data
				if (out.isValid())
					outList.push_back(out);
				else
					context.write("Invalid selection: Data %s, handler %s, label */%s, item %u\n", dataCurrentPtr->first.c_str(), inp.getType().c_str(), inp.getLabel().c_str(), inp.getItem());
			});
			try {
				if (!outList.empty()) process(outList);
			}
			catch (const std::exception& ex) {
				context.write("%s\n", ex.what());
			}
		}
	}
	else {
		setCurrentDataPtr(targetDataPtr);
		process(list);
	}
}
		

//------------------------------------------------------------------------------

/** tiny::Tiny */
void golem::Manager::menu() {
	try {
		// run menu
		MenuCmdMap menuCmdMap = this->menuCmdMap;
		menuLevel = 0;
		golem::Menu::menu(menuCtrlMap, menuCmdMap, menuLevel);
	}
	catch (const Cancel& cancel) {
		context.write("%s\n", cancel.what());
	}
	catch (const golem::Message& msg) {
		context.write(msg);
		if (msg.level() == Message::LEVEL_CRIT) throw;
	}
	catch (const std::exception& ex) {
		context.error("%s\n", ex.what());
	}
}


void golem::Manager::main(bool runMenu) {
	// Initialise data collection
	for (StringSeq::const_iterator i = dataSeq.begin(); i != dataSeq.end(); ++i) {
		try {
			data::Data::Ptr data = createData();
			context.write("Loading %s...\n", i->c_str());
			data->load(*i, handlerMap);
			scene.getOpenGL(to<Data>(data)->getView(true).openGL);
			
			golem::CriticalSectionWrapper cswData(scene.getCS());
			dataCurrentPtr = dataMap.insert(dataMap.begin(), Data::Map::value_type(*i, data));
		}
		catch (const std::exception& ex) {
			context.write("%s\n", ex.what());
		}
	}
	
	{
		golem::CriticalSectionWrapper cswData(scene.getCS());
		if (dataMap.empty())
			dataMap.insert(Data::Map::value_type(dataPath, createData()));
		dataCurrentPtr = dataMap.begin();
		scene.getOpenGL(to<Data>(dataCurrentPtr)->getView(true).openGL);
		scene.getOpenGL(dataView.openGL);
	}
	
	// set render buffer
	UI::addCallback(*this, getCurrentHandler());
	createRender();

	// I am ready!
	if (runMenu)
		context.write("Ready!\n");

	// main loop
	while (runMenu)
		menu();
}

//------------------------------------------------------------------------------
