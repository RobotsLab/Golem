/** @file Manager.h
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
#ifndef _GOLEM_APP_MANAGER_H_
#define _GOLEM_APP_MANAGER_H_

//------------------------------------------------------------------------------

#include <Golem/App/Application.h>
#include <Golem/Tools/Menu.h>
#include <Golem/Plugin/UI.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/Defs.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Manager. */
class Manager : public golem::Object, public golem::UICallback, public golem::Menu {
public:
	/** General string->index */
	typedef std::map<std::string, golem::U32> StringIndexMap;

	/** OpenGL renderer */
	typedef std::set<const golem::UIRenderer*> RendererSet;

	/** Data */
	class Data : public data::Data {
	public:
		/** Default label */
		static const std::string LABEL_DEFAULT;

		/** View */
		class View {
		public:
			/** Local: labels */
			bool bLabels;
			/** Local: items */
			bool bItems;
			
			/** Local: current label */
			std::string ptrLabel;
			/** Local: current item */
			golem::U32 ptrItem;

			/** Local: OpenGL settings */
			golem::OpenGL openGL;

			/** Creates data view */
			View() {
				setToDefault();
			}
			/** Default settings */
			void setToDefault() {
				bLabels = false;
				bItems = true;
				ptrLabel = LABEL_DEFAULT;
				ptrItem = 0;
				openGL.setToDefault();
			}
			/** Multimap item access using labels and indices */
			template <typename _Iter, typename _Index> static _Iter getPtr(_Iter begin, _Iter end, const _Index& index, bool modify = true) {
				_Index n = 0;
				for (; begin != end; ++n, ++begin)
					if (n == index)
						return begin;

				if (modify) const_cast<_Index&>(index) = n > 0 ? n - 1 : 0;
				return n > 0 ? --begin : begin;
			}
			/** Multimap item access using labels and indices */
			template <typename _Iter, typename _Map, typename _Key, typename _Index> static _Iter getPtr(_Map& map, const _Key& key, const _Index& index, bool modify = true) {
				const std::pair<_Iter, _Iter> range = map.equal_range(key);
				if (range.first != range.second)
					return getPtr(range.first, range.second, index, modify);
				if (modify) const_cast<_Index&>(index) = 0;
				return map.end();
			}
			/** Multimap item access using labels and indices */
			template <typename _Iter, typename _Map> static _Iter getItem(_Map& map, const std::string& label, const golem::U32& index, bool modify = true) {
				return label == LABEL_DEFAULT ? getPtr(map.begin(), map.end(), index, modify) : getPtr<_Iter>(map, label, index, modify);
			}
			/** Multimap item access using labels and indices */
			template <typename _Iter, typename _Map> static _Iter getItem(_Map& map, const View& view, bool modify = true) {
				return getItem<_Iter>(map, view.ptrLabel, view.ptrItem, modify);
			}

			/** Multimap item access using labels and indices */
			template <typename _Iter, typename _Index> static bool setPtr(_Iter begin, _Iter end, _Iter ptr, _Index& index) {
				_Index n = 0;
				for (_Iter i = begin; i != end; ++i, ++n)
					if (i == ptr) {
						index = n;
						return true;
					}
				return false;
			}
			/** Multimap item access using labels and indices */
			template <typename _Map, typename _Iter> static bool setItem(_Map& map, _Iter ptr, std::string& label, golem::U32& index) {
				const std::pair<_Iter, _Iter> range = map.equal_range(label);
				if (setPtr(range.first, range.second, ptr, index))
					return true;
				if (setPtr(map.begin(), map.end(), ptr, index)) {
					label = LABEL_DEFAULT;
					return true;
				}
				return false;
			}
			/** Multimap item access using labels and indices */
			template <typename _Map, typename _Iter> static bool setItem(_Map& map, _Iter ptr, View& view) {
				return setItem(map, ptr, view.ptrLabel, view.ptrItem);
			}
		};

		/** Item Selection */
		class Selection {
		public:
			typedef std::list<Selection> List;
			typedef std::map<const data::Item*, List::iterator> Map;
			typedef std::function<void(const List&)> Process;

			/** Creates empty/invalid selection */
			Selection();
			/** Creates selection from view */
			Selection(const data::Item::Map& itemMap, const View& view);
			/** Creates selection from other selection */
			Selection(const data::Item::Map& itemMap, const Selection& selection);

			/** Is valid? */
			bool isValid() const {
				return !type.empty();
			}
			/** Is valid? */
			void assertValid() const;

			/** Create selection */
			void create(const data::Item::Map& itemMap, const std::string& label, golem::U32 item);

			/** Append to the list or remove */
			static bool add(const Selection& selection, List& list);
			/** Append to the list or remove if already present */
			static bool add(const Selection& selection, List& list, Map& map);
			/** Append to the list whole label */
			static bool add(const data::Item::Map& itemMap, const std::string& label, List& list, Map& map);

			/** Append to the list or remove */
			static bool remove(const Selection& selection, List& list);
			/** Append to the list or remove if already present */
			static bool remove(const Selection& selection, List& list, Map& map);
			/** Remove from the list whole label */
			static bool remove(const data::Item::Map& itemMap, const std::string& label, List& list, Map& map);

			/** Print selection */
			static void print(List::const_iterator begin, List::const_iterator end, golem::Context& context);

			/** Label */
			const std::string& getLabel() const {
				return label;
			}
			/** Handler type */
			const std::string& getType() const {
				return type;
			}
			/** Item */
			golem::U32 getItem() const {
				return item;
			}

			/** Pointer */
			data::Item::Map::const_iterator getPtr() const {
				return ptr;
			}

		private:
			/** Label */
			std::string label;
			/** Handler type */
			std::string type;
			/** Item */
			golem::U32 item;
			
			/** Pointer */
			data::Item::Map::const_iterator ptr;
		};

		/** Data bundle description */
		class Desc : public data::Data::Desc {
		public:
			/** Creates the object from the description. */
			virtual data::Data::Ptr create(golem::Context &context) const;
		};

		/** Manager */
		virtual void setOwner(Manager* owner);

		/** View */
		View& getView(bool dataBundleMode = false) {
			return dataBundleMode || owner->dataBundleMode  ? view : owner->dataView;
		}
		/** View */
		const View& getView(bool dataBundleMode = false) const {
			return dataBundleMode || owner->dataBundleMode ? view : owner->dataView;
		}

		/** Creates render buffer of the bundle without items */
		virtual void createRender();

		/** Get current item */
		template <typename _Iter> _Iter getItem(bool modify = true) const {
			return View::getItem<_Iter>(itemMap, getView(), modify);
		}

		/** Data and item name */
		static std::string toString(data::Data::Map::const_iterator data, bool showItem = true);

	protected:
		/** Manager */
		Manager* owner;
		/** View */
		View view;

		/** Creates data bundle */
		void create(const Desc& desc);
		/** Creates data bundle */
		Data(golem::Context &context);
	};

	/** Manager description */
	class Desc : public golem::Object::Desc, public golem::Application {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Data handler libraries */
		Library::Path::Seq handlers;
		/** Data bundle description */
		data::Data::Desc::Ptr dataDesc;
		/** Data bundle file path */
		std::string dataPath;
		/** Data bundle file extension */
		std::string dataExt;
		/** Data bundle auto-load */
		StringSeq dataSeq;
		/** Data bundle mode */
		bool dataBundleMode;

		/** Pre-defined poses */
		ConfigMat34::Map poseMap;

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			Object::Desc::setToDefault();

			handlers.clear();
			dataDesc.reset(new Data::Desc);
			dataPath = "./data.xml";
			dataExt = ".xml";
			dataSeq.clear();
			dataBundleMode = false;

			poseMap.clear();
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(Object::Desc::isValid(), ac, "Object::Desc: invalid");

			//Assert::valid(!handlers.empty(), ac, "handlers: empty");
			for (Library::Path::Seq::const_iterator i = handlers.begin(); i != handlers.end(); ++i)
				i->assertValid(Assert::Context(ac, "handlers->"));

			Assert::valid(dataDesc != nullptr, ac, "dataDesc: null");
			dataDesc->assertValid(Assert::Context(ac, "dataDesc->"));

			Assert::valid(dataPath.length() > 0, ac, "dataPath: invalid");
			Assert::valid(dataExt.length() > 0, ac, "dataExt: invalid");

			for (ConfigMat34::Map::const_iterator i = poseMap.begin(); i != poseMap.end(); ++i)
				i->second.assertValid(Assert::Context(ac, "poseMap[]."));
		}
		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

	protected:
		GOLEM_CREATE_FROM_OBJECT_DESC1(Manager, golem::Object::Ptr, golem::Scene&)

		/** Runs Application */
		virtual void run(int argc, char *argv[]);
	};

	/** Runs main task */
	virtual void main(bool runMenu = true);

protected:
	/** Test process items function */
	typedef std::function<bool(const data::Item&)> TestProcessItems;

	/** Renderer blocker stops rendering the current item/handler within the scope of the defined variable */
	class RenderBlock {
	public:
		RenderBlock(Manager& manager) : manager(manager) {
			UI::removeCallback(manager, manager.getCurrentHandler());
		}
		~RenderBlock() {
			UI::addCallback(manager, manager.getCurrentHandler());
			manager.createRender();
		}
	private:
		Manager& manager;
	};

	/** Input blocker stops keyboard and mouse pugin interaction */
	class InputBlock {
	public:
		InputBlock(Manager& manager) : manager(manager) {
			manager.setInputEnabled(false);
		}
		~InputBlock() {
			manager.setInputEnabled(true);
		}
	private:
		Manager& manager;
	};

	/** Enables keyboard and mouse interaction for the current item/handler within the scope of the defined variable */
	class EnableKeyboardMouse {
	public:
		EnableKeyboardMouse(Manager& manager) : manager(manager), menuLevel(manager.menuLevel) {
			manager.menuLevel = 0;
		}
		~EnableKeyboardMouse() {
			manager.menuLevel = menuLevel;
		}
	private:
		Manager& manager;
		const golem::U32 menuLevel;
	};

	/** Data handler libraries */
	data::Handler::Map handlerMap;
	/** Data bundle description */
	data::Data::Desc::Ptr dataDesc;
	/** Data bundle file path */
	std::string dataPath;
	/** Data import path */
	std::string dataImportPath;
	/** Data export path */
	std::string dataExportPath;
	/** Data item label */
	std::string dataItemLabel;
	/** Data bundle file extension */
	std::string dataExt;
	/** Data bundle auto-load */
	StringSeq dataSeq;
	/** Data view */
	Data::View dataView;
	/** Data bundle mode */
	bool dataBundleMode;
	/** Option save lll data */
	size_t optionAllData;

	/** Data bundle collection: (1) safe to read in the main thread, (2) requires scene.getCS() to (2a) write in the main thread, or (2a) read in any other thread */
	data::Data::Map dataMap;
	/** Data bundle pointer - use set/get methods to access it. (1) safe to read in the main thread, (2) requires csData to (2a) write in the main thread, or (2a) read in any other thread */
	data::Data::Map::iterator dataCurrentPtr;

	/** Generate interface handler pointer */
	golem::U32 generatePtrIndex;
	/** Convert interface handler pointer */
	golem::U32 convertPtrIndex;
	/** Transform interface handler pointer */
	golem::U32 transformPtrIndex;
	/** Transform interface process mode */
	golem::U32 transformProcess;

	/** OpenGL renderer */
	RendererSet rendererSet;

	/** Menu control set */
	MenuCtrlMap menuCtrlMap;
	/** Menu command set */
	MenuCmdMap menuCmdMap;
	/** Menu level */
	golem::U32 menuLevel;

	/** Pre-defined poses */
	ConfigMat34::Map poseMap;

	/** Select range of poses */
	ConfigMat34::Range selectPoseRange(const ConfigMat34::Map& poseMap, StringIndexMap::value_type* index = nullptr) const;

	/** Top menu */
	bool isTopMenuLevel() const;
	/** Run menu once */
	virtual void menu();

	/** Current item */
	data::Item* getCurrentItem() const;
	/** Current handler */
	data::Handler* getCurrentHandler() const;
	
	/** Current data pointer */
	data::Data::Map::iterator getCurrentDataPtr() const;
	/** Sets current data pointer */
	void setCurrentDataPtr(data::Data::Map::iterator dataPtr);

	/** Create current data */
	virtual data::Data::Ptr createData() const;

	/** Select items */
	virtual void processItems(Data::Selection::Process process, TestProcessItems testProcessItems = nullptr, bool allData = false, bool enableEmpty = false, bool enableMultichoice = false);

	/** Create render buffer */
	virtual void createRender();
	/** golem::UIRenderer interface */
	virtual void render() const;
	/** golem::UIRenderer interface */
	virtual void customRender() const;

	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** golem::UI/golem::UIKeyboardMouseCallback: read a key, wait no longer than timeOut */
	virtual int waitKey(golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF);
	/** golem::UI/golem::UIRendererCallback: get OpenGL */
	virtual void getOpenGL(golem::OpenGL& openGL) const;
	/** golem::UI/golem::UIRendererCallback: get OpenGL */
	virtual void setOpenGL(const golem::OpenGL& openGL);
	/** UIRendererCallback: Synchronisation with rendering thread */
	virtual golem::CriticalSection &getCS() const;

	/** golem::UI: clear renderers */
	virtual void clearRenderers();
	/** golem::UI: add renderer */
	virtual void addRenderer(const UIRenderer* uiRenderer = nullptr);
	/** golem::UI: remove renderer */
	virtual void removeRenderer(const UIRenderer* uiRenderer = nullptr);
	/** golem::UI: request rendering */
	virtual void requestRender(const golem::UIRenderer* uiRenderer);

	void create(const Desc& desc);
	Manager(golem::Scene &scene);
};

//------------------------------------------------------------------------------

};	// namespace


#endif /*_GOLEM_APP_MANAGER_H_*/
