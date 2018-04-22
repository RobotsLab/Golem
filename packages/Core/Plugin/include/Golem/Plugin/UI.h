/** @file UI.h
 * 
 * Input and output devices' user interface
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
#ifndef _GOLEM_PLUGIN_UI_H_
#define _GOLEM_PLUGIN_UI_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Mat34.h>
#include <Golem/Sys/Timer.h>
#include <Golem/Sys/Thread.h>
#include <Golem/Plugin/Defs.h>
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <functional>
#include <vector>
#include <string>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** RGBA colour */
class RGBA {
public:
	typedef std::vector<RGBA> Seq;

	struct rgba {
		/** RGBA struct. Little endian. */
		U8 r, g, b, a;
	};

	union {
		struct rgba _rgba;
		U32 _U32;
		U8 _U8[4];
	};

	RGBA() {
	}
	RGBA(const RGBA& _RGBA) : _rgba(_RGBA._rgba) {
	}
	RGBA(const rgba& _rgba) : _rgba(_rgba) {
	}
	RGBA(U32 _U32) : _U32(_U32) {
	}
	RGBA(const U8* _U8) : _U32(*(U32*)_U8) {
	}
	RGBA(U8 r, U8 g, U8 b, U8 a) : _U32(r | (g << 8) | (b << 16) | (a << 24)) {
	}

	inline void set(const RGBA& _RGBA) {
		this->_rgba = _RGBA._rgba;
	}

	inline void set(const rgba& _rgba) {
		this->_rgba = _rgba;
	}

	inline void set(U32 _U32) {
		this->_U32 = _U32;
	}

	inline void set(const U8* _U8) {
		this->_U32 = *(U32*)_U8;
	}

	inline void set(U8 r, U8 g, U8 b, U8 a) {
		this->_rgba.r = r;
		this->_rgba.g = g;
		this->_rgba.b = b;
		this->_rgba.a = a;
	}

	inline void get(U8* _U8) const {
		*(U32*)_U8 = this->_U32;
	}

	inline void get(U8& r, U8& g, U8& b, U8& a) const {
		r = this->_rgba.r;
		g = this->_rgba.g;
		b = this->_rgba.b;
		a = this->_rgba.a;
	}

	/** Basic RGBA colours */
	static const RGBA BLACK;
	static const RGBA RED;
	static const RGBA GREEN;
	static const RGBA YELLOW;
	static const RGBA BLUE;
	static const RGBA MAGENTA;
	static const RGBA CYAN;
	static const RGBA WHITE;
};

//------------------------------------------------------------------------------

/** OpenGL settings */
class OpenGL {
public:
	typedef std::vector<OpenGL> Seq;

	/** Draw mode */
	enum Draw {
		/** Draw default */
		DRAW_DEFAULT = 0x07,
		/** Draw solid */
		DRAW_DEFAULT_SOLID = 0x01,
		/** Draw wireframe */
		DRAW_DEFAULT_WIRE = 0x02,
		/** Draw shadows */
		DRAW_DEFAULT_SHADOW = 0x04,

		/** Draw debug data */
		DRAW_DEBUG = 0x30,
		/** Draw debug data simulation */
		DRAW_DEBUG_SIMULATION = 0x10,
		/** Draw debug data normals */
		DRAW_DEBUG_NORMALS = 0x20,
	};

	/** Frame mode */
	enum Frame {
		/** Frame disabled */
		FRAME_DISABLED = 0,
		/** Frame x */
		FRAME_X,
		/** Frame y */
		FRAME_Y,
		/** Frame z */
		FRAME_Z,
		/** Frame roll */
		FRAME_ROLL,
		/** Frame pitch */
		FRAME_PITCH,
		/** Frame yaw */
		FRAME_YAW,
		/** Frame size */
		FRAME_SIZE,
	};

	/** Frame */
	static const char* frameName[FRAME_SIZE];

	/** Alternative scene view setup using OpenGL camera matrices */
	mutable GLfloat glMatIntrinsic[16];
	mutable GLfloat glMatExtrinsic[16];

	/** GUI window width */
	int x;
	/** GUI window height */
	int y;

	/** View point */
	std::string viewName;
	/** View point */
	Vec3 viewPoint;
	/** View direction */
	Vec3 viewDir;
	/** View viewUp direction */
	Vec3 viewUp;
	/** Increment */
	Real viewInc;

	/** Default colors */
	RGBA clearColor;
	RGBA ambientColor;
	RGBA diffuseColor;
	RGBA specularColor;

	/** Draw mode */
	U32 draw;

	/** Increment */
	Real drawNormalLen;

	/** Construction */
	OpenGL() {
		setToDefault();
	}
	/** Sets to the default state */
	void setToDefault() {
		Mat34 id;
		id.setId();
		id.getColumn44(glMatIntrinsic);
		id.getColumn44(glMatExtrinsic);

		x = 0;
		y = 0;

		viewName = "Default";
		viewPoint.set((Real)10.0, (Real)10.0, (Real)10.0);
		viewDir.set((Real)-1.0, (Real)-1.0, (Real)-0.7);
		viewUp.set((Real)0.0, (Real)0.0, (Real)1.0);
		viewInc = Real(0.2);

		clearColor.set(51, 76, 76, 255);
		ambientColor.set(0, 26, 51, 0);
		diffuseColor.set(255, 255, 255, 0);
		specularColor.set(0, 0, 0, 0);

		draw = DRAW_DEFAULT_SOLID | DRAW_DEFAULT_SHADOW;

		drawNormalLen = Real(0.01);
	}
	/** Init */
	void init(Real scale) {
		viewPoint *= scale;
		viewDir.normalise();
		viewUp.normalise();
	}
	/** Checks if it is valid. */
	bool isValid() const {
		if (viewName.empty())
			return false;
		if (!viewDir.isFinite() || Math::equals(viewDir.magnitude(), REAL_ZERO, REAL_EPS))
			return false;
		if (!viewUp.isFinite() || Math::equals(viewUp.magnitude(), REAL_ZERO, REAL_EPS))
			return false;
		if (!Math::isFinite(viewInc))
			return false;
		if (!Math::isPositive(drawNormalLen))
			return false;
		return true;
	}
};

//------------------------------------------------------------------------------

/** Interface: keyboard and mouse
*/
class UIKeyboardMouse {
public:
	/** Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y) = 0;

	/** Mouse motion handler. */
	virtual void motionHandler(int x, int y) = 0;

	/** Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y) = 0;
};

/** Interface: keyboard callback
*/
class UIKeyboardMouseCallback {
public:
	/** Key mask */
	static const int KEY_MASK = 0xFF;
	/** Special key mask */
	static const int KEY_SHIFT = 0x1000;
	/** Special key mask */
	static const int KEY_CTRL = 0x2000;
	/** Special key mask */
	static const int KEY_ALT = 0x4000;
	/** Special key mask */
	static const int KEY_SPECIAL = 0x0100;

	/** Blocking: read a key, wait no longer than timeOut */
	virtual int waitKey(golem::MSecTmU32 timeOut = golem::MSEC_TM_U32_INF) = 0;
};

//------------------------------------------------------------------------------

/** Interface: OpenGL renderer
*/
class UIRenderer {
public:
	/** Render on output device. */
	virtual void render() const = 0;

	/** Render on output device. */
	virtual void customRender() const = 0;
};

/** Interface: OpenGL renderer callback
*/
class UIRendererCallback {
public:
	/** Non-blocking: get OpenGL */
	virtual void getOpenGL(OpenGL& openGL) const = 0;

	/** Non-blocking: set OpenGL */
	virtual void setOpenGL(const OpenGL& openGL) = 0;

	/** Synchronisation with rendering thread */
	virtual CriticalSection &getCS() const = 0;
};

/** Interface: OpenGL capture
*/
class UICapture {
public:
	/** Capture OpenGL context. */
	virtual void capture(int x, int y, int width, int height) = 0;
};

//------------------------------------------------------------------------------

/** User interface callback */
class UICallback: public golem::UIKeyboardMouseCallback, public golem::UIRendererCallback {
public:
	/** Add renderer */
	virtual void addRenderer(const golem::UIRenderer* uiRenderer = nullptr) = 0;
	/** Remove renderer */
	virtual void removeRenderer(const golem::UIRenderer* uiRenderer = nullptr) = 0;
	/** Request rendering */
	virtual void requestRender(const golem::UIRenderer* uiRenderer) = 0;

	/** Mouse and keyboard input state */
	bool hasInputEnabled() const {
		return inputEnabled;
	}
	/** Mouse and keyboard input enable/disable */
	void setInputEnabled(bool enabled) {
		inputEnabled = enabled;
	}

	/** Initialisation */
	UICallback() : inputEnabled(true) {}

private:
	/** Mouse and keyboard input */
	bool inputEnabled;
};

/** User interface */
class UI: public golem::UIRenderer, public golem::UIKeyboardMouse {
public:
	/** conditional CriticalSectionWrapper */
	class CriticalSectionWrapper {
	public:
		CriticalSectionWrapper(const UICallback* callback) : callback(callback) { if (callback) callback->getCS().lock(); }
		~CriticalSectionWrapper() { if (callback) callback->getCS().unlock(); }
	private:
		const UICallback* callback;
	};

	/** Add callback */
	virtual void addCallback(UICallback& callback) {
		golem::CriticalSectionWrapper csw(csCallback);
		callback.addRenderer(this);
		this->callback = &callback;
	}
	/** Add callback (called by app which implements UICallback).	*/
	template <typename _Type> static void addCallback(UICallback& callback, _Type* type) {
		UI* ui = golem::is<UI>(type);
		if (ui) ui->addCallback(callback);
	}

	/** Remove callback */
	virtual void removeCallback(UICallback& callback) {
		golem::CriticalSectionWrapper csw(csCallback);
		callback.removeRenderer(this);
		this->callback = nullptr;
	}
	/** Remove callback (called by app which implements UICallback) */
	template <typename _Type> static void removeCallback(UICallback& callback, _Type* type) {
		UI* ui = golem::is<UI>(type);
		if (ui) ui->removeCallback(callback);
	}

	/** Reset callback */
	UI() : callback(nullptr), renderBlock(false) {}
	/** Required */
	virtual ~UI() {}

protected:
	/** Renderer blocker */
	class RenderBlock {
	public:
		RenderBlock(UI& ui) : ui(ui) { ui.renderBlock = true; }
		~RenderBlock() { ui.renderBlock = false; }
	private:
		UI& ui;
	};

	/** UI callback (called from the main thread) */
	UICallback* getUICallback() {
		return callback;
	}
	/** UI callback (called from the main thread) */
	const UICallback* getUICallback() const {
		return callback;
	}
	/** Request rendering (can be called from async threads) */
	virtual void requestRender() {
		golem::CriticalSectionWrapper csw(csCallback);
		if (callback)
			callback->requestRender(this);
	}

	/** Renderer blocker */
	bool hasRenderBlock() const {
		return renderBlock;
	}

private:
	/** UI callback */
	UICallback* callback;
	/** Callback CS */
	golem::CriticalSection csCallback;
	/** Render blocking */
	bool renderBlock;
};

//------------------------------------------------------------------------------

/** Debug Rendere */
class DebugRenderer;

/** DebugRenderer callback */
typedef std::function<void(const golem::DebugRenderer&)> DebugRendererCallback;

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_PLUGIN_UI_H_*/
