/** @file WorkspaceCtrl.h
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
#ifndef _GOLEM_ACTIVECTRL_WORKSPACECTRL_WORKSPACECTRL_H_
#define _GOLEM_ACTIVECTRL_WORKSPACECTRL_WORKSPACECTRL_H_

//------------------------------------------------------------------------------

#include <Golem/Plugin/ActiveCtrl.h>
#include <Golem/Plugin/UI.h>
#include <Golem/App/GraphRenderer.h>
#include <Golem/Contact/Manipulator.h>
#include <Golem/Math/RB.h>
#include <Golem/Math/Vec2.h>
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
#include <Golem/ActiveCtrl/ActiveCtrl/InputCtrl.h>
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
#include <Golem/Ctrl/Virtuose6D/Virtuose6D.h>
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** WorkspaceCtrl is the interafce to a robot (right arm + hand), sensing devices and objects. */
class GOLEM_LIBRARY_DECLDIR WorkspaceCtrl : public ActiveCtrl, public UI, public IActiveCtrl {
public:
	/** Control mode */
	enum Mode {
		/** Disabled */
		MODE_DISABLED,
		/** Position */
		MODE_POSITION,
		/** Orientation */
		MODE_ORIENTATION,
		/** Position x */
		MODE_POSITION_X,
		/** Position y */
		MODE_POSITION_Y,
		/** Position z */
		MODE_POSITION_Z,
		/** Orientation x */
		MODE_ORIENTATION_X,
		/** Orientation y */
		MODE_ORIENTATION_Y,
		/** Orientation z */
		MODE_ORIENTATION_Z,

		/** First */
		MODE_FIRST = MODE_POSITION,
		/** Last */
		MODE_LAST = MODE_ORIENTATION_Z,
		/** Last */
		MODE_LAST_TRJ = MODE_ORIENTATION,
	};

	/** Mode name */
	static const std::string ModeName[MODE_LAST + 1];

	/** Coordinate description */
	class CoordDesc {
	public:
		/** Coordinate output index -> description */
		typedef std::map<golem::U32, CoordDesc> Map;

		/** Coordinate input index */
		golem::U32 inp;

		/** Control output offset */
		golem::Real offset;
		/** Control output gain */
		golem::Real gain;

		/** Constructs from description object */
		CoordDesc() {
			setToDefault();
		}
		/** Sets the parameters to the default values */
		void setToDefault() {
			inp = 0;
			offset = REAL_ZERO;
			gain = REAL_ONE;
		}
		
		/** Map */
		static void trn(const Map::value_type& desc, const Real* inp, Real* out) {
			out[desc.first] = desc.second.offset + desc.second.gain*inp[desc.second.inp];
		}
		/** Map */
		static void trn(const Map& map, const Real* inp, Real* out) {
			for (auto &desc : map)
				trn(desc, inp, out);
		}
		
		/** Assert that the description is valid. */
		void assertValid(const Assert::Context& ac) const {
			Assert::valid(Math::isFinite(offset), ac, "offset: inf");
			Assert::valid(Math::isFinite(gain), ac, "gain: inf");
		}
		/** Assert that the description is valid. */
		static void assertValid(const Assert::Context& ac, const CoordDesc::Map& map) {
			for (auto& coord : map) {
				coord.second.assertValid(Assert::Context(ac, "[]."));
				Assert::valid(coord.second.inp < 6, ac, "[].inp: invalid");
				Assert::valid(coord.first < 6, ac, "[].out: invalid");
			}
		}
	};

	/** WorkspaceCtrl factory */
	class GOLEM_LIBRARY_DECLDIR Desc : public ActiveCtrl::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;

		/** Control thread idle sleep */
		golem::MSecTmU32 ctrlThreadSleep;

		/** Simulated mode control gains */
		golem::Vec3 ctrlSimGain;
		/** Simulated mode control reaction period of time */
		golem::Real ctrlSimReac;
		/** Simulated mode control prediction period of time */
		golem::Real ctrlSimPred;

		/** Frame increment */
		golem::RBDist frameIncrement;
		/** Time increment */
		golem::Real timeIncrement;
		/** Frame low increment */
		golem::U32 incrementLow;

		/** Frame size */
		golem::Vec3 frameSize;

		/** Linear transform keys */
		std::string linKeys;
		/** Angular transform keys */
		std::string angKeys;
		/** Waypoints quick-access keys */
		std::string wptKeys;

		/** Cache file */
		std::string file;

		/** Frame in global frame */
		bool modeGlobal;
		/** Simplified operation mode */
		bool modeSimple;
		/** Master-slave mapping mode */
		bool modeMap;

		/** Direct hand control */
		bool handDirCtrl;
		/** Direct hand control open */
		RealSeq handDirCtrlOpen;
		/** Direct hand control closed */
		RealSeq handDirCtrlClosed;

		/** Trajectory time increment */
		golem::Real trjIncrement;

		/** Trajectory renderer - chains */
		golem::GraphRenderer trjRendererChains;
		/** Trajectory renderer - mean */
		golem::GraphRenderer trjRendererMean;
		/** Trajectory renderer - deviation */
		golem::GraphRenderer trjRendererDev;
		/** Trajectory renderer - mean trajectory */
		bool trjShowMean;
		/** Trajectory renderer - mean frame */
		bool trjShowMeanFrame;
		/** Trajectory renderer - deviation trajectory */
		bool trjShowDev;
		/** Trajectory renderer - deviation frame */
		bool trjShowDevFrame;
		/** Trajectory renderer - target frame */
		bool trjShowTar;
		/** Trajectory renderer - current frame */
		bool trjShowCurr;
		/** Trajectory renderer - global frame */
		bool trjShowGlobalFrame;
		/** Trajectory renderer - global frame increment */
		bool trjShowGlobalIncFrame;

		/** Manipulator description */
		Manipulator::Desc::Ptr manipulatorDesc;
		/** Manipulator appearance - mean */
		Manipulator::BoundsAppearance appearanceMean;
		/** Manipulator appearance - deviation */
		Manipulator::BoundsAppearance appearanceDev;

		/** Manifold appearance */
		ManifoldCtrl::Appearance appearanceManifold;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
		/** Input device descriptor */
		InputCtrl::Desc inputCtrlDesc;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		/** Virtuose6D controller description */
		Controller::Desc::Ptr virtuose6DDesc;

		/** Pose gain increment */
		Twist poseGainIncrement;
		/** Force gain trajectory mode */
		Twist forceGainManifold;
		/** Force gain increment */
		Twist forceGainIncrement;

		/** Coordinate input index -> description */
		CoordDesc::Map trjCoordDescMap;
		/** Coordinate input index -> description */
		CoordDesc::Map poseCoordDescMap;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

		/** Constructs from description object */
		Desc() {
			Desc::setToDefault();
		}
		/** Sets the parameters to the default values */
		virtual void setToDefault() {
			ActiveCtrl::Desc::setToDefault();

			ctrlThreadSleep = MSecTmU32(100);

			ctrlSimGain.set(golem::Real(0.001), golem::Real(0.001), golem::Real(0.05));
			ctrlSimReac = golem::Real(0.1);
			ctrlSimPred = golem::Real(0.2);
			frameIncrement.set(golem::Real(0.05), golem::Real(0.05)*golem::REAL_PI);
			timeIncrement = Real(5.0);
			incrementLow = 4;
			frameSize.set(0.1);
			linKeys = "wsedrf";
			angKeys = "tgyhuj";
			wptKeys.clear();
			file.clear();

			modeGlobal = false;
			modeSimple = false;
			modeMap = false;

			handDirCtrl = false;
			handDirCtrlOpen.clear();
			handDirCtrlClosed.clear();

			trjIncrement = golem::Real(0.2);
			trjRendererChains.setToDefault();
			trjRendererChains.edgeShow = true;
			trjRendererChains.show = true;

			trjRendererMean.setToDefault();
			trjRendererDev.setToDefault();

			trjShowMean = true;
			trjShowMeanFrame = true;
			trjShowDev = true;
			trjShowDevFrame = true;
			trjShowTar = true;
			trjShowCurr = true;
			trjShowGlobalFrame = true;
			trjShowGlobalIncFrame = true;

			manipulatorDesc.reset(new Manipulator::Desc);
			appearanceMean.setToDefault();
			appearanceDev.setToDefault();

			appearanceManifold.setToDefault();

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
			inputCtrlDesc.setToDefault();
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
			virtuose6DDesc.reset();
			poseGainIncrement.setZero();
			forceGainManifold.setZero();
			forceGainIncrement.setZero();
			trjCoordDescMap.clear();
			poseCoordDescMap.clear();
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			ActiveCtrl::Desc::assertValid(ac);

			Assert::valid(ctrlThreadSleep >= 0, ac, "ctrlThreadSleep: < 0");

			Assert::valid(ctrlSimGain.isValid(), ac, "ctrlSimGain: invalid");
			Assert::valid(ctrlSimReac > golem::REAL_ZERO, ac, "ctrlSimReac: <= 0");
			Assert::valid(ctrlSimReac <= ctrlSimPred, ac, "ctrlSimReac: > ctrlSimPred");
			Assert::valid(frameIncrement.isValid(), ac, "frameIncrement: invalid");
			Assert::valid(Math::isFinite(timeIncrement), ac, "timeIncrement: invalid");
			Assert::valid(frameSize.isPositive(), ac, "frameSize: <= 0");
			Assert::valid(linKeys.length() == 6 || linKeys.length() == 0, ac, "linKeys: length must be 0 or 6");
			Assert::valid(angKeys.length() == 6 || angKeys.length() == 0, ac, "angKeys: length must be 0 or 6");

			if (handDirCtrl) {
				Assert::valid(!handDirCtrlOpen.empty(), ac, "handDirCtrlOpen: empty");
				for (auto& coord: handDirCtrlOpen)
					Assert::valid(Math::isFinite(coord), ac, "handDirCtrlOpen[]: invalid");
				Assert::valid(!handDirCtrlClosed.empty(), ac, "handDirCtrlClosed: empty");
				for (auto& coord : handDirCtrlClosed)
					Assert::valid(Math::isFinite(coord), ac, "handDirCtrlClosed[]: invalid");
			}

			Assert::valid(trjIncrement > golem::REAL_ZERO, ac, "trjIncrement: <= 0");
			Assert::valid(trjRendererChains.isValid(), ac, "trjRendererChains: invalid");
			Assert::valid(trjRendererMean.isValid(), ac, "trjRendererMean: invalid");
			Assert::valid(trjRendererDev.isValid(), ac, "trjRendererDev: invalid");

			Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(Assert::Context(ac, "manipulatorDesc->"));
			appearanceMean.assertValid(Assert::Context(ac, "appearanceMean."));
			appearanceDev.assertValid(Assert::Context(ac, "appearanceDev."));

			appearanceManifold.assertValid(Assert::Context(ac, "appearanceManifold->"));

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
			inputCtrlDesc.assertValid(Assert::Context(ac, "inputCtrlDesc."));
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
			if (virtuose6DDesc != nullptr)
				Assert::valid(virtuose6DDesc->isValid(), ac, "virtuose6DDesc: invalid");
			Twist poseGainIncrement;
			Assert::valid(poseGainIncrement.isValid(), ac, "poseGainIncrement: invalid");
			Assert::valid(forceGainManifold.isValid(), ac, "forceGainManifold: invalid");
			Assert::valid(forceGainIncrement.isValid(), ac, "forceGainIncrement: invalid");
			CoordDesc::assertValid(Assert::Context(ac, "trjCoordDescMap"), trjCoordDescMap);
			CoordDesc::assertValid(Assert::Context(ac, "poseCoordDescMap"), poseCoordDescMap);
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		GOLEM_CREATE_FROM_OBJECT_DESC2(WorkspaceCtrl, ActiveCtrl::Ptr, golem::Planner&, const Sensor::Seq&)
	};

protected:
	typedef std::pair<Real, size_t> ContactManifoldVal;
	typedef std::vector<ContactManifoldVal> ContactManifoldMap;

	/** Controller state info */
	golem::Controller::State::Info ctrlInfo;

	/** Controller state info for hand */
	golem::Controller::State::Info handInfo;
	/** Hand */
	bool handDirCtrlAvailable;
	/** Direct hand control open command */
	bool handDirCtrlOpenCmd;
	/** Direct hand control open */
	ConfigspaceCoord handDirCtrlOpen;
	/** Direct hand control closed command */
	bool handDirCtrlClosedCmd;
	/** Direct hand control closed */
	ConfigspaceCoord handDirCtrlClosed;

	/** Controller name */
	std::string name;
	/** Command latency */
	golem::SecTmReal commandLatency;
	
	/** Control thread idle sleep */
	golem::MSecTmU32 ctrlThreadSleep;

	/** Control mode */
	golem::U32 mode;
	/** Frame in global frame */
	bool modeGlobal;
	/** Simplified operation mode */
	bool modeSimple;
	/** Master-slave mapping mode */
	bool modeMap;
	/** Trajectory mode */
	bool modeTrj;
	/** Force mode */
	bool modeForce;
	/** Control mode */
	bool modeWaypoint;

	/** Controller task */
	ThreadTask::Ptr task;
	/** Global coordinate */
	golem::ConfigspaceCoord coordGlobal;
	/** Global frame */
	golem::Mat34 frameGlobal;
	/** Local frame */
	golem::Mat34 frameLocal;
	/** Target frame */
	golem::Mat34 frameTarget;

	/** Trajectory control */
	golem::Controller::State::Seq trjSeq;
	/** Trajectory Contact Manifold */
	golem::Twist trjContactManifold;
	/** Trajectory Contact Manifold Map */
	ContactManifoldMap trjContactManifoldMap;
	/** Trajectory control */
	golem::Mat34Seq trjSeqMat34;
	/** Trajectory control */
	mutable golem::Mat34Seq trjSeqMat34Mean, trjSeqMat34Dev;
	/** Trajectory time increment */
	golem::Real trjIncrement;
	/** Trajectory control time */
	golem::SecTmReal trjTime;
	/** Trajectory control time */
	golem::SecTmReal trjTimeDelta;
	/** Trajectory control time */
	golem::SecTmReal trjTimeSent;
	/** Trajectory control coord */
	golem::ConfigspaceCoord trjCoord;
	/** IActiveCtrl: Update callback */
	IActiveCtrl::Update trjUpdateCalback;
	/** IActiveCtrl: Scale callback */
	IActiveCtrl::Scale trjScaleCalback;

	/** Increment step */
	golem::U32 incrementStep;
	/** Frame low increment */
	golem::U32 incrementLow;
	/** Frame increment */
	golem::RBDist frameIncrement;
	/** Time increment */
	golem::Real timeIncrement;

	/** Simulated mode control gains */
	golem::Vec3 ctrlSimGain;
	/** Simulated mode control reaction period of time */
	golem::Real ctrlSimReac;
	/** Simulated mode control prediction period of time */
	golem::Real ctrlSimPred;
	/** Simulated mode transformation */
	golem::Twist ctrlSimModeTrn;
	/** Simulated mode screen coords */
	golem::Vec2 ctrlSimModeScr;
	/** Simulated mode on */
	bool ctrlSimMode;

	/** Waypoints */
	golem::Controller::State::Seq waypoints;
	/** Waypoint index */
	golem::U32 waypointIndex;

	/** Control update arm */
	bool ctrlUpdateArm;
	/** Control update hand */
	bool ctrlUpdateHand;

	/** Linear transform keys */
	std::string linKeys;
	/** Angular transform keys */
	std::string angKeys;
	/** Waypoints quick-access keys */
	std::string wptKeys;

	/** Cache file */
	std::string file;

	/** Trajectory renderer - chains */
	golem::GraphRenderer trjRendererChains;
	/** Trajectory renderer - mean */
	mutable golem::GraphRenderer trjRendererMean;
	/** Trajectory renderer - deviation */
	mutable golem::GraphRenderer trjRendererDev;

	/** Trajectory renderer - mean trajectory */
	bool trjShowMean;
	/** Trajectory renderer - mean frame */
	bool trjShowMeanFrame;
	/** Trajectory renderer - deviation trajectory */
	bool trjShowDev;
	/** Trajectory renderer - deviation frame */
	bool trjShowDevFrame;
	/** Trajectory renderer - target frame */
	bool trjShowTar;
	/** Trajectory renderer - current frame */
	bool trjShowCurr;
	/** Trajectory renderer - global frame */
	bool trjShowGlobalFrame;
	/** Trajectory renderer - global frame increment */
	bool trjShowGlobalIncFrame;

	/** Manipulator */
	Manipulator::Ptr manipulator;

	/** Manipulator appearance - mean */
	Manipulator::BoundsAppearance appearanceMean;
	/** Manipulator appearance - deviation */
	Manipulator::BoundsAppearance appearanceDev;

	/** Manifold appearance */
	ManifoldCtrl::Appearance appearanceManifold;

	/** Frame size */
	golem::Vec3 frameSize;
	/** Renderer */
	mutable golem::DebugRenderer renderer;
	/** Critical section */
	mutable golem::CriticalSection cs;

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML
	/** Input devices */
	InputCtrl::Ptr inputCtrlPtr;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_SFML

#ifdef _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D
	/** Virtuose6D controller */
	Controller::Ptr controllerPtr;
	/** Virtuose6D controller */
	Virtuose6D* virtuose6D;

	/** Reference pose (slave robot) */
	ConfigspaceCoord referencePoseSlave;
	/** Reference pose (master robot) */
	RBCoord referencePoseMaster;
	/** Reference time */
	SecTmReal referenceTime;

	/** Pose gain increment */
	Twist poseGainIncrement;
	/** Force gain trajectory mode */
	Twist forceGainManifold;
	/** Force gain increment */
	Twist forceGainIncrement;

	/** Coordinate input index -> description */
	CoordDesc::Map trjCoordDescMap;
	/** Coordinate input index -> description */
	CoordDesc::Map poseCoordDescMap;

	/** Default stiffness */
	Twist stiffnessDflt;
	/** Default damping */
	Twist dampingDflt;

	/** Simulation */
	bool modeSimulationVirt;

	/** Trajectory profile */
	void profile(golem::Controller::State::Seq& trajectory) const;

	/** Stiffness from manifold */
	Twist getStiffness() const;
	/** Damping from manifold */
	Twist getDamping() const;

	/** Pose update */
	Twist getCoordTrn(bool modeTrj, const Twist& t) const;

	/** Pose update */
	void getUpdate(bool modeTrj, const RBCoord& poseMaster, const ConfigspaceCoord& poseSlave, const Twist& force, Twist& updateFrame, SecTmReal& updateTime) const;
#endif // _GOLEM_BUILD_PLUGIN_ACTIVECTRL_WORKSPACECTRL_VIRTUOSE6D

	/** Pose update */
	static Twist toTwist(const Mat34& m);

	/** Increment */
	golem::Real getInc() const;
	/** Time increment */
	golem::Real getTimeInc(golem::Real sign = golem::REAL_ONE) const;
	/** Frame increment */
	golem::RBDist getFrameInc(golem::Real sign = golem::REAL_ONE) const;

	/** Command time */
	golem::SecTmReal getCommandTime() const;
	/** Controller command */
	golem::Controller::State getCommand(golem::SecTmReal t = golem::SEC_TM_REAL_MAX) const;
	/** Target */
	golem::Mat34 getTarget(bool modeGlobal, const Mat34& frameGlobal, const Mat34& frameLocal, const Mat34& frameTarget, const Mat34& frameCurr, golem::SecTmReal trjTime) const;
		/** Interpolate trajectory */
	golem::Controller::State interpolate(const golem::Controller::State::Seq& trajectory, golem::SecTmReal t) const;
	/** Controller reference frame */
	golem::Mat34 getRef() const;
	/** Controller frame */
	golem::Mat34 getFrame(const golem::ConfigspaceCoord& coord) const;
	/** Controller frame */
	golem::Mat34 getFrameLocal(const golem::ConfigspaceCoord& coord) const;
	/** Controller frame */
	golem::Mat34 getFrameInterpol(const Mat34& frame, const golem::Mat34& trn, Real t) const;

	/** Controller update */
	void ctrlUpdate(const golem::ConfigspaceCoord& coord, const Twist* pUpdateFrame = nullptr, const SecTmReal* pUpdateTime = nullptr);

	/** Trajectory mode */
	bool isTrjMode() const;

	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const {}

	/** golem::UIKeyboardMouse: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIKeyboardMouse: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIKeyboardMouse: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** IActiveCtrl: trajectory control */
	virtual void trajectoryCtrl(const golem::Controller::State::Seq& trajectory, SecTmReal referenceTime, const Mat34& targetFrame, const Mat34& manifoldFrame, const Twist& manifoldFrameDev, bool globalMode, IActiveCtrl::Control control, IActiveCtrl::Update update, IActiveCtrl::Scale scale = nullptr);

	/** IActiveCtrl: variable access */
	virtual Mat34 trajectoryCtrlLocalFrame() const;
	/** IActiveCtrl: variable access */
	virtual Mat34 trajectoryCtrlTargetFrame() const;

	/** Load cache from file */
	virtual void load();
	/** Save cache to file */
	virtual void save() const;

	/** Creates/initialises the ActiveCtrl */
	void create(const Desc& desc);

	/** Constructs the ActiveCtrl */
	WorkspaceCtrl(golem::Planner &planner, const Sensor::Seq& sensorSeq);

	/** Release the ActiveCtrl */
	virtual ~WorkspaceCtrl();
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(golem::WorkspaceCtrl::CoordDesc::Map::value_type& val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_ACTIVECTRL_WORKSPACECTRL_WORKSPACECTRL_H_*/
