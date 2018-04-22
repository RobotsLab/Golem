/** @file Trajectory.h
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
#ifndef _GOLEM_DATA_TRAJECTORYUA_TRAJECTORYUA_H_
#define _GOLEM_DATA_TRAJECTORYUA_TRAJECTORYUA_H_

//------------------------------------------------------------------------------

#include <Golem/Data/Trajectory/Trajectory.h>
#include <Golem/Tools/Cloud.h>

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void* golemDescLoader(void);
};

//------------------------------------------------------------------------------

namespace golem {
namespace data {

//------------------------------------------------------------------------------

class ItemTrajectoryUA;
class HandlerTrajectoryUA;

/** Map of pose sequences */
typedef std::map<std::string, golem::Mat34Seq> Mat34MapSeq;

/** Data item representing trajectory.
*/
class GOLEM_LIBRARY_DECLDIR ItemTrajectoryUA : public ItemTrajectory, public Convert, public Export {
public:
	friend class HandlerTrajectoryUA;

	/** Robot hand trajectory */
	golem::Mat34Seq robotHandPoses;
	/** Object trajectory */
	golem::Mat34Seq objectPoses;
	/** Object points */
	golem::Cloud::PointSeq objectPoints;
	/** Joint poses */
	Mat34MapSeq jointsPoses;

	/** Robot hand trajectory */
	mutable File robotHandPoseFile;
	/** Object trajectory file */
	mutable File objectPoseFile;
	/** Object trajectory file */
	mutable File objectPointFile;
	/** Joint poses file */
	mutable File jointsPoseFile;

	/** Clones item. */
	virtual Item::Ptr clone() const;

	/** Creates render buffer, the buffer can be shared and allocated on Handler */
	virtual void createRender();

	/** Trajectory: Sets waypoint collection with no velocity profile. */
	virtual void setWaypoints(const golem::WaypointCtrl::Seq& waypoints);

protected:
	/** Data handler */
	HandlerTrajectoryUA& handler;

	/** Convert: Convert current item */
	virtual Item::Ptr convert(const Handler& handler);
	/** Convert: return available interfaces */
	virtual const StringSeq& getConvertInterfaces() const;
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const Handler& handler) const;

	/** Export: Export to file */
	virtual void exportt(const std::string& path) const;
	/** Export: Available file types */
	virtual const StringSeq& getExportFileTypes() const;

	/** Load item from xml context, accessible only by Data. */
	virtual void load(const std::string& prefix, const golem::XMLContext* xmlcontext);
	/** Save item to xml context, accessible only by Data. */
	virtual void save(const std::string& prefix, golem::XMLContext* xmlcontext) const;

	/** Initialise data item */
	ItemTrajectoryUA(HandlerTrajectoryUA& handler);
};

/** Data handler is associated with a particular item type, it knows how to create items, it can hold shared buffer.
*/
class GOLEM_LIBRARY_DECLDIR HandlerTrajectoryUA : public HandlerTrajectory {
public:
	friend class ItemTrajectoryUA;

	/** Data handler description */
	class GOLEM_LIBRARY_DECLDIR Desc : public HandlerTrajectory::Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Manipulator description */
		Manipulator::Desc::Ptr manipulatorDesc;

		/** Pose suffix */
		std::string poseSuffix;

		/** Robot hand pose */
		ImportFrame importRobotHandPose;
		/** Object pose */
		ImportFrame importObjectPose;
		/** Joints pose */
		ImportFrame importJointsPose;
		/** Transform */
		golem::Mat34 importRobotTrjTransform;
		/** Import from HDF5 dump file: hand pose dataset */
		std::string importHDF5RobotHandPose;
		/** Import from HDF5 dump file: object pose dataset */
		std::string importHDF5ObjectPose;
		/** Import from HDF5 dump file: joints pose dataset */
		StringSeq importHDF5JointsPose;

		/** Appearance frame size */
		golem::Vec3 appearanceHandFrameSize;
		/** Appearance frame size */
		golem::Vec3 appearanceJointsFrameSize;
		/** Appearance point colour */
		golem::Cloud::Appearance appearancePoints;

		/** Set to default */
		Desc() {
			setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			golem::data::HandlerTrajectory::Desc::setToDefault();

			manipulatorDesc.reset(new Manipulator::Desc);

			poseSuffix = getFileExtPose();

			importRobotHandPose.setToDefault();
			importObjectPose.setToDefault();
			importJointsPose.setToDefault();
			importRobotTrjTransform.setId();
			importHDF5RobotHandPose = "RobotHandPose";
			importHDF5ObjectPose = "ObjectPose";
			importHDF5JointsPose.clear();

			appearanceHandFrameSize.set(golem::Real(0.1));
			appearanceJointsFrameSize.set(golem::Real(0.02));
			appearancePoints.setToDefault();
		}

		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			golem::data::HandlerTrajectory::Desc::assertValid(ac);

			Assert::valid(manipulatorDesc != nullptr, ac, "manipulatorDesc: null");
			manipulatorDesc->assertValid(Assert::Context(ac, "manipulatorDesc->"));

			Assert::valid(poseSuffix.length() > 0, ac, "poseSuffix: empty");

			importRobotHandPose.assertValid(Assert::Context(ac, "importRobotHandPose."));
			importObjectPose.assertValid(Assert::Context(ac, "importObjectPose."));
			importJointsPose.assertValid(Assert::Context(ac, "importJointsPose."));
			Assert::valid(importRobotTrjTransform.isValid(), ac, "importRobotTrjTransform: invalid");
			Assert::valid(!importHDF5RobotHandPose.empty(), ac, "importHDF5RobotHandPose: invalid");
			Assert::valid(!importHDF5ObjectPose.empty(), ac, "importHDF5ObjectPose: invalid");
			for (StringSeq::const_iterator i = importHDF5JointsPose.begin(); i != importHDF5JointsPose.end(); ++i)
				Assert::valid(!i->empty(), ac, "importHDF5JointsPose[i]: invalid");

			Assert::valid(appearanceHandFrameSize.isPositive(), ac, "appearanceHandFrameSize: invalid");
			Assert::valid(appearanceJointsFrameSize.isPositive(), ac, "appearanceJointsFrameSize: invalid");
			appearancePoints.assertValid(Assert::Context(ac, "appearancePoints."));
		}

		/** Load descritpion from xml context. */
		virtual void load(golem::Context& context, const golem::XMLContext* xmlcontext);

		/** Creates the object from the description. */
		virtual data::Handler::Ptr create(golem::Context &context) const;
	};

	/** File extension: pose sequence */
	static std::string getFileExtPose();

protected:
	/** Manipulator description */
	Manipulator::Desc::Ptr manipulatorDesc;
	/** Manipulator */
	Manipulator::Ptr manipulator;

	/** Pose suffix */
	std::string poseSuffix;

	/** Robot hand pose */
	ImportFrame importRobotHandPose;
	/** Object pose */
	ImportFrame importObjectPose;
	/** Joints pose */
	ImportFrame importJointsPose;
	/** Transform */
	golem::Mat34 importRobotTrjTransform;
	/** Import from HDF5 dump file: hand pose dataset */
	std::string importHDF5RobotHandPose;
	/** Import from HDF5 dump file: object pose dataset */
	std::string importHDF5ObjectPose;
	/** Import from HDF5 dump file: joints pose dataset */
	StringSeq importHDF5JointsPose;

	/** Appearance frame size */
	golem::Vec3 appearanceHandFrameSize;
	/** Appearance frame size */
	golem::Vec3 appearanceJointsFrameSize;
	/** Appearance point colour */
	golem::Cloud::Appearance appearancePoints;

	/** Convert interfaces */
	StringSeq convertInterfaces;

	/** Export types */
	StringSeq exportTypes;

	/** Rendering */
	golem::DebugRenderer renderer;

	/** HandlerPlanner: Sets planner and controllers. */
	virtual void set(const golem::Planner& planner, const ControllerId::Seq& controllerIDSeq);

	/** Convert: Convert current item */
	virtual Item::Ptr convert(ItemTrajectoryUA& item, const data::Handler& handler);
	/** Convert: is supported by the interface */
	virtual bool isConvertSupported(const data::Handler& handler) const;

	/** Creates render buffer */
	void createRender(const ItemTrajectoryUA& item);
	/** golem::UIRenderer: Render on output device. */
	virtual void render() const;
	/** golem::UIRenderer: Render on output device. */
	virtual void customRender() const;

	/** golem::UIRenderer: Mouse button handler. */
	virtual void mouseHandler(int button, int state, int x, int y);
	/** golem::UIRenderer: Mouse motion handler. */
	virtual void motionHandler(int x, int y);
	/** golem::UIRenderer: Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y);

	/** Construct empty item, accessible only by Data. */
	virtual Item::Ptr create() const;

	/** Import: Import from file */
	virtual Item::Ptr import(const std::string& path);

	/** Export: Export to file */
	virtual void exportt(const ItemTrajectoryUA& item, const std::string& path) const;
	/** Export: Available file types */
	virtual const StringSeq& getExportFileTypes() const;

	/** Initialise handler */
	void create(const Desc& desc);
	/** Initialise handler */
	HandlerTrajectoryUA(golem::Context &context);
};

//------------------------------------------------------------------------------

};	// namespace
};	// namespace

//------------------------------------------------------------------------------

namespace golem {
	template <> GOLEM_LIBRARY_DECLDIR void Stream::read(golem::data::Mat34MapSeq::value_type& value) const;
	template <> GOLEM_LIBRARY_DECLDIR void Stream::write(const golem::data::Mat34MapSeq::value_type& value);
};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_DATA_TRAJECTORYUA_TRAJECTORYUA_H_*/
