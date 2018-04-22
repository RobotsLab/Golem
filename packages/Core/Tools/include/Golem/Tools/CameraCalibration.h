/** @file CameraCalibration.h
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
#ifndef _GOLEM_TOOLS_CAMERA_CALIBRATION_H_
#define _GOLEM_TOOLS_CAMERA_CALIBRATION_H_

//------------------------------------------------------------------------------

/** Kuka encoder errors and kinematics parameters (lengths of links) estimation */
//#define _GOLEM_KUKA_KINEMATICS

//------------------------------------------------------------------------------

#include <Golem/Plugin/UI.h>
#include <Golem/Tools/Defs.h>
#include <Golem/Tools/Image.h>
#include <Golem/Math/RBOptimisation.h>
#include <Golem/Math/Rand.h>

#ifdef _GOLEM_KUKA_KINEMATICS
#include <Golem/Ctrl/Kuka/KukaKR5Sixx.h>
#endif // _GOLEM_KUKA_KINEMATICS

//------------------------------------------------------------------------------

namespace cv {
	class Mat;
	template <typename _Tp> class Size_;
	template <typename _Tp> class Point_;
	template <typename _Tp> class Point3_;
};

//------------------------------------------------------------------------------

namespace golem {

namespace data {
	class Capture;
};

//------------------------------------------------------------------------------

/** forward declaration of Camera */
class Camera;

/** Camera calibration */
class CameraCalibration {
public:
	typedef golem::shared_ptr<CameraCalibration> Ptr;
	typedef std::map<std::string, Ptr> Map;

	typedef golem::ConfigMat34 Config;
	typedef std::pair<Config, golem::Mat34> Equation;
	typedef std::vector<Equation> EquationSeq;

	typedef golem::ConfigPose<RBCoord> RBConfig;
	typedef std::pair<RBConfig, RBCoord> RBEquation;
	typedef std::vector<RBEquation> RBEquationSeq;

	/** Transformations */
	static const size_t RB_N = 2;
#ifdef _GOLEM_KUKA_KINEMATICS
	/** Kuka custom kinematics constants */
	class KukaDesc {
	public:
		// Encoder offsets
		static const size_t OFFS_IDX = 0;
		// Kuka links L10, L11, L20, L21 indices
		static const size_t L10_IDX = 6;
		static const size_t L11_IDX = 7;
		static const size_t L20_IDX = 8;
		static const size_t L21_IDX = 9;
		/** Number of parameters to estimate */
		static const size_t M = golem::KukaKR5SixxChain::NUM_JOINTS + 4;

		/** Offset angle range */
		golem::Real offsetAngRange;
		/** Kuka links' distance weights */
		golem::Real offsetDistWeights;
		/** Link length range */
		golem::Real linkLenRange;
		/** Kuka links' distance weights */
		golem::Real linkDistWeights;

		KukaDesc() {
			setToDefault();
		}
		void setToDefault() {
			offsetAngRange = golem::Real(0.05)*golem::REAL_PI;
			offsetDistWeights = golem::Real(1.0);
			linkLenRange = golem::Real(0.05);
			linkDistWeights = golem::Real(1.0);
		}
		bool isValid() const {
			if (offsetAngRange < golem::REAL_ZERO || offsetDistWeights < golem::REAL_ZERO)
				return false;
			if (linkLenRange < golem::REAL_ZERO || linkDistWeights < golem::REAL_ZERO)
				return false;
			return true;
		}
	};
	/** 2-rigid body heuristic */
	typedef golem::RBHeuristic<RBCoord, RB_N, KukaDesc::M> Heuristic;
	/** 2-rigid body optimisation */
	typedef golem::DEOptimisation<Heuristic> Optimisation;
#else // _GOLEM_KUKA_KINEMATICS
	/** 2-rigid body heuristic */
	typedef golem::RBHeuristic<RBCoord, RB_N> Heuristic;
	/** 2-rigid body optimisation */
	typedef golem::DEOptimisation<Heuristic> Optimisation;
#endif // _GOLEM_KUKA_KINEMATICS

	/** Configuration command */
	typedef std::function<bool()> ConfigCommand;
	/** Image drawing */
	typedef std::function<void(const IplImage*, Camera* camera)> DrawImage;

	/** Camera intrinsic and extrinsic parameters */
	class Parameters {
	public:
		/** Width */
		golem::U32 width;
		/** Height */
		golem::U32 height;

		// Intrinsic parameters:
		// focal length
		golem::Real f;
		// entries of the camera matrix
		golem::Real fx;
		golem::Real fy;
		golem::Real cx;
		golem::Real cy;
		// radial distortion parameters
		golem::Real k1, k2, k3, k4, k5, k6;
		// tangential distortion parameters
		golem::Real p1, p2;

		// Extrinsic parameters: camera pose (local or global depending on the setup)
		golem::Mat34 pose;

		/** Clear values */
		Parameters() {
			setToDefault();
		}

		/** Clear values */
		void setToDefault() {
			width = 1;
			height = 1;
			f = golem::REAL_ZERO;
			fx = golem::REAL_ZERO;
			fy = golem::REAL_ZERO;
			cx = golem::REAL_ZERO;
			cy = golem::REAL_ZERO;
			k1 = k2 = k3 = k4 = k5 = k6 = golem::REAL_ZERO;
			p1 = p2 = golem::REAL_ZERO;
			pose.setToDefault();
		}

		/** Export intrinsic parameters in a matrix form */
		template <typename _Real> void getIntrinsicMatrix(_Real zNear, _Real zFar, _Real intrinsic[]) const {
			// intrinsic parameters
			// transform the coordinate system of computer vision to OpenGL 
			// Vision: origin is in the up left corner, x-axis pointing right, y-axis pointing down
			// OpenGL: origin is in the middle, x-axis pointing right, y-axis pointing up

			// scale range from [0 ... width] to [0 ... 2]
			const _Real fx = (_Real)(golem::REAL_TWO*this->fx/this->width);
			// scale range from [0 ... height] to [0 ... 2]
			const _Real fy = (_Real)(golem::REAL_TWO*this->fy/this->height);
			// move coordinates from left to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
			const _Real cx = (_Real)(golem::REAL_ONE - (golem::REAL_TWO*this->cx/this->width));
			// flip and move coordinates from top to middle of image: [0 ... 2] -> [-1 ... 1] (not negative z value at w-division)
			const _Real cy = (_Real)((golem::REAL_TWO*this->cy/this->height) - golem::REAL_ONE);
			// entries for clipping planes
			const _Real z1 = (zFar + zNear)/(zNear - zFar);
			// look up for gluPerspective
			const _Real z2 = _Real(2.0)*zFar*zNear/(zNear - zFar);

			// intrinsic matrix
			// last row assigns w=-z which inverts cx and cy at w-division
			intrinsic[0] = fx;		intrinsic[4] = 0;		intrinsic[8] = cx;		intrinsic[12] = 0;
			intrinsic[1] = 0;		intrinsic[5] = fy;		intrinsic[9] = cy;		intrinsic[13] = 0;
			intrinsic[2] = 0;		intrinsic[6] = 0;		intrinsic[10] = z1;		intrinsic[14] = z2;
			intrinsic[3] = 0;		intrinsic[7] = 0;		intrinsic[11] = -1;		intrinsic[15] = 0;
		}
		/** Export extrinsic parameters in a matrix form */
		template <typename _Real> static void getExtrinsicMatrix(const golem::Mat34& pose, _Real extrinsic[]) {
			// extrinsic matrix
			golem::Mat34 invPose, ex, flipYZ;
			invPose.setInverseRT(pose);
			flipYZ.setId();
			flipYZ.R.m[1][1] = -1.0;
			flipYZ.R.m[2][2] = -1.0;
			ex.multiply(flipYZ, invPose);
			ex.getColumn44(extrinsic);
		}

		/** OpenCV conversion - intrinsics */
		void fromCv(const cv::Size_<int>& size, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
		/** OpenCV conversion - intrinsics */
		void toCv(cv::Size_<int>& size, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) const;
	};

	/** Pattern */
	class Pattern {
	public:
		/** Number of points in x- and y-direction */
		golem::U32 nx, ny;
		/** Spacing of points in x- and y-direction */
		golem::Real dx, dy;

		Pattern() {
			setToDefault();
		}
		void setToDefault() {
			nx = ny = 1;
			dx = dy = golem::REAL_ONE;
		}

		/** OpenCV conversion */
		void toCvPattern(cv::Size_<int>& size, std::vector< cv::Point3_<float> >& pattern) const;
	};

	/** Extrinsic model */
	class ExtrinsicModel {
	public:
		/** Model pose */
		golem::Mat34 pose;
		/** Equations */
		EquationSeq equations;
		/** Estimate model pose */
		bool estimatePose;
		/** Simulate calibration */
		bool simulation;
		/** Manual calibration */
		bool manual;

		ExtrinsicModel() {
			setToDefault();
		}
		void setToDefault() {
			pose.setId();
			equations.clear();
			estimatePose = false;
			simulation = false;
			manual = false;
		}
	};

	/** CameraCalibration description */
	class Desc {
	public:
		typedef golem::shared_ptr<Desc> Ptr;
		typedef std::map<std::string, Ptr> Map;

		/** Calibration file */
		std::string file;

		/** optimisation */
		Optimisation::Desc optimisationDesc;
		/** sample magnitude */
		RBDist sampleMagnitude;
		/** distance weights */
		RBDist distanceWeights;
#ifdef _GOLEM_KUKA_KINEMATICS
		/** Kuka custom kinematics constants */
		KukaDesc kukaDesc;
#endif // _GOLEM_KUKA_KINEMATICS

		/** Image displaying */
		DrawImage drawImage;
		/** Set camera resolution during displaying the calibration image */
		bool setCameraRes;

		/** Deformation dist */
		Config deformationDist;
		/** Use deformation map */
		bool useDeformationMap;

		/** Constructs description. */
		Desc() {
			setToDefault();
		}
		/** Destroys description. */
		virtual ~Desc() {
		}
		/** Creates the object from the description. */
		GOLEM_CREATE_FROM_OBJECT_DESC1(CameraCalibration, CameraCalibration::Ptr, Camera&)
		/** Sets the parameters to the default values. */
		void setToDefault() {
			file = "Camera.cal";
			optimisationDesc.setToDefault();
			sampleMagnitude = RBDist(golem::Real(0.1), golem::Real(0.1));
			distanceWeights = RBDist(golem::Real(1.0), golem::Real(1.0));
#ifdef _GOLEM_KUKA_KINEMATICS
			kukaDesc.setToDefault();
#endif // _GOLEM_KUKA_KINEMATICS
			drawImage = nullptr;
			setCameraRes = false;
			deformationDist.setToDefault();
			useDeformationMap = true;
		}
		/** Assert that the description is valid. */
		virtual void assertValid(const Assert::Context& ac) const {
			Assert::valid(!file.empty(), ac, "file: empty");
			Assert::valid(optimisationDesc.isValid(), ac, "optimisationDesc: invalid");
			Assert::valid(sampleMagnitude.isValid(), ac, "sampleMagnitude: invalid");
			Assert::valid(distanceWeights.isValid(), ac, "distanceWeights: invalid");
#ifdef _GOLEM_KUKA_KINEMATICS
			Assert::valid(kukaDesc.isValid(), ac, "kukaDesc: invalid");
#endif // _GOLEM_KUKA_KINEMATICS
		}
		/** Load descritpion from xml context. */
		virtual void load(const golem::XMLContext* xmlcontext);
	};

	/** Calibration file */
	const std::string& getFile() const {
		return file;
	}

	/** Load calibration */
	void load();
	/** Save calibration */
	void save() const;

	/** Intrinsic params camera calibration */
	void calibrateIntrinsic(golem::UIKeyboardMouseCallback* callback);
	/** Extrinsic params (pose) camera calibration */
	void calibrateExtrinsic(golem::UIKeyboardMouseCallback* callback, ConfigCommand configCommand);
	/** Extrinsic params (pose) camera calibration with Capture interface */
	void calibrateExtrinsic(data::Capture* capture, golem::UIKeyboardMouseCallback* callback, ConfigCommand configCommand);

	/** Camera intrinsic and extrinsic parameters */
	const Parameters& getParameters() const {
		return parameters;
	}
	/** Camera intrinsic and extrinsic parameters */
	void setParameters(const Parameters& parameters) {
		this->parameters = parameters;
	}

	/** Pattern */
	const Pattern& getPattern() const {
		return pattern;
	}

	/** Extrinsic model */
	const ExtrinsicModel& getExtrinsicModel() const {
		return extrinsicModel;
	}
	/** Extrinsic model */
	void setExtrinsicModel(const ExtrinsicModel& extrinsicModel) {
		this->extrinsicModel = extrinsicModel;
	}

	/** Deformation */
	void setDeformationMap(const EquationSeq& deformationMap);
	/** Deformation */
	golem::Mat34 getDeformation(const Config& config) const;
	/** Deformation */
	inline const EquationSeq& getDeformationMap() const {
		return deformationMap;
	}
	
	/** Has deformation map */
	bool hasDeformationMap() const;
	/** Use deformation map */
	bool isEnabledDeformationMap() const {
		return useDeformationMap;
	}
	/** Enable deformation map */
	void enableDeformationMap(bool useDeformationMap);

	/** Is camera resolution set during displaying the calibration image */
	bool isSetCameraRes() const {
		return setCameraRes;
	}

	/** Camera frame */
	golem::Mat34 getFrame(bool useDeformationMap = false) const;

	/** Extrinsic- and Intrinsic- Matrix of OpenGL (matrix as vector of 16 floats in column-major order)
	*	zNear and zFar describe the near and far z values of the clipping plane
	*/
	void getGLMatrices(const float zNear, const float zFar, float intrinsic[], float extrinsic[]) const;
	
	/** virtual descructor required */
	virtual ~CameraCalibration();

protected:
	/** Camera */
	Camera& camera;
	/** Context reference */
	golem::Context& context;
	/** Random num generator */
	golem::Rand rand;

	/** Calibration file */
	std::string file;

	/** Image displaying */
	DrawImage drawImage;
	/** Set camera resolution during displaying the calibration image */
	bool setCameraRes;

	/** Camera intrinsic and extrinsic parameters */
	Parameters parameters;
	/** Pattern */
	Pattern pattern;

	/** Extrinsic model */
	ExtrinsicModel extrinsicModel;

	/** optimisation */
	Optimisation::Desc optimisationDesc;
	/** sample magnitude */
	RBDist sampleMagnitude;
	/** distance weights */
	RBDist distanceWeights;

	/** Deformation dist */
	Config deformationDist;
	/** Deformation map */
	EquationSeq deformationMap;
	/** Use deformation map */
	bool useDeformationMap;

#ifdef _GOLEM_KUKA_KINEMATICS
	/** Kuka custom kinematics constants */
	KukaDesc kukaDesc;
#endif // _GOLEM_KUKA_KINEMATICS

	/** Reads/writes Calibration from/to a given context */
	void XMLData(CameraCalibration& val, golem::XMLContext* context, bool create) const;
	/** Extrinsic params (pose) camera calibration with intrinsic model */
	void calibrateExtrinsicWithModel(golem::UIKeyboardMouseCallback* callback, Equation& equation, U32 solution = 0);
	/** Extrinsic params (pose) camera calibration */
	void calibrateExtrinsic(ExtrinsicModel& extrinsicModel, Parameters& parameters) const;
	/** Find camera pose */
	void findCamera(const EquationSeq& equations, golem::Mat34& cameraPose) const;
	/** Find camera and model pose */
	void findCameraAndModel(const EquationSeq& equations, golem::Mat34& cameraPose, golem::Mat34& modelPose) const;

	/** OpenCV conversion - extrinsics */
	static void fromCv(const cv::Mat& t, const cv::Mat& r, Mat34& pose);
	/** OpenCV conversion - extrinsics */
	static void toCv(const Mat34& pose, cv::Mat& t, cv::Mat& r);

	/** Draw pose in the image */
	static void drawPose(const Mat34& pose, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double size, cv::Mat& image);
	/** Projection errors */
	//static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3_<float>>>& cvpatternSeq, const std::vector<std::vector<cv::Point_<float>>>& cvpointsSeq, const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, std::vector<double>& perViewErrors);

	/** Creates/initialises the Calibration */
	void create(const Desc& desc);
	/** Constructs Calibration */
	CameraCalibration(Camera& camera);
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(CameraCalibration::Equation &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes Calibration Parameters from/to a given context */
void XMLData(CameraCalibration::Parameters& val, golem::XMLContext* context, bool create = false);
/** Reads/writes Calibration IntrinsicModel from/to a given context */
void XMLData(CameraCalibration::Pattern& val, golem::XMLContext* context, bool create = false);
/** Reads/writes Calibration ExtrinsicModel from/to a given context */
void XMLData(CameraCalibration::ExtrinsicModel& val, golem::XMLContext* context, bool create = false);
/** Reads/writes Calibration ExtrinsicModel from/to a given context */
void XMLData(CameraCalibration::Desc::Map::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** OpenCV image wrapper. TODO update with OpenCV Mat */
class CVImage {
public:
	/** OpenCV Image */
	IplImage* image;

	/** No data allocation */
	CVImage() : image(nullptr) {}
	/** Copies image */
	CVImage(const CVImage& image, int code = -1);
	/** Copies image */
	CVImage(const IplImage* image, int code = -1);
	/** Reserves image data */
	CVImage(int width, int height, int depth = IPL_DEPTH_8U, int channels = 3);

	/** Releases data */
	virtual ~CVImage();

	/** Sets image */
	virtual void set(const CVImage& image, int code = -1);
	/** Sets image */
	virtual void set(const IplImage* image, int code = -1);

	/** Reserves data */
	virtual void reserve(const IplImage* image);
	/** Reserves data */
	virtual void reserve(int width, int height, int depth = IPL_DEPTH_8U, int channels = 3);

	/** Release data */
	virtual void release();
};

//------------------------------------------------------------------------------

};	// namespace golem

#endif /*_GOLEM_TOOLS_CAMERA_CALIBRATION_H_*/
