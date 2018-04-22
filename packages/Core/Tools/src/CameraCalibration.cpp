/** @file CameraCalibration.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/CameraCalibration.h>
#include <Golem/Tools/Camera.h>
#include <Golem/Plugin/DataI.h>
#include <Golem/Plugin/SensorI.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/XMLParser.h>
#include <Golem/Sys/XMLData.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/legacy/compat.hpp>
#include <GL/gl.h>
#include <GL/glut.h>
#include <fstream>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(CameraCalibration::Equation &val, golem::XMLContext* xmlcontext, bool create) {
	val.first.xmlData(xmlcontext->getContextFirst("first", create), create);
	golem::XMLData(val.second, xmlcontext->getContextFirst("second", create), create);
}

void golem::XMLData(CameraCalibration::Parameters& val, golem::XMLContext* context, bool create) {
	golem::XMLData("w", val.width, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("h", val.height, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("f", val.f, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("fx", val.fx, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("fy", val.fy, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("cx", val.cx, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("cy", val.cy, context->getContextFirst("intrinsic", create), create);
	
	golem::XMLData("k1", val.k1, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("k2", val.k2, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("k3", val.k3, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("p1", val.p1, context->getContextFirst("intrinsic", create), create);
	golem::XMLData("p2", val.p2, context->getContextFirst("intrinsic", create), create);

	try {
		golem::XMLData("k4", val.k4, context->getContextFirst("intrinsic", create), create);
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create) throw msg;
	}
	try {
		golem::XMLData("k5", val.k5, context->getContextFirst("intrinsic", create), create);
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create) throw msg;
	}
	try {
		golem::XMLData("k6", val.k6, context->getContextFirst("intrinsic", create), create);
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create) throw msg;
	}

	if (create) {
		golem::XMLData(val.pose.R, context->getContextFirst("extrinsic", true), true); // as rotation matrix
		//golem::XMLData(Quat(val.pose.R), context->getContextFirst("extrinsic", true), true); // as quaternion
		golem::XMLData(val.pose.p, context->getContextFirst("extrinsic", true), true);
	}
	else {
		golem::XMLData(val.pose, context->getContextFirst("extrinsic"));
	}
}

void golem::XMLData(CameraCalibration::Pattern& val, golem::XMLContext* context, bool create) {
	golem::XMLData("nx", val.nx, context->getContextFirst("pattern", create), create);
	golem::XMLData("ny", val.ny, context->getContextFirst("pattern", create), create);
	golem::XMLData("dx", val.dx, context->getContextFirst("pattern", create), create);
	golem::XMLData("dy", val.dy, context->getContextFirst("pattern", create), create);
}

void golem::XMLData(CameraCalibration::ExtrinsicModel& val, golem::XMLContext* context, bool create) {
	golem::XMLData("simulation", val.simulation, context, create);
	golem::XMLData("manual", val.manual, context, create);
	
	if (create) {
		// as rotation matrix
		golem::XMLData(val.pose.R, context->getContextFirst("pose", true), true);
		golem::XMLData(val.pose.p, context->getContextFirst("pose", true), true);
		// as quaternion
		Quat quat(val.pose.R);
		golem::XMLData(quat, context->getContextFirst("pose_quat", true), true);
		golem::XMLData(val.pose.p, context->getContextFirst("pose_quat", true), true);
		// as Euler angles
		Real roll, pitch, yaw;
		val.pose.R.toEuler(roll, pitch, yaw);
		golem::XMLDataEuler(roll, pitch, yaw, context->getContextFirst("pose_euler", true), true);
		golem::XMLData(val.pose.p, context->getContextFirst("pose_euler", true), true);
	}
	else {
		golem::XMLData(val.pose, context->getContextFirst("pose"));
	}
	
	if (!create) val.equations.clear();
	try {
		golem::XMLData(val.equations, val.equations.max_size(), context, "equation", create);
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create)
			throw msg;
		else {
			val.equations.clear();
			val.simulation = false; // overwrite
		}
	}
}

void golem::XMLData(CameraCalibration::Desc::Map::value_type& val, golem::XMLContext* context, bool create) {
	golem::XMLData("file", const_cast<std::string&>(val.first), context, create);
	if (val.second == nullptr)
		val.second.reset(new CameraCalibration::Desc);
	val.second->load(context);
}

//------------------------------------------------------------------------------

void golem::CameraCalibration::Desc::load(const golem::XMLContext* xmlcontext) {
	golem::XMLData("file", file, const_cast<golem::XMLContext*>(xmlcontext), false);

	optimisationDesc.load(xmlcontext->getContextFirst("optimisation"));
	golem::XMLData(sampleMagnitude, xmlcontext->getContextFirst("optimisation sample_magnitude"), false);
	golem::XMLData(distanceWeights, xmlcontext->getContextFirst("optimisation distance_weights"), false);

#ifdef _GOLEM_KUKA_KINEMATICS
	golem::XMLData("offset_ang_range", kukaDesc.offsetAngRange, xmlcontext->getContextFirst("optimisation kuka"), false);
	golem::XMLData("offset_dist_weights", kukaDesc.offsetDistWeights, xmlcontext->getContextFirst("optimisation kuka"), false);
	golem::XMLData("link_len_range", kukaDesc.linkLenRange, xmlcontext->getContextFirst("optimisation kuka"), false);
	golem::XMLData("link_dist_weights", kukaDesc.linkDistWeights, xmlcontext->getContextFirst("optimisation kuka"), false);
#endif // _GOLEM_KUKA_KINEMATICS

	try {
		golem::XMLData("set_camera_res", setCameraRes, const_cast<golem::XMLContext*>(xmlcontext), false);
	}
	catch (const golem::MsgXMLParser&) {}

	golem::XMLData("use_deformation_map", useDeformationMap, const_cast<golem::XMLContext*>(xmlcontext), false);
	try {
		deformationDist.xmlData(xmlcontext->getContextFirst("deformation_dist"), false);
	}
	catch (const golem::MsgXMLParser&) {
		if (useDeformationMap) throw;
	}
}

//------------------------------------------------------------------------------

void CameraCalibration::Parameters::fromCv(const cv::Size_<int>& size, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs) {
	width = static_cast<U32>(size.width);
	height = static_cast<U32>(size.height);

	typedef double MatType;

	if (cameraMatrix.rows != 3 || cameraMatrix.cols != 3)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::Parameters::fromCv(): cameraMatrix: invalid dimensions %dx%d", cameraMatrix.rows, cameraMatrix.cols);
	if (cameraMatrix.depth() != CV_64F)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::Parameters::fromCv(): cameraMatrix: unknown element type");
	fx = static_cast<Real>(cameraMatrix.at<MatType>(0, 0));
	cx = static_cast<Real>(cameraMatrix.at<MatType>(0, 2));
	fy = static_cast<Real>(cameraMatrix.at<MatType>(1, 1));
	cy = static_cast<Real>(cameraMatrix.at<MatType>(1, 2));

	if (distCoeffs.total() < 5)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::Parameters::fromCv(): distCoeffs: invalid dimensions %d", U32(distCoeffs.total()));
	if (distCoeffs.depth() != CV_64F)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::Parameters::fromCv(): distCoeffs: unknown element type");
	k1 = static_cast<Real>(distCoeffs.at<MatType>(0));
	k2 = static_cast<Real>(distCoeffs.at<MatType>(1));
	p1 = static_cast<Real>(distCoeffs.at<MatType>(2));
	p2 = static_cast<Real>(distCoeffs.at<MatType>(3));
	k3 = static_cast<Real>(distCoeffs.at<MatType>(4));
	k4 = distCoeffs.total() > 5 ? static_cast<Real>(distCoeffs.at<MatType>(5)) : REAL_ZERO;
	k5 = distCoeffs.total() > 6 ? static_cast<Real>(distCoeffs.at<MatType>(6)) : REAL_ZERO;
	k6 = distCoeffs.total() > 7 ? static_cast<Real>(distCoeffs.at<MatType>(7)) : REAL_ZERO;
}

void CameraCalibration::Parameters::toCv(cv::Size_<int>& size, cv::Mat& cameraMatrix, cv::Mat& distCoeffs) const {
	size.width = static_cast<int>(width);
	size.height = static_cast<int>(height);

	typedef double MatType;

	// fx  0  cx
	// 0  fy  cy
	// 0   0   1
	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
	cameraMatrix.at<MatType>(0, 0) = static_cast<MatType>(fx);
	cameraMatrix.at<MatType>(0, 1) = static_cast<MatType>(0.);
	cameraMatrix.at<MatType>(0, 2) = static_cast<MatType>(cx);
	cameraMatrix.at<MatType>(1, 0) = static_cast<MatType>(0.);
	cameraMatrix.at<MatType>(1, 1) = static_cast<MatType>(fy);
	cameraMatrix.at<MatType>(1, 2) = static_cast<MatType>(cy);
	cameraMatrix.at<MatType>(2, 0) = static_cast<MatType>(0.);
	cameraMatrix.at<MatType>(2, 1) = static_cast<MatType>(0.);
	cameraMatrix.at<MatType>(2, 2) = static_cast<MatType>(1.);

	// k1, k2, p1, p2 [, k3 [, k4, k5, k6]]
	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
	distCoeffs.at<MatType>(0) = static_cast<MatType>(k1);
	distCoeffs.at<MatType>(1) = static_cast<MatType>(k2);
	distCoeffs.at<MatType>(2) = static_cast<MatType>(p1);
	distCoeffs.at<MatType>(3) = static_cast<MatType>(p2);
	distCoeffs.at<MatType>(4) = static_cast<MatType>(k3);
	distCoeffs.at<MatType>(5) = static_cast<MatType>(k4);
	distCoeffs.at<MatType>(6) = static_cast<MatType>(k5);
	distCoeffs.at<MatType>(7) = static_cast<MatType>(k6);
}

void CameraCalibration::Pattern::toCvPattern(cv::Size_<int>& size, std::vector< cv::Point3_<float> >& pattern) const {
	size.width = static_cast<int>(nx);
	size.height = static_cast<int>(ny);
	pattern.clear();
	pattern.reserve(nx*ny);
	for (U32 i = 0; i < ny; ++i)
		for (U32 j = 0; j < nx; ++j)
			pattern.push_back(cv::Point3f(static_cast<float>(j*dx), static_cast<float>(i*dy), static_cast<float>(0.))); // [m] -> [mm]
}

//------------------------------------------------------------------------------

void CameraCalibration::fromCv(const cv::Mat& t, const cv::Mat& r, Mat34& pose) {
	typedef double MatType;

	if (t.total() != 3)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::fromCv(): translation: invalid dimensions %d", U32(t.total()));
	if (t.depth() != CV_64F)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::fromCv(): translation: unknown element type");
	pose.p.set(t.at<MatType>(0), t.at<MatType>(1), t.at<MatType>(2));

	if (r.total() != 3)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::fromCv(): rotation: invalid dimensions %d", U32(r.total()));
	if (r.depth() != CV_64F)
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::fromCv(): rotation: unknown element type");
	cv::Mat R;
	cv::Rodrigues(r, R);
	pose.R.m11 = static_cast<Real>(R.at<MatType>(0));
	pose.R.m12 = static_cast<Real>(R.at<MatType>(1));
	pose.R.m13 = static_cast<Real>(R.at<MatType>(2));
	pose.R.m21 = static_cast<Real>(R.at<MatType>(3));
	pose.R.m22 = static_cast<Real>(R.at<MatType>(4));
	pose.R.m23 = static_cast<Real>(R.at<MatType>(5));
	pose.R.m31 = static_cast<Real>(R.at<MatType>(6));
	pose.R.m32 = static_cast<Real>(R.at<MatType>(7));
	pose.R.m33 = static_cast<Real>(R.at<MatType>(8));
	
	//Vec3 axis(r.at<MatType>(0), r.at<MatType>(1), r.at<MatType>(2));
	//Real angle = axis.normalise();
	//pose.R.fromAngleAxis(angle, axis);
}

void CameraCalibration::toCv(const Mat34& pose, cv::Mat& t, cv::Mat& r) {
	typedef double MatType;

	t = cv::Mat::zeros(3, 1, CV_64F);
	t.at<MatType>(0) = static_cast<MatType>(pose.p.x);
	t.at<MatType>(1) = static_cast<MatType>(pose.p.y);
	t.at<MatType>(2) = static_cast<MatType>(pose.p.z);

	cv::Mat R = cv::Mat::zeros(3, 3, CV_64F);
	R.at<MatType>(0) = static_cast<MatType>(pose.R.m11);
	R.at<MatType>(1) = static_cast<MatType>(pose.R.m12);
	R.at<MatType>(2) = static_cast<MatType>(pose.R.m13);
	R.at<MatType>(3) = static_cast<MatType>(pose.R.m21);
	R.at<MatType>(4) = static_cast<MatType>(pose.R.m22);
	R.at<MatType>(5) = static_cast<MatType>(pose.R.m23);
	R.at<MatType>(6) = static_cast<MatType>(pose.R.m31);
	R.at<MatType>(7) = static_cast<MatType>(pose.R.m32);
	R.at<MatType>(8) = static_cast<MatType>(pose.R.m33);
	cv::Rodrigues(R, r);

	//Vec3 axis;
	//Real angle;
	//pose.R.toAngleAxis(angle, axis);
	//axis.setMagnitude(REAL_ONE/angle);
	//r = cv::Mat::zeros(3, 1, CV_64F);
	//r.at<MatType>(0) = static_cast<MatType>(axis.x);
	//r.at<MatType>(1) = static_cast<MatType>(axis.y);
	//r.at<MatType>(2) = static_cast<MatType>(axis.z);
}

//------------------------------------------------------------------------------

void CameraCalibration::XMLData(CameraCalibration& val, golem::XMLContext* context, bool create) const {
	golem::XMLData(val.parameters, context, create);
	golem::XMLData(val.pattern, context, create);
	golem::XMLData(val.extrinsicModel, context->getContextFirst("extrinsic model", create), create);
	if (!create) val.deformationMap.clear();
	try {
		golem::XMLData(val.deformationMap, val.deformationMap.max_size(), context->getContextFirst("deformation", create), "equation", create);
	}
	catch (const golem::MsgXMLParser& msg) {
		if (create)
			throw msg;
	}
}

void CameraCalibration::load() {
	// load config
	golem::XMLParser::Ptr parser;
	try {
		// first try to load directly the specified config
		parser = XMLParser::load(file);
	}
	catch (const golem::Message&) {
		// if failed, attempt to load from library location
		parser = XMLParser::load(context.getLibrary(camera.getPath().library).getDir() + file);
	}
	XMLData(*this, parser->getContextRoot()->getContextFirst("golem sensor"), false);
}

void CameraCalibration::save() const {
	// Create XML parser
	XMLParser::Ptr pParser = XMLParser::Desc().create();
	// Save config
	XMLData(const_cast<CameraCalibration&>(*this), pParser->getContextRoot()->getContextFirst("golem sensor", true), true);
	FileWriteStream fws(file.c_str());
	pParser->store(fws);
}

//------------------------------------------------------------------------------

CameraCalibration::CameraCalibration(Camera& camera) : camera(camera), context(camera.getContext()), rand(context.getRandSeed()), drawImage(nullptr) {
}

CameraCalibration::~CameraCalibration() {
}

void CameraCalibration::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraCalibration::Desc."));

	file = desc.file;

	optimisationDesc = desc.optimisationDesc;
	sampleMagnitude = desc.sampleMagnitude;
	distanceWeights = desc.distanceWeights;
#ifdef _GOLEM_KUKA_KINEMATICS
	kukaDesc = desc.kukaDesc;
#endif // _GOLEM_KUKA_KINEMATICS

	if (!desc.drawImage)
		throw golem::Message(golem::Message::LEVEL_CRIT, "CameraCalibration::create(): Null GUI callback interface");
	drawImage = desc.drawImage;
	setCameraRes = desc.setCameraRes;

	deformationDist = desc.deformationDist;
	useDeformationMap = desc.useDeformationMap;

	// TODO initialise Parameters with some reasonable values
}

//------------------------------------------------------------------------------

//double CameraCalibration::computeReprojectionErrors(const std::vector<std::vector<cv::Point3f>>& cvpatternSeq, const std::vector<std::vector<cv::Point2f>>& cvpointsSeq, const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, std::vector<double>& perViewErrors) {
//	std::vector<cv::Point2f> imagePoints2;
//	perViewErrors.resize(cvpatternSeq.size());
//
//	double totalErr = 0;
//	size_t totalPoints = 0;
//	for (size_t i = 0; i < cvpatternSeq.size(); ++i) {
//		cv::projectPoints(cv::Mat(cvpatternSeq[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
//		const double err = cv::norm(cv::Mat(cvpointsSeq[i]), cv::Mat(imagePoints2), CV_L2);
//		const size_t n = cvpatternSeq[i].size();
//		perViewErrors[i] = std::sqrt(err*err / n);
//		totalErr += err*err;
//		totalPoints += n;
//	}
//
//	return std::sqrt(totalErr / totalPoints);
//}

void CameraCalibration::calibrateIntrinsic(golem::UIKeyboardMouseCallback* callback) {
	(void)camera.set(Camera::CMD_VIDEO);
	ScopeGuard guard([&] () { (void)camera.set(Camera::CMD_STOP); });

	context.write("<Space> to add image markers, <Enter> to process markers and exit, <Esc> to exit\n");

	std::vector<cv::Point2f> cvpoints;
	std::vector<std::vector<cv::Point2f>> cvpointsSeq;
	cv::Size patternSize, imageSize;
	std::vector<cv::Point3f> cvpattern;
	std::vector<std::vector<cv::Point3f>> cvpatternSeq;
	cv::Mat viewGray, viewRef;

	pattern.toCvPattern(patternSize, cvpattern);

	bool process = false;
	for (U32 index = 0;;) {
		Image::Ptr pImage = camera.pop_back();
		Image::assertData(pImage->image);
		cv::Mat view(pImage->image, false); // shallow copy
		imageSize = view.size();

		cvpoints.clear();

		const bool found = cv::findChessboardCorners(view, patternSize, cvpoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (found) {
			// refine
			cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
			// TODO remove magic numbers
			cv::cornerSubPix(viewGray, cvpoints, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			// process
			if (process) {
				cvpointsSeq.push_back(cvpoints);
				if (index++ == 0)
					viewRef = view.clone(); // deep copy
				context.write("Image #%d: %d points added\n", index, cvpoints.size());
				process = false;
			}
		}

		// Draw the corners.
		cv::drawChessboardCorners(view, patternSize, cv::Mat(cvpoints), found);
		IplImage img(view);
		drawImage(&img, &camera);

		switch (callback->waitKey(1)) {
		case 32: // <Space>
			process = true;
			break;
		case 13: // <Enter>
		{
			cvpatternSeq.resize(cvpointsSeq.size(), cvpattern);
			cv::Mat cameraMatrix, distCoeffs;
			std::vector<cv::Mat> rvecs, tvecs;
			const double rme = cv::calibrateCamera(cvpatternSeq, cvpointsSeq, patternSize, cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);
			if (!cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs))
				throw Message(Message::LEVEL_ERROR, "CameraCalibration::calibrateIntrinsic(): Calibration failed");

			//std::vector<double> reprojErrs;
			//std::vector<std::vector<cv::Point2f>> imagePoints;
			//const double are = computeReprojectionErrors(cvpatternSeq, cvpointsSeq, rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

			context.write("Showing undistorted image with projected model points, reprojection error = %f, press a key to save calibration data...\n", rme);
			cv::undistort(viewRef.clone(), viewRef, cameraMatrix, distCoeffs);
			IplImage img(viewRef);
			drawImage(&img, &camera);

			callback->waitKey(golem::MSEC_TM_U32_INF);
			parameters.fromCv(imageSize, cameraMatrix, distCoeffs);

			context.write("CameraMatrix_{fx=%.6f, cx=%.6f, fy=%.6f, cy=%.6f}, DistortionCoeffs_{k1=%.6f, k2=%.6f, p1=%.6f, p2=%.6f, k3=%.6f, k4=%.6f, k5=%.6f, k6=%.6f}\n",
				parameters.fx, parameters.cx, parameters.fy, parameters.cy, parameters.k1, parameters.k2, parameters.p1, parameters.p2, parameters.k3, parameters.k4, parameters.k5, parameters.k6);

			save();
			return;
		}
		case 27: // <Esc>
			throw Cancel("Calibration cancelled");
		}

		camera.push(pImage);
	}
}

//------------------------------------------------------------------------------

void CameraCalibration::drawPose(const Mat34& pose, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, double size, cv::Mat& image) {
	std::vector<cv::Point3f> cvpoints3D;
	std::vector<cv::Point2f> cvpoints2D;

	// 3D points
	for (int i = 0; i < 4; ++i) {
		Vec3 p(REAL_ZERO);
		if (i < 3) p[(size_t)i] = size;
		cvpoints3D.push_back(cv::Point3f((float)p.x, (float)p.y, (float)p.z));
	}

	// transform to image
	cv::Mat t, r;
	toCv(pose, t, r);
	cv::projectPoints(cvpoints3D, r, t, cameraMatrix, distCoeffs, cvpoints2D);

	// draw
	for (int i = 0; i < 3; ++i) {
		cv::Scalar colour(0, 0, 0);
		colour[2 - i] = 255; // BGR: X-red, Y-green, Z-blue
		cv::line(image, cvpoints2D[3], cvpoints2D[(size_t)i], colour, 2);
	}
}

void CameraCalibration::calibrateExtrinsicWithModel(golem::UIKeyboardMouseCallback* callback, Equation& equation, U32 solution) {
	std::vector<cv::Point2f> cvpoints;
	cv::Size patternSize, imageSize;
	std::vector<cv::Point3f> cvpattern;
	cv::Mat viewGray;

	cv::Mat cameraMatrix, distCoeffs;
	
	parameters.toCv(imageSize, cameraMatrix, distCoeffs);
	pattern.toCvPattern(patternSize, cvpattern);

	const Vec3 dimensions(pattern.dx*(pattern.nx - 1), pattern.dy*(pattern.ny - 1), REAL_ZERO);

	for (bool found = false; !found;) {
		switch (callback->waitKey(1)) {
		case 27: // <Esc>
			throw Cancel("Calibration cancelled");
		}

		Image::Ptr pImage = camera.pop_back();
		Image::assertData(pImage->image);
		cv::Mat view(pImage->image, false); // shallow copy

		cvpoints.clear();

		found = cv::findChessboardCorners(view, patternSize, cvpoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
		if (found) {
			// refine
			cv::cvtColor(view, viewGray, cv::COLOR_BGR2GRAY);
			// TODO remove magic numbers
			cv::cornerSubPix(viewGray, cvpoints, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			
			// Draw the corners.
			cv::drawChessboardCorners(view, patternSize, cv::Mat(cvpoints), found);
			
			// process
			context.write("Computing camera pose...\n");

			cv::Mat rvec, tvec;
			if (!cv::solvePnP(cvpattern, cvpoints, cameraMatrix, distCoeffs, rvec, tvec))
				continue;

			//cv::undistort(view.clone(), view, cameraMatrix, distCoeffs);

			fromCv(tvec, rvec, parameters.pose);

			// Z-axis fix
			Mat33 R;
			R.rotY(REAL_PI);
			parameters.pose.R = parameters.pose.R * R;
			R.rotZ(REAL_PI_2);
			parameters.pose.R = parameters.pose.R * R;

			// ambiguous pattern frame fix
			if (solution%2 == 1) {
				Mat34 tmp = Mat34::identity();
				tmp.p.set(dimensions.y, dimensions.x, REAL_ZERO);
				tmp.R.rotZ(REAL_PI);
				parameters.pose = parameters.pose * tmp;
			}
			equation.second.setInverse(parameters.pose);

			// TODO do not use magic numbers
			const Real size = 0.5*std::min(dimensions.x, dimensions.y);
			drawPose(parameters.pose, cameraMatrix, distCoeffs, size, view);
		}

		IplImage img(view);
		drawImage(&img, &camera);
		camera.push(pImage);
	}
}

void CameraCalibration::calibrateExtrinsic(golem::UIKeyboardMouseCallback* callback, ConfigCommand configCommand) {
	extrinsicModel.estimatePose = configCommand != nullptr;

	if (!extrinsicModel.simulation) {
		(void)camera.set(Camera::CMD_VIDEO);
		ScopeGuard guard([&] () { (void)camera.set(Camera::CMD_STOP); });

		extrinsicModel.equations.clear();
		for (U32 ptr = 0, solution = 0;;) {
			if (extrinsicModel.manual) {
				context.write("<Bkspace> to repeat, <Enter> to continue, <Esc> to cancel\n");
				LOOP:
				switch (callback->waitKey(golem::MSEC_TM_U32_INF)) {
				case 27: // <Esc>
					throw Cancel("Calibration cancelled");
				case 13: // <Enter>
					solution = 0;
					break;
				case 8: // <Bkspace>
					extrinsicModel.equations.erase(extrinsicModel.equations.begin() + ptr);
					++solution;
					goto REPEAT;
				default:
					goto LOOP;
				}
			}
			
			if ((configCommand == nullptr || !configCommand()) && !extrinsicModel.equations.empty())
				break;
			drawImage(nullptr, nullptr);

			REPEAT:
			
			Equation equation;
			
			if (camera.hasVariableMounting())
				camera.getConfig(equation.first);

			calibrateExtrinsicWithModel(callback, equation, solution);
			ptr = (U32)extrinsicModel.equations.size();
			if (!extrinsicModel.estimatePose)
				equation.second.multiply(extrinsicModel.pose, equation.second);

			extrinsicModel.equations.push_back(equation);
		}
	}

	calibrateExtrinsic(extrinsicModel, parameters);
}

void CameraCalibration::calibrateExtrinsic(data::Capture* capture, golem::UIKeyboardMouseCallback* callback, ConfigCommand configCommand) {
	extrinsicModel.estimatePose = configCommand != nullptr;

	if (!extrinsicModel.simulation) {
		ScopeGuard guard([&]() { enableDeformationMap(isEnabledDeformationMap()); });
		enableDeformationMap(false);

		extrinsicModel.equations.clear();
		for (;;) {
			if (extrinsicModel.manual) {
				context.write("<Enter> to continue, <Esc> to cancel\n");
				LOOP:
				switch (callback->waitKey(golem::MSEC_TM_U32_INF)) {
				case 27: // <Esc>
					throw Cancel("Calibration cancelled");
				case 13: // <Enter>
					break;
				default:
					goto LOOP;
				}
			}
			if ((configCommand == nullptr || !configCommand()) && !extrinsicModel.equations.empty())
				break;

			Equation equation;
			if (camera.hasVariableMounting())
				camera.getConfig(equation.first);

			// capture data item
			data::Item::Ptr item = capture->capture(camera, [&](const golem::TimeStamp*) -> bool { return true; });
			// create temporary containers
			data::Item::Map map;
			map.insert(std::make_pair("item", item));
			data::Item::List list;
			list.insert(list.end(), map.begin());
			// process data item
			data::Transform* transform = is<data::Transform>(capture);
			if (!transform)
				throw Message(Message::LEVEL_ERROR, "CameraCalibration::calibrateExtrinsic(): data::Transform interface not available");
			item = transform->transform(list);
			// find model pose
			data::Model* model = is<data::Model>(item.get());
			if (!model)
				throw Message(Message::LEVEL_ERROR, "CameraCalibration::calibrateExtrinsic(): data::Model interface not available");
			golem::ConfigMat34 config;
			model->model(config, equation.second);
			// remove camera frame
			Mat34 cameraFrame;
			cameraFrame.setInverse(getFrame());
			equation.second.multiply(cameraFrame, equation.second);
			equation.second.setInverse(equation.second); // camera in the frame of model
			if (!extrinsicModel.estimatePose)
				equation.second.multiply(extrinsicModel.pose, equation.second);

			extrinsicModel.equations.push_back(equation);
		}
	}

	calibrateExtrinsic(extrinsicModel, parameters);
}

//------------------------------------------------------------------------------

void CameraCalibration::calibrateExtrinsic(ExtrinsicModel& extrinsicModel, Parameters& parameters) const {
	if (extrinsicModel.equations.empty())
		throw Message(Message::LEVEL_ERROR, "CameraCalibration::calibrateExtrinsic(): Calibration failed");

	// debug information
	for (EquationSeq::const_iterator i = extrinsicModel.equations.begin(); i != extrinsicModel.equations.end(); ++i) {
		const Quat firstQ(i->first.w.R), secondQ(i->second.R);

		std::stringstream str;
		for (RealSeq::const_iterator j = i->first.c.begin(); j != i->first.c.end(); ++j)
			str << *j << ", ";

		context.debug("Equation#%d: first = {(%f, %f, %f, %f), (%f, %f, %f), (%s)}, second = {(%f, %f, %f, %f), (%f, %f, %f)}\n",
			1 + (i - extrinsicModel.equations.begin()),
			firstQ.q0, firstQ.q1, firstQ.q2, firstQ.q3, i->first.w.p.v1, i->first.w.p.v2, i->first.w.p.v3,
			str.str().c_str(),
			secondQ.q0, secondQ.q1, secondQ.q2, secondQ.q3, i->second.p.v1, i->second.p.v2, i->second.p.v3
			);
	}

	if (extrinsicModel.estimatePose) {
		// second = model_pose*first*camera_frame
		findCameraAndModel(extrinsicModel.equations, parameters.pose, extrinsicModel.pose);
		extrinsicModel.pose.setInverse(extrinsicModel.pose);
	}
	else
		// second = first*camera_frame
		findCamera(extrinsicModel.equations, parameters.pose);

	// print results
	const RBCoord cp(parameters.pose);
	context.write("Camera pose: {p = (%f, %f, %f), q = (%f, %f, %f, %f)}\n", cp.p.v1, cp.p.v2, cp.p.v3, cp.q.q0, cp.q.q1, cp.q.q2, cp.q.q3);
	const RBCoord mp(extrinsicModel.pose);
	context.write("Model pose : {p = (%f, %f, %f), q = (%f, %f, %f, %f)}\n", mp.p.v1, mp.p.v2, mp.p.v3, mp.q.q0, mp.q.q1, mp.q.q2, mp.q.q3);

	context.write("Calibration succeeded\n");

	// save data
	save();
}

void CameraCalibration::findCamera(const EquationSeq& equations, golem::Mat34& cameraPose) const {
	// second = first*cameraPose ==> cameraPose = first^-1*second

	typedef std::pair<Vec3, Quat> Trn;
	typedef std::vector<Trn> TrnSeq;
	
	// create quaternion representation, account for quaternion double cover of SO(3)
	TrnSeq trnSeq;
	for (EquationSeq::const_iterator i = equations.begin(); i != equations.end(); ++i) {
		Mat34 pose;
		pose.setInverse(i->first.w);
		pose.multiply(pose, i->second);
		const Quat q(pose.R);
		
		if (i == equations.begin())
			trnSeq.push_back(Trn(pose.p, q));
		else {
			const Quat qinv(-q.q0, -q.q1, -q.q2, -q.q3);
			q.dot(trnSeq.front().second) > qinv.dot(trnSeq.front().second) ? trnSeq.push_back(Trn(pose.p, q)) : trnSeq.push_back(Trn(pose.p, qinv));
		}
	}

	// TODO pairwise summation with geodesic distance between quaternions (slerp)

	// simple summation
	Trn avr(Vec3(REAL_ZERO, REAL_ZERO, REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
	for (TrnSeq::const_iterator i = trnSeq.begin(); i != trnSeq.end(); ++i) {
		avr.first.set(avr.first.v1 + i->first.v1, avr.first.v2 + i->first.v2, avr.first.v3 + i->first.v3);
		avr.second.set(avr.second.q0 + i->second.q0, avr.second.q1 + i->second.q1, avr.second.q2 + i->second.q2, avr.second.q3 + i->second.q3);
	}
	const Real s = REAL_ONE/trnSeq.size();
	avr.first.set(s*avr.first.v1, s*avr.first.v2, s*avr.first.v3);
	avr.second.set(s*avr.second.q0, s*avr.second.q1, s*avr.second.q2, s*avr.second.q3);
	avr.second.normalise();

	// compute linear and angular variance with respect to the average
	Variable<Real> lin, ang;
	for (TrnSeq::const_iterator i = trnSeq.begin(); i != trnSeq.end(); ++i) {
		lin.update(avr.first.distance(i->first));
		ang.update(REAL_ONE - avr.second.dot(i->second));
	}
	context.write("Camera pose variance: {lin=%f, ang=%f}\n", lin.getVarianceN(), ang.getVarianceN());

	// update camera pose
	cameraPose.p = avr.first;
	cameraPose.R.fromQuat(avr.second);
}

void CameraCalibration::findCameraAndModel(const EquationSeq& poses, golem::Mat34& cameraPose, golem::Mat34& modelPose) const {
	// second = modelPose*first*cameraPose

	// model pose estimation requires first != Id, i.e. configQuery != nullptr
	if (!camera.hasVariableMounting())
		throw Message(Message::LEVEL_ERROR, "Camera::findCameraAndModel(): model pose estimation requires variable camera pose");
#ifdef _GOLEM_KUKA_KINEMATICS
	if (poses.size() < 4)
#else // _GOLEM_KUKA_KINEMATICS
	if (poses.size() < 2)
#endif // _GOLEM_KUKA_KINEMATICS
		throw Message(Message::LEVEL_ERROR, "Camera::findCameraAndModel(): insufficient number of poses");

	Heuristic heuristic(context);

	// initial modelPose and cameraPose pose the same as in xml config
	const RBCoord initPoseArr[2] = {cameraPose, modelPose};
	const RBCoord *initPose = initPoseArr; // HACK to avoid errors in Lambda due to passing by valye of arrays
	
	// system of equations: second = modelPose*first*cameraPose
	auto equation = [] (const RBCoord& modelPose, const RBCoord& first, const RBCoord& cameraPose, RBCoord& second) {
		second.multiply(first, cameraPose);
		second.multiply(modelPose, second);
	};

#ifdef _GOLEM_KUKA_KINEMATICS
	// Kuka kinematics
	auto kinematics = [] (const Optimisation::Vec& vec, KukaKR5SixxChain::ChainModel& chainModel) {
		for (size_t i = 0; i < KukaKR5SixxChain::NUM_JOINTS; ++i)
			chainModel.encoderOffset[i] = vec.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i);
		chainModel.L10 = vec.getReserved(CameraCalibration::KukaDesc::L10_IDX);
		chainModel.L11 = vec.getReserved(CameraCalibration::KukaDesc::L11_IDX);
		chainModel.L20 = vec.getReserved(CameraCalibration::KukaDesc::L20_IDX);
		chainModel.L21 = vec.getReserved(CameraCalibration::KukaDesc::L21_IDX);
		chainModel.create();
	};
#endif // _GOLEM_KUKA_KINEMATICS

	// generate equations
	RBEquationSeq rbequations;
	for (EquationSeq::const_iterator i = poses.begin(); i != poses.end(); ++i)
		rbequations.push_back(RBEquation(RBConfig(i->first.c, RBCoord(i->first.w)), RBCoord(i->second)));
	
	// sampling TODO: use more efficient (different than rejection) sampling of quaternions around given direction
	heuristic.pSample = [=] (golem::Rand& rand, Heuristic::Vec& vec, golem::Real& value) -> bool {
		for (size_t i = 0; i < CameraCalibration::Heuristic::Vec::RB_N; ++i) {
			Vec3 dp;
			dp.next(rand);
			dp.setMagnitude(sampleMagnitude.lin);
			vec.getRB(i).p.add(initPose[i].p, dp);
			for (;;) {
				Quat q;
				q.next(rand);
				if (q.dot(initPose[i].q) > REAL_ONE - sampleMagnitude.ang) {
					vec.getRB(i).q = q;
					break;
				}
			}
		}
#ifdef _GOLEM_KUKA_KINEMATICS
		for (size_t i = 0; i < KukaKR5SixxChain::NUM_JOINTS; ++i)
			vec.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i) = rand.nextUniform(-kukaDesc.offsetAngRange, +kukaDesc.offsetAngRange);
		KukaKR5SixxChain::ChainModel chainModel; // TODO read xml loaded values in place of the default ones
		vec.getReserved(CameraCalibration::KukaDesc::L10_IDX) = chainModel.L10 + rand.nextUniform(-kukaDesc.linkLenRange, +kukaDesc.linkLenRange);
		vec.getReserved(CameraCalibration::KukaDesc::L11_IDX) = chainModel.L11 + rand.nextUniform(-kukaDesc.linkLenRange, +kukaDesc.linkLenRange);
		vec.getReserved(CameraCalibration::KukaDesc::L20_IDX) = chainModel.L20 + rand.nextUniform(-kukaDesc.linkLenRange, +kukaDesc.linkLenRange);
		vec.getReserved(CameraCalibration::KukaDesc::L21_IDX) = chainModel.L21 + rand.nextUniform(-kukaDesc.linkLenRange, +kukaDesc.linkLenRange);
#endif // _GOLEM_KUKA_KINEMATICS
		return true;
	};
	
	// preserve parameter range
	heuristic.pProcess = [=] (Heuristic::Vec& vec, Heuristic::ThreadData&) {
		for (size_t i = 0; i < RB_N; ++i)
			vec.getRB(i).q.normalise();
#ifdef _GOLEM_KUKA_KINEMATICS
		for (size_t i = 0; i < KukaKR5SixxChain::NUM_JOINTS; ++i)
			vec.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i) = Math::clamp(vec.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i), -kukaDesc.offsetAngRange, +kukaDesc.offsetAngRange);
		KukaKR5SixxChain::ChainModel chainModel; // TODO read xml loaded values in place of the default ones
		vec.getReserved(CameraCalibration::KukaDesc::L10_IDX) = Math::clamp(vec.getReserved(CameraCalibration::KukaDesc::L10_IDX), chainModel.L10 - kukaDesc.linkLenRange, chainModel.L10 + kukaDesc.linkLenRange);
		vec.getReserved(CameraCalibration::KukaDesc::L11_IDX) = Math::clamp(vec.getReserved(CameraCalibration::KukaDesc::L11_IDX), chainModel.L11 - kukaDesc.linkLenRange, chainModel.L11 + kukaDesc.linkLenRange);
		vec.getReserved(CameraCalibration::KukaDesc::L20_IDX) = Math::clamp(vec.getReserved(CameraCalibration::KukaDesc::L20_IDX), chainModel.L20 - kukaDesc.linkLenRange, chainModel.L20 + kukaDesc.linkLenRange);
		vec.getReserved(CameraCalibration::KukaDesc::L21_IDX) = Math::clamp(vec.getReserved(CameraCalibration::KukaDesc::L21_IDX), chainModel.L21 - kukaDesc.linkLenRange, chainModel.L21 + kukaDesc.linkLenRange);
#endif // _GOLEM_KUKA_KINEMATICS
	};
	
	// objective function
	heuristic.pValue = [=] (const Heuristic::Vec& vec, Heuristic::ThreadData&) -> Real {
		Real value = REAL_ZERO;
#ifdef _GOLEM_KUKA_KINEMATICS
		KukaKR5SixxChain::ChainModel chainModel;
		kinematics(vec, chainModel);
#endif // _GOLEM_KUKA_KINEMATICS
		for (CameraCalibration::RBEquationSeq::const_iterator i = rbequations.begin(); i != rbequations.end(); ++i) {
			RBCoord second;
#ifdef _GOLEM_KUKA_KINEMATICS
			Mat34 id = Mat34::identity(), first; // TODO read actual arm local pose
			chainModel.chainForwardTransform(id, i->first.c.data(), first);
			equation(vec.getRB(1), RBCoord(first), vec.getRB(0), second);
#else // _GOLEM_KUKA_KINEMATICS
			equation(vec.getRB(1), i->first.w, vec.getRB(0), second);
#endif // _GOLEM_KUKA_KINEMATICS
			value += distanceWeights.dot(RBDist(i->second, second));
		}
		return value;
	};
	
	// distance function
	heuristic.pDistance = [=] (const Heuristic::Vec& a, const Heuristic::Vec& b) -> Real {
		Real d = REAL_ZERO;
		for (size_t i = 0; i < CameraCalibration::Heuristic::Vec::RB_N; ++i)
			d += distanceWeights.dot(RBDist(a[i], b[i]));

#ifdef _GOLEM_KUKA_KINEMATICS
		Real dOffset = REAL_ZERO;
		for (size_t i = 0; i < KukaKR5SixxChain::NUM_JOINTS; ++i)
			dOffset += Math::sqr(a.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i) - b.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i));
		d += kukaDesc.offsetDistWeights*Math::sqrt(dOffset);
		Real dLink = REAL_ZERO;
		dLink += Math::sqr(a.getReserved(CameraCalibration::KukaDesc::L10_IDX) - b.getReserved(CameraCalibration::KukaDesc::L10_IDX));
		dLink += Math::sqr(a.getReserved(CameraCalibration::KukaDesc::L11_IDX) - b.getReserved(CameraCalibration::KukaDesc::L11_IDX));
		dLink += Math::sqr(a.getReserved(CameraCalibration::KukaDesc::L20_IDX) - b.getReserved(CameraCalibration::KukaDesc::L20_IDX));
		dLink += Math::sqr(a.getReserved(CameraCalibration::KukaDesc::L21_IDX) - b.getReserved(CameraCalibration::KukaDesc::L21_IDX));
		d += kukaDesc.linkDistWeights*Math::sqrt(dLink);
#endif // _GOLEM_KUKA_KINEMATICS
		
		return d;
	};
	
	// run computation
	Optimisation optimisation(heuristic);
	optimisation.start(optimisationDesc); // throws
	// get results
	const size_t solution = optimisation.stop();
	const Optimisation::Vec estimated = optimisation.getVectors()[solution];
	const Real val = optimisation.getValues()[solution]; // objective function value
	const Real var = optimisation.getVariance(); // population distance variance
	const size_t gen = optimisation.getGenerations(); // num of generations processed
	// copy results
	cameraPose.p.set(estimated.getRB(0).p);
	cameraPose.R.fromQuat(estimated.getRB(0).q);
	modelPose.p.set(estimated.getRB(1).p);
	modelPose.R.fromQuat(estimated.getRB(1).q);
	
	// print debug information
	context.debug("Objective value: %f, Population variance: %f, Number of generations: %d\n", val, var, gen);

	Variable<Real> avr[2];
#ifdef _GOLEM_KUKA_KINEMATICS
	KukaKR5SixxChain::ChainModel chainModel;
	kinematics(estimated, chainModel);
#endif // _GOLEM_KUKA_KINEMATICS
	for (RBEquationSeq::const_iterator i = rbequations.begin(); i != rbequations.end(); ++i) {
		RBCoord second;
#ifdef _GOLEM_KUKA_KINEMATICS
		Mat34 id = Mat34::identity(), first; // TODO read actual arm local pose
		chainModel.chainForwardTransform(id, i->first.c.data(), first);
		equation(estimated.getRB(1), RBCoord(first), estimated.getRB(0), second);
#else // _GOLEM_KUKA_KINEMATICS
		equation(estimated.getRB(1), i->first.w, estimated.getRB(0), second);
#endif // _GOLEM_KUKA_KINEMATICS
		const RBDist rbdist(i->second, second);
		avr[0].update(rbdist.lin);
		avr[1].update(rbdist.ang);

		// print debug error information
		context.debug("Pose#%d: {lin = %.9f, ang = %.9f}\n", 1 + (i - rbequations.begin()), Math::sqrt(rbdist.lin), Math::sqrt(Math::abs(rbdist.ang)));
	}

#ifdef _GOLEM_KUKA_KINEMATICS
	std::stringstream strEnc;
	for (size_t i = 0; i < KukaKR5SixxChain::NUM_JOINTS; ++i)
		strEnc << estimated.getReserved(CameraCalibration::KukaDesc::OFFS_IDX + i) << ", ";
	context.write("Kuka encoder offsets: {%s} \n", strEnc.str().c_str());
	context.write("Kuka links lengths  : {L10 = %f, L11 = %f, L20 = %f, L21 = %f} \n",
		estimated.getReserved(CameraCalibration::KukaDesc::L10_IDX), estimated.getReserved(CameraCalibration::KukaDesc::L11_IDX), estimated.getReserved(CameraCalibration::KukaDesc::L20_IDX), estimated.getReserved(CameraCalibration::KukaDesc::L21_IDX)
	);
#endif // _GOLEM_KUKA_KINEMATICS
	context.write("Average error {lin = %f, ang = %f}\n", Math::sqrt(avr[0].getMean()), Math::sqrt(avr[1].getMean()));
}

//------------------------------------------------------------------------------

void CameraCalibration::setDeformationMap(const EquationSeq& deformationMap) {
	this->deformationMap = deformationMap;

	// save data
	save();
}

golem::Mat34 CameraCalibration::getDeformation(const Config& config) const {
	if (deformationDist.c.size() < config.c.size() || deformationMap.empty()) {
		context.warning("CameraCalibration::getDeformation(): id %s: invalid size of deformation distance (%d < %d) or deformation map (%d)\n", camera.getID().c_str(), deformationDist.c.size(), config.c.size(), deformationMap.size());
		return Mat34::identity();
	}

	RBCoord mean(Vec3(REAL_ZERO), Quat(REAL_ZERO, REAL_ZERO, REAL_ZERO, REAL_ZERO));
	Real norm = REAL_ZERO;
	for (EquationSeq::const_iterator i = deformationMap.begin(); i != deformationMap.end(); ++i) {
		const size_t size = std::min(i->first.c.size(), config.c.size());
		Real d = REAL_ZERO;
		for (size_t j = 0; j < size; ++j)
			d += deformationDist.c[j]*Math::sqr(i->first.c[j] - config.c[j]);
		const Real w = Math::exp(-d);
		const RBCoord c(i->second);
		for (size_t j = 0; j < RBCoord::N; ++j)
			mean[j] += w*c[j];
		norm += w;
		//context.verbose("w=%f, p=(%f, %f, %f), q=(%f, %f, %f, %f)\n", w, c.p.x, c.p.y, c.p.z, c.q.x, c.q.y, c.q.z, c.q.w);
	}

	const Real normInv = REAL_ONE/norm;
	mean.p *= normInv;
	mean.q *= normInv;
	mean.q.normalise();
	//context.verbose("CameraCalibration::getDeformation(): p=(%f, %f, %f), q=(%f, %f, %f, %f)\n", mean.p.x, mean.p.y, mean.p.z, mean.q.x, mean.q.y, mean.q.z, mean.q.w);

	return mean.toMat34();
}

bool golem::CameraCalibration::hasDeformationMap() const {
	return !deformationDist.c.empty() && !deformationMap.empty();
}

void golem::CameraCalibration::enableDeformationMap(bool useDeformationMap) {
	this->useDeformationMap = useDeformationMap;
}

//------------------------------------------------------------------------------

golem::Mat34 golem::CameraCalibration::getFrame(bool useDeformationMap) const {
	if (camera.hasVariableMounting()) {
		// WARNING getDeformation() is thread insecure, if setDeformationMap() is called from another thread! Set this->useDeformationMap=false for updates
		Config config;
		camera.getConfig(config);
		return this->useDeformationMap && useDeformationMap && hasDeformationMap() ? getDeformation(config) * config.w * parameters.pose : config.w * parameters.pose;
	}
	else
		return parameters.pose;
}

void CameraCalibration::getGLMatrices(const float zNear, const float zFar, float intrinsic[], float extrinsic[]) const {
	parameters.getIntrinsicMatrix(zNear, zFar, intrinsic);
	parameters.getExtrinsicMatrix(getFrame(), extrinsic); // camera frame
}

//------------------------------------------------------------------------------

CVImage::CVImage(const CVImage& image, int code) : image(nullptr) {
	set(image, code);
}

CVImage::CVImage(const IplImage* data, int code) : image(nullptr) {
	set(data, code);
}

CVImage::CVImage(int width, int height, int depth, int channels) : image(nullptr) {
	reserve(width, height, depth, channels);
}

CVImage::~CVImage() {
	release();
}

void CVImage::set(const CVImage& image, int code) {
	set(image.image, code);
}

void CVImage::set(const IplImage* image, int code) {
	reserve(image);
	if (image != nullptr)
		code < 0 ? cvCopyImage(image, this->image) : cvCvtColor(image, this->image, code);
}

void CVImage::reserve(const IplImage* image) {
	if (image != nullptr)
		reserve(image->width, image->height, image->depth, image->nChannels);
}

void CVImage::reserve(int width, int height, int depth, int channels) {
	if (image == nullptr || image->width != width || image->height != height || image->depth != depth || image->nChannels != channels) {
		release();
		image = cvCreateImage(cvSize(width, height), depth, channels);
	}
}

void CVImage::release() {
	if (image != nullptr) {
		cvReleaseImage(&image);
		image = nullptr;
	}
}

//------------------------------------------------------------------------------
