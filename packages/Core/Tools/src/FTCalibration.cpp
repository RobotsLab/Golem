/** @file FTCalibration.cpp
 * 
 * @author	Marek Kopicki
 * @author	Maxime Adjigble
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki and Maxime Adjigble, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Tools/FTCalibration.h>
#include <Golem/Tools/FT.h>
#include <Golem/Math/Optimisation.h>
#include <Golem/Sys/Message.h>
#include <Golem/Sys/XMLData.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

FTCalibration::FTCalibration(FT& ft) : ft(ft), context(ft.getContext()) {
}

FTCalibration::~FTCalibration() {}

void FTCalibration::create(const Desc& desc) {
	desc.assertValid(Assert::Context("FTCalibration::Desc."));

	localFrame = desc.localFrame;
	samplingInterval = desc.samplingInterval;
	sampleWindowSize = desc.sampleWindowSize;
	file = desc.file;
	useInertia = desc.useInertia;

	massParamMagnitude = desc.massParamMagnitude;
	offsetParamMagnitude = desc.offsetParamMagnitude;
	localFrameParamMagnitude = desc.localFrameParamMagnitude;

	offsetParam.setZero();
	massParam.setZero();
	localFrameParam.setId();

	calibrated = false;
}

//------------------------------------------------------------------------------

void FTCalibration::calibrate() {
	struct Vec {
		golem::Twist massParam;
		golem::Twist offsetParam;
		golem::Vec3 localFrameParamEuler;
		inline Real& operator [] (size_t idx) { return massParam.data()[idx]; }
		inline const Real& operator [] (size_t idx) const { return massParam.data()[idx]; }
	};
	struct Heuristic : public golem::DEHeuristic<void*, Vec, golem::Real> {
		FTCalibration& calibration;
		Real fMagnitude, tMagnitude, massMagnitude;
		Real fNorm, tNorm;

		Heuristic(FTCalibration& calibration) : DEHeuristic(calibration.getContext(), 6 + 6 + 3/*2 x Twist + Vec3*/), calibration(calibration) {
			if (calibration.points.empty())
				throw Message(Message::LEVEL_ERROR, "FTCalibration::calibrate(): %s: no calibration points", calibration.ft.getID().c_str());
			if (calibration.points.size() <= size())
				throw Message(Message::LEVEL_ERROR, "FTCalibration::calibrate(): %s: insufficient number of calibration points %u <= %u", calibration.ft.getID().c_str(), calibration.points.size(), size());
			
			fMagnitude = tMagnitude = REAL_ZERO;
			fNorm = tNorm = REAL_ZERO;
			for (EquationSeq::const_iterator i = calibration.points.begin(); i != calibration.points.end(); ++i) {
				const Real f = i->second.v.magnitudeSqr();
				const Real t = i->second.w.magnitudeSqr();
				fMagnitude += Math::sqrt(f);
				tMagnitude += Math::sqrt(t);
				fNorm += f;
				tNorm += t;
			}
			fMagnitude /= calibration.points.size();
			tMagnitude /= calibration.points.size();
			massMagnitude = calibration.massParamMagnitude * tMagnitude;
			fMagnitude *= calibration.offsetParamMagnitude;
			tMagnitude *= calibration.offsetParamMagnitude;
			fNorm = REAL_ONE / fNorm;
			tNorm = REAL_ONE / tNorm;
		}
		bool sample(Rand& rand, Vec& vector, golem::Real& value) {
			for (size_t i = 0; i < 6; ++i)
				vector[i] = rand.nextGaussian(REAL_ZERO, massMagnitude);
			for (size_t i = 6; i < 9; ++i)
				vector[i] = rand.nextGaussian(REAL_ZERO, fMagnitude);
			for (size_t i = 9; i < 12; ++i)
				vector[i] = rand.nextGaussian(REAL_ZERO, tMagnitude);
			for (size_t i = 0; i < 3; ++i)
				vector.localFrameParamEuler[i] = calibration.localFrameParamMagnitude[i] > REAL_EPS ? rand.nextUniform(-calibration.localFrameParamMagnitude[i], +calibration.localFrameParamMagnitude[i]) : REAL_ZERO;
			return true;
		}
		void process(Vec& vector, ThreadData& threadData) const {
			for (size_t i = 0; i < 3; ++i)
				vector.localFrameParamEuler[i] = golem::Math::clampPI(vector.localFrameParamEuler[i]);
		}
		Type value(Vec& vector, ThreadData& threadData) const {
			Mat33 localFrameParam;
			localFrameParam.fromEuler(vector.localFrameParamEuler.v1, vector.localFrameParamEuler.v2, vector.localFrameParamEuler.v3);
			// minimise average F/T magnitude at calibration points
			Real error = REAL_ZERO;
			for (EquationSeq::const_iterator i = calibration.points.begin(); i != calibration.points.end(); ++i) {
				const Mat33 frame = i->first.w.R * calibration.localFrame.R;
				Twist out;
				calibration.transformMass(frame, vector.massParam, vector.offsetParam, localFrameParam, i->second, out);
				error += fNorm * out.v.magnitudeSqr() + tNorm * out.w.magnitudeSqr(); // combined magnitude of F and T
			}
			return error;
		}
		inline Type distance(const Vec& a, const Vec& b) const {
			Type d = numeric_const<Type>::ZERO;
			const size_t size = this->size();
			for (size_t i = 0; i < size; ++i)
				d += Math::sqr(a[i] - b[i]);
			return Math::sqrt(d);
		}
	};
	typedef golem::DEOptimisation<Heuristic> Optimisation;

	// initialise
	Heuristic heuristic(*this);
	Optimisation optimisation(heuristic);
	
	// run
	Optimisation::Desc optimisationDesc;
	optimisation.start(optimisationDesc);
	
	// fetch results
	const size_t solution = optimisation.stop();
	massParam = optimisation.getVectors()[solution].massParam;
	offsetParam = optimisation.getVectors()[solution].offsetParam;
	const Vec3 localFrameParamEuler = optimisation.getVectors()[solution].localFrameParamEuler;
	localFrameParam.fromEuler(localFrameParamEuler.v1, localFrameParamEuler.v2, localFrameParamEuler.v3);
	const Real val = optimisation.getValues()[solution]; // objective function value
	const Real var = optimisation.getVariance(); // population distance variance
	const size_t gen = optimisation.getGenerations(); // num of generations processed
	
	// print debug information
	context.debug("FTCalibration::calibrate(): %s: Normalised average error: %.9f, Variance: %.6e, Number of iterations: %d\n", ft.getID().c_str(), Math::sqrt(val), var, gen);
	context.debug("FTCalibration::calibrate(): %s: <mass_param v1=\"%.8f\" v2=\"%.8f\" v3=\"%.8f\" w1=\"%.8f\" w2=\"%.8f\" w3=\"%.8f\"/>\n", ft.getID().c_str(), massParam.v.v1, massParam.v.v2, massParam.v.v3, massParam.w.v1, massParam.w.v2, massParam.w.v3);
	context.debug("FTCalibration::calibrate(): %s: <offset_param v1=\"%.8f\" v2=\"%.8f\" v3=\"%.8f\" w1=\"%.8f\" w2=\"%.8f\" w3=\"%.8f\"/>\n", ft.getID().c_str(), offsetParam.v.v1, offsetParam.v.v2, offsetParam.v.v3, offsetParam.w.v1, offsetParam.w.v2, offsetParam.w.v3);
	context.debug("FTCalibration::calibrate(): %s: <local_frame_param roll=\"%.8f\" pitch=\"%.8f\" yaw=\"%.8f\"/>\n", ft.getID().c_str(), localFrameParamEuler.v1, localFrameParamEuler.v2, localFrameParamEuler.v3);
	for (EquationSeq::const_iterator i = points.begin(); i != points.end(); ++i) {
		const Mat33 frame = i->first.w.R * localFrame.R;
		Twist out;
		transformMass(frame, massParam, offsetParam, localFrameParam, i->second, out);
		const Real error = (heuristic.fNorm * out.v.magnitudeSqr() + heuristic.tNorm * out.w.magnitudeSqr()) * points.size(); // combined magnitude of F and T
		context.verbose("FTCalibration::calibrate(): %s: Point #%i/%i, Normalised error %.8f: {(%f, %f, %f), (%f, %f, %f)} -> {(%f, %f, %f), (%f, %f, %f)}\n",
			ft.getID().c_str(), (i - points.begin()) + 1, points.size(), Math::sqrt(error),
			i->second.v.x, i->second.v.y, i->second.v.z, i->second.w.x, i->second.w.y, i->second.w.z,
			out.v.x, out.v.y, out.v.z, out.w.x, out.w.y, out.w.z
		);
	}

	// Done!
	calibrated = true;
}

void FTCalibration::transform(const Config& config, const golem::Twist& inp, golem::Twist& out) const {
	if (!calibrated || !useInertia) {
		out = inp;
		return;
	}

	// sensor frame
	const Mat33 frame = config.w.R * localFrame.R;
	// transform
	transformMass(frame, massParam, offsetParam, localFrameParam, inp, out);
}

//------------------------------------------------------------------------------

/**
Indentification equation for the FT sensor

| F + F_offset | = | R^-1           0          | = | M_Fglobal |
| T + T_offset |   |  0   (T + T_offset)^skewT |   | M_Tlocal  |

F + F_offset = R^-1 * M_Fglobal
T + T_offset = (T + T_offset)^skewT * M_Tlocal

| oFx + Fx |	|												   |	|MassParamsX|
| oFy + Fy |	|inv(ReeToFT)*inv(RbaseToee)		zeros(3,3)	   |	|MassParamsY|
| oFz + Fz | =	|												   | *  |MassParamsZ|
| oMx + Mx |	|												   |	|MassParamsUx|
| oMy + My |	|		zeros(3,3)					skewT(Ff + oFf)|	|MassParamsUy|
| oMz + Mz |	|												   |	|MassParamsUz|

oF = [oFx oFy oFz oMx oMy oMxz] : Intrinsic offset values of the FT sensor
F = [Fx Fy Fz Mx My Mxz]		: Forces without the intrinsic offset of the FT sensor
Ff= [Fx Fy Fz ]					: Force Vector
oFf= [oFx oFy oFz ]				: Force offset Vector
ReeToFT							: Transformation from End effector frame to the FT frame
RbaseToee						: Transformation from The base frame to the end effector frame
MassParams = [MassParamsX MassParamsY MassParamsZ    --> Projection of the forces created by the weight of the DLR hand in the base frame
MassParamsUx MassParamsUy MassParamsUz] --> Position of the center of mass of the DLR Hand in the FT sensor frame
skewT(M)						: Transpose of the skew-symmetric matrix of M
inv(M)							: Inverse of the matrix M
zeros(x,y)						: Zero matrix of x lines and y columns

REMARK:
The above equation is valid for oF=0, thus the offsets need to be computed and substrated from the equation before trying to get the masse parameters
*/

void FTCalibration::transformMass(const golem::Mat33& frame, const golem::Twist& massParam, const golem::Twist& offsetParam, const golem::Mat33& localFrameParam, const golem::Twist& inp, golem::Twist& out) const {
	golem::Mat33 frameInv;
	frameInv.setInverse(frame * localFrameParam);

	golem::Twist ftDelta;
	ftDelta.subtract(inp, offsetParam);

	golem::Mat33 skewT;
	skewT.axisToSkew(ftDelta.v);
	skewT.setTransposed();

	golem::Twist ft;
	ft.v = frameInv * massParam.v;
	ft.w = skewT * massParam.w;

	out.subtract(ftDelta, ft);
}

//------------------------------------------------------------------------------

void FTCalibration::add() {
	Equation equation;

	ft.getConfig(equation.first);

	// collect calibration samples
	Rand rand(RandSeed(context.getRandSeed()._U64[0] + (U64)(SecTmReal(1e6)*context.getTimer().elapsed())));
	SecTmReal timeStamp;
	TwistSeq ftSeq(sampleWindowSize);
	for (U32 j = 0; j < sampleWindowSize; ++j) {
		golem::Sleep::msleep(1 + rand.next()%samplingInterval); // randomise sampling interval
		ft.readSensor(ftSeq[j], timeStamp);
	}

	// find median value
	RealSeq seq(sampleWindowSize);
	for (size_t i = 0; i < 6; ++i) {
		for (U32 j = 0; j < sampleWindowSize; ++j)
			seq[j] = ftSeq[j].data()[i];
		const size_t N = seq.size(), M = N / 2;
		std::nth_element(seq.begin(), seq.begin() + M + 1, seq.end());
		equation.second.data()[i] = seq[M];
	}

	// ad calibration point
	points.push_back(equation);

	//Display the values of the force
	context.debug("FTCalibration::add(): %s: F = (% 8.4f, % 8.4f, % 8.4f), T = (% 8.4f, % 8.4f, % 8.4f)\n", ft.getID().c_str(), equation.second.v.x, equation.second.v.y, equation.second.v.z, equation.second.w.x, equation.second.w.y, equation.second.w.z);
}

void FTCalibration::clear() {
	calibrated = false;
	points.clear();
}

golem::Mat34 FTCalibration::getFrame(const Config& config) const {
	return config.w * localFrame;
}

void FTCalibration::enableInertia(bool useInertia) {
	this->useInertia = useInertia;
}

void FTCalibration::load() {
	calibrated = false;

	// load config
	golem::XMLParser::Ptr parser;
	try {
		// first try to load directly the specified config
		parser = XMLParser::load(file);
	}
	catch (const golem::Message&) {
		// if failed, attempt to load from library location
		parser = XMLParser::load(context.getLibrary(ft.getPath().library).getDir() + file);
	}
	// Load calibration data
	try {
		golem::XMLData(massParam, parser->getContextRoot()->getContextFirst("golem ft_sensor mass_param"));
		golem::XMLData(offsetParam, parser->getContextRoot()->getContextFirst("golem ft_sensor offset_param"));
		golem::XMLData(localFrameParam, parser->getContextRoot()->getContextFirst("golem ft_sensor local_frame_param"));
		calibrated = true;
	}
	catch (const golem::MsgXMLParser&) {
		context.debug("FTCalibration::load(): %s: unable to load calibration parameters from %s\n", ft.getID().c_str(), file.c_str());
	}

	// Load calibration points from xml
	try {
		points.clear();
		golem::XMLData(points, points.max_size(), parser->getContextRoot()->getContextFirst("golem ft_sensor calibration_points"), "equation");

		// if Calibrated - nothing else to do here
		if (calibrated)
			return;

		// Calibrate
		try {
			calibrate();
		}
		catch (const std::exception& ex) {
			context.write("%s\n", ex.what());
		}
	}
	catch (const golem::MsgXMLParser&) {
		context.debug("FTCalibration::load(): %s: unable to load calibration points from %s\n", ft.getID().c_str(), file.c_str());
	}
}

void FTCalibration::save() const {
	// Create XML parser
	XMLParser::Ptr parser = XMLParser::Desc().create();
	// Save config into xml tree
	golem::XMLData(const_cast<EquationSeq&>(points), points.max_size(), parser->getContextRoot()->getContextFirst("golem ft_sensor calibration_points", true), "equation", true);
	if (calibrated) {
		golem::XMLData(const_cast<Twist&>(massParam), parser->getContextRoot()->getContextFirst("golem ft_sensor mass_param", true), true);
		golem::XMLData(const_cast<Twist&>(offsetParam), parser->getContextRoot()->getContextFirst("golem ft_sensor offset_param", true), true);
		golem::XMLData(const_cast<Mat33&>(localFrameParam), parser->getContextRoot()->getContextFirst("golem ft_sensor local_frame_param", true), true);
	}
	// Save config into file
	FileWriteStream fws(file.c_str());
	parser->store(fws);
}

//------------------------------------------------------------------------------

void golem::XMLData(FTCalibration::Equation &val, golem::XMLContext* xmlcontext, bool create) {
	val.first.xmlData(xmlcontext->getContextFirst("first", create), create);
	golem::XMLData(val.second, xmlcontext->getContextFirst("second", create), create);
}

void golem::XMLData(FTCalibration::Desc &val, golem::XMLContext* xmlcontext, bool create) {
	golem::XMLData(val.localFrame, xmlcontext->getContextFirst("local_frame", create), create);
	golem::XMLData("file", val.file, xmlcontext, create);
	golem::XMLData("use_inertia", val.useInertia, xmlcontext, create);
	golem::XMLData("sampling_interval", val.sampleWindowSize, xmlcontext, create);
	golem::XMLData("sample_window_size", val.sampleWindowSize, xmlcontext, create);
	golem::XMLData("mass_param_manitude", val.massParamMagnitude, xmlcontext, create);
	golem::XMLData("offset_param_manitude", val.offsetParamMagnitude, xmlcontext, create);
	golem::XMLData(val.localFrameParamMagnitude, xmlcontext->getContextFirst("local_frame_param_manitude"), create);
}

//------------------------------------------------------------------------------

void golem::XMLData(FTCalibration::Desc::Ptr &val, golem::XMLContext* xmlcontext, bool create) {
	FTCalibration::Desc* desc = new FTCalibration::Desc;
	val.reset(desc);
	golem::XMLData(*desc, xmlcontext, create);
}

//------------------------------------------------------------------------------
