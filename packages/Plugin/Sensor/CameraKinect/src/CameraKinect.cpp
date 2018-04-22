/** @file Kinect.cpp
 * 
 * @author	Marek Kopicki
 * @author	Krzysztof Walas (PUT)
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki and Krzysztof Walas, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sensor/CameraKinect/CameraKinect.h>
#include <Golem/Tools/Cloud.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Math/Data.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Kinect.h>

// TODO: use C++
#define DELETE_ARRAY(p) if (p!=nullptr) { delete[] p; p=nullptr; }
#define RELEASE_PTR(p)  if (p!=nullptr) { p->Release(); p=nullptr; }

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new CameraKinect::Desc();
}

//------------------------------------------------------------------------------

void golem::CameraKinect::Desc::load(golem::Context& context, const golem::XMLContext* xmlcontext) {
	CameraDepth::Desc::load(context, xmlcontext);

	XMLData("stream_timeout", streamTimeOut, const_cast<golem::XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

class CameraKinectDevice : public Camera::Device
{
	static const int cDepthWidth  = 512;
	static const int cDepthHeight = 424;
	static const int cColorWidth  = 1920;
	static const int cColorHeight = 1080;

public:
	typedef std::map<U32, U32> ModeFormatMap;


	CameraKinectDevice(Camera* camera, const Camera::Property& property, golem::I32 index, const RGBA& colour, golem::MSecTmU32 streamTimeOut)
		: Device(camera, property),
		  mode(CameraKinect::MODE_UNDEF),
		  m_pKinectSensor(nullptr),
		  m_pCoordinateMapper(nullptr),
		  m_pCameraCoordinatesColor(nullptr),
		  m_pCameraCoordinatesDepth(nullptr),
		  m_pMultiSourceFrameReader(nullptr),
		  m_pColorRGBX(nullptr),
		  colour(colour),
		  hasImageStream(false),
		  timeout(streamTimeOut)
	{
		StringSeq modeStr, formatStr;
		CameraKinect::getStrings(this->property.mode, modeStr);
		CameraKinect::getStrings(this->property.format, formatStr);
		if (modeStr.empty() || modeStr.size() != formatStr.size())
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraKinectDevice::CameraKinectDevice: Camera modes do not correspond to formats");

		ModeFormatMap modeFormatMap;
		mode = CameraKinect::MODE_UNDEF;
		for (size_t i = 0; i < modeStr.size(); ++i)
			mode |= modeFormatMap.insert(ModeFormatMap::value_type(CameraKinect::getMode(modeStr[i]), std::stoul(formatStr[i]))).first->first;

		
		HRESULT hr;

		hr = GetDefaultKinectSensor(&m_pKinectSensor);
		if (FAILED(hr))
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraKinectDevice::CameraKinectDevice: No cameras detected");

		if (m_pKinectSensor!=nullptr)
		{
			// Initialize the Kinect and get coordinate mapper and the frame reader

			if (SUCCEEDED(hr))
				hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);

			hr = m_pKinectSensor->Open();

			if (SUCCEEDED(hr))
				hr = m_pKinectSensor->OpenMultiSourceFrameReader(
					FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color, &m_pMultiSourceFrameReader);
		}

		if (m_pKinectSensor==nullptr || FAILED(hr))
			throw golem::Message(golem::Message::LEVEL_CRIT, "CameraKinectDevice::CameraKinectDevice: No ready Kinect found!");

		golem::Sleep timer;
		IMultiSourceFrame* pMultiSourceFrame = nullptr;
		int iter = 0;
		timer.msleep(1000);
		do {
			camera->getContext().debug("CameraKinectDevice::CameraKinectDevice: Waiting for Kinect to initialize.\n");
			timer.msleep(500);
			if (iter > 8)
				assertValid(hr, "CameraKinectDevice::CameraKinectDevice: Camera::Device:\n");
			iter++;

		} while (FAILED(hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame)));

		RELEASE_PTR(pMultiSourceFrame);

		camera->getContext().debug("CameraKinectDevice::CameraKinectDevice: Initialized Kinect\n");

		m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight]; // color pixel data in RGBX format

		//m_pDepthCoordinates = new DepthSpacePoint[cDepthWidth * cDepthHeight]; // coordinate mapping from depth to color
		//m_pColorCoordinates = new ColorSpacePoint[cColorWidth * cColorHeight]; // coordinate mapping from color to depth

		m_pCameraCoordinatesDepth = new CameraSpacePoint[cDepthWidth * cDepthHeight]; // coordinate mapping from depth to camera space [x,y,x] in meters
		m_pCameraCoordinatesColor = new CameraSpacePoint[cColorWidth * cColorHeight]; // coordinate mapping from color to camera space [x,y,x] in meters
	}

	~CameraKinectDevice()
	{
		DELETE_ARRAY(m_pColorRGBX);
		//DELETE_ARRAY(m_pDepthCoordinates);
		//DELETE_ARRAY(m_pColorCoordinates);
		DELETE_ARRAY(m_pCameraCoordinatesDepth);
		DELETE_ARRAY(m_pCameraCoordinatesColor);

		RELEASE_PTR(m_pMultiSourceFrameReader);
		RELEASE_PTR(m_pCoordinateMapper);

		if (m_pKinectSensor!=nullptr) m_pKinectSensor->Close();
		RELEASE_PTR(m_pKinectSensor);
	}

	void start() { camera->getContext().debug("CameraKinectDevice::start\n"); }

	void stop()  { camera->getContext().debug("CameraKinectDevice::stop\n"); }

	void capture(Image::Ptr& pImage)
	{
		HRESULT hr;
		
		if (m_pMultiSourceFrameReader==nullptr)
		{
			camera->getContext().error("CameraKinectDevice::capture: m_pMultiSourceFrameReader is nullptr\n");
			// this is bad news - perhaps throw?
			return; // @@@
		}

		IMultiSourceFrame* pMultiSourceFrame = nullptr;
		IDepthFrame* pDepthFrame = nullptr;
		IColorFrame* pColorFrame = nullptr;

		const golem::MSecTmU32 waitStep = 1;
		golem::MSecTmU32 timeWaited = 0;
		golem::Sleep timer;
		while (FAILED(hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame)))
		{
			// this is in CameraOpenNI, but suspect may be causing problem here
			// if (camera->isTerminating())	return;

			timer.msleep(waitStep);
			timeWaited += waitStep;
			if (timeWaited >= timeout)
			{
				camera->getContext().error("CameraKinectDevice::capture: failed to acquire frame within %d ms\n", timeout);
				// keep going - don't return with nothing; reset stopwatch @@@
				timeWaited = 0;
			}
		}
		
		const golem::SecTmReal systemTime1 = camera->getContext().getTimer().elapsed();

		if (SUCCEEDED(hr))
		{
			IDepthFrameReference* pDepthFrameReference = nullptr;

			hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
			}
			RELEASE_PTR(pDepthFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			IColorFrameReference* pColorFrameReference = nullptr;

			hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
			if (SUCCEEDED(hr))
			{
				hr = pColorFrameReference->AcquireFrame(&pColorFrame);
			}
			RELEASE_PTR(pColorFrameReference);
		}

		if (SUCCEEDED(hr))
		{
			INT64 nDepthTime = 0;
			IFrameDescription* pDepthFrameDescription = nullptr;
			int nDepthWidth = 0;
			int nDepthHeight = 0;
			UINT nDepthBufferSize = 0;
			UINT16 *pDepthBuffer = nullptr;

			IFrameDescription* pColorFrameDescription = nullptr;
			int nColorWidth = 0;
			int nColorHeight = 0;
			ColorImageFormat imageFormat = ColorImageFormat_None;
			UINT nColorBufferSize = 0;
			RGBQUAD *pColorBuffer = nullptr;

			// get depth frame data

			hr = pDepthFrame->get_RelativeTime(&nDepthTime);

			if (SUCCEEDED(hr))
				hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);

			if (SUCCEEDED(hr))
				hr = pDepthFrameDescription->get_Width(&nDepthWidth);

			if (SUCCEEDED(hr))
				hr = pDepthFrameDescription->get_Height(&nDepthHeight);

			if (SUCCEEDED(hr))
				hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);

			// get color frame data

			if (SUCCEEDED(hr))
				hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);

			if (SUCCEEDED(hr))
				hr = pColorFrameDescription->get_Width(&nColorWidth);

			if (SUCCEEDED(hr))
				hr = pColorFrameDescription->get_Height(&nColorHeight);

			if (SUCCEEDED(hr))
				hr = pColorFrame->get_RawColorImageFormat(&imageFormat);

			if (SUCCEEDED(hr))
			{
				if (imageFormat == ColorImageFormat_Bgra)
				{
					hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
				}
				else if (m_pColorRGBX)
				{
					pColorBuffer = m_pColorRGBX;
					nColorBufferSize = nColorWidth * nColorHeight * sizeof(RGBQUAD);
					hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
				}
				else
				{
					hr = E_FAIL;
				}
			}
			
			cv::Mat colorFrame;
			colorFrame.create(cColorHeight, cColorWidth, CV_8UC4);
			colorFrame.setTo(cv::Scalar(0, 0, 0, 0));
			//copying color buffer into cv::mat		
			memcpy(colorFrame.data, pColorBuffer, (nColorWidth * nColorHeight) * 4);
			//originally kinect has mirrored image
			cv::flip(colorFrame, colorFrame, 1);
			//converstion to pImage which is BGR 
			cv::Mat colorFrameT;
			cv::cvtColor(colorFrame, colorFrameT, CV_BGRA2BGR);
			
			if (mode & CameraKinect::MODE_COLOUR)
			{
				// initialize buffer
				if (pImage == nullptr)
					pImage.reset(new Image());
				pImage->reserve((int)1920, (int)1080);
				//printf("Before clonning frame\n");
				IplImage * img = cvCloneImage(&(IplImage)colorFrameT);
				cvCopy(img, pImage->image);
				cvReleaseImage(&img);
				//printf("After clonning frame\n");
				const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();
				//seting timestamp
				pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2);
			}
			//camera->getContext().debug("After colour\n");
			//needed for retain information of the begining of the buffer
			const UINT16* pBufferStart = pDepthBuffer;
			const UINT16* pBufferEnd = pDepthBuffer + (nDepthWidth * nDepthHeight);
			//copying depth buffer into cv::Mat
			long depthSizeP = (pBufferEnd - pBufferStart) * 2;

			cv::Mat depthFrame;
			depthFrame.create(cDepthHeight, cDepthWidth, CV_16UC1);
			depthFrame.setTo(cv::Scalar(0));
			memcpy(depthFrame.data, pBufferStart, depthSizeP);
			//originally kinect has mirrored image
			cv::flip(depthFrame, depthFrame, 1);
			//camera->getContext().debug("After getting depth data\n");
			//depth mode color data mapped into camera space (remember high resolution 1920x1080)
			if (mode & CameraKinect::MODE_DEPTH && (std::stoi(this->property.format.c_str()) == 101))
			{
				//camera->getContext().debug("In depth mode\n");
				if (pImage == nullptr)
					pImage.reset(new Image());
				pImage->reserve((int)1920, (int)1080);

				pImage->cloud.reset(new PointSeq((uint32_t)property.width, (uint32_t)property.height));
			
				//Obtaining the pointcloud
				hasImageStream = false;
				float missingPoint = std::numeric_limits<float>::quiet_NaN();

				hr = m_pCoordinateMapper->MapColorFrameToCameraSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, cColorHeight * cColorWidth, m_pCameraCoordinatesColor);

				golem::Point* pOutWidth = &pImage->cloud->front();
				CameraSpacePoint* pCamCWidth = m_pCameraCoordinatesColor;

				for (int y = 0; y < cColorHeight; ++y)
				{
					golem::Point* pOut = pOutWidth;
					CameraSpacePoint* pCamC = pCamCWidth;

					for (int x = 0; x < cColorWidth; ++x, pCamC++, ++pOut) {
						cv::Vec4b color;
						int abc = pCamC->Z;
						if (abc != 0) {

							pOut->x = pCamC->X;
							pOut->y = pCamC->Y;
							pOut->z = pCamC->Z;
						}
						else {
							pOut->x = pOut->y = pOut->z = missingPoint;
							//printf("Getting to this point4\n");
						}
						pOut->normal_x = pOut->normal_y = pOut->normal_z = golem::numeric_const<float>::ZERO;

						color = colorFrame.at<cv::Vec4b>(y, x);
						if (hasImageStream) {
							pOut->b = color[0];
							pOut->g = color[1];
							pOut->r = color[2];
							pOut->a = colour._rgba.a;
						}
						else {
							pOut->r = colour._rgba.r;
							pOut->g = colour._rgba.g;
							pOut->b = colour._rgba.b;
							pOut->a = colour._rgba.a;
						}
					}
					colorFrame += cColorWidth;
					pCamCWidth += cColorWidth;
					pOutWidth += cColorWidth;
				}
			}

			//depth mode depth data mapped into camera space
			if (mode & CameraKinect::MODE_DEPTH && (std::stoi(this->property.format.c_str()) == 102))
			{
				if (pImage == nullptr)
					pImage.reset(new Image());
				//camera->getContext().debug("In depth mode\n");
				pImage->cloud.reset(new PointSeq((uint32_t)property.width, (uint32_t)property.height));
				//Obtaining the pointcloud
				hasImageStream = false;
				float missingPoint = std::numeric_limits<float>::quiet_NaN();

				hr = m_pCoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth * nDepthHeight, (UINT16*)pDepthBuffer, cDepthHeight * cDepthWidth, m_pCameraCoordinatesDepth);

				golem::Point* pOutWidth = &pImage->cloud->front();
				CameraSpacePoint* pCamDWidth = m_pCameraCoordinatesDepth;

				for (int y = 0; y < cDepthHeight; ++y)
				{
					golem::Point* pOut = pOutWidth;
					CameraSpacePoint* pCamC = pCamDWidth;

					for (int x = 0; x < cDepthWidth; ++x, pCamC++, ++pOut) {
						cv::Vec4b color;
						//int abc = pCamC->Z;
						if (pCamC->Z != 0) {
							pOut->x = pCamC->X;
							pOut->y = pCamC->Y;
							pOut->z = pCamC->Z;
						}
						else {
							pOut->x = missingPoint;
							pOut->y = missingPoint;
							pOut->z = missingPoint;
							//	//printf("Getting to this point4\n");
						}
						pOut->normal_x = pOut->normal_y = pOut->normal_z = golem::numeric_const<float>::ZERO;

						/*color = colorframe.at<cv::vec4b>(y, x);
						if (hasimagestream) {
						pout->b = color[0];
						pout->g = color[1];
						pout->r = color[2];
						pout->a = colour._rgba.a;
						}*/
						//else {
						pOut->r = colour._rgba.r;
						pOut->g = colour._rgba.g;
						pOut->b = colour._rgba.b;
						pOut->a = colour._rgba.a;
						//}
					}
					//colorFrame += cDepthWidth;
					pCamDWidth += cDepthWidth;
					pOutWidth += cDepthWidth;
				}
				golem::Mat33 aMatrix;
				aMatrix.m11 = golem::Real(-1);
				aMatrix.m12 = golem::Real(0);
				aMatrix.m13 = golem::Real(0);
				aMatrix.m21 = golem::Real(0);
				aMatrix.m22 = golem::Real(-1);
				aMatrix.m23 = golem::Real(0);
				aMatrix.m31 = golem::Real(0);
				aMatrix.m32 = golem::Real(0);
				aMatrix.m33 = golem::Real(1);
				golem::Vec3 aVector;
				aVector.v1 = 0;
				aVector.v2 = 0;
				aVector.v3 = 0;
				golem::Mat34 aMatrix34;
				aMatrix34.R = aMatrix;
				aMatrix34.p = aVector;

				Cloud::PointSeqPtr pCloud = pImage->cloud;
				const Mat34 frame = Cloud::getSensorFrame(*pCloud);
				Cloud::transform(aMatrix34, *pCloud, *pCloud);
				Cloud::setSensorFrame(frame, *pCloud); // don't change the camera frame !!!!!!!!!!!!!!!!!!!!!!!!!!!!

				const golem::SecTmReal systemTime2 = camera->getContext().getTimer().elapsed();
				//seting timestamp
				pImage->timeStamp = REAL_HALF*(systemTime1 + systemTime2);
			}
			RELEASE_PTR(pDepthFrameDescription);
			RELEASE_PTR(pColorFrameDescription);
		}
		RELEASE_PTR(pDepthFrame);
		RELEASE_PTR(pColorFrame);
		RELEASE_PTR(pMultiSourceFrame);
	}

	bool assertValid(const HRESULT hr, const char* context, bool bThrow = true) {
		if (FAILED(hr)) {
			if (bThrow)
				throw Message(Message::LEVEL_CRIT, "%s: %s", context, "Failed to initialize Kinect\n");
			else
				camera->getContext().error("%s: %s", context, "Failed to initialize Kinect\n");
			return false;
		}
		return true;
	}

private:
	golem::U32 mode;

	IKinectSensor*          m_pKinectSensor;
	ICoordinateMapper*      m_pCoordinateMapper;
	//DepthSpacePoint*      m_pDepthCoordinates;
	//ColorSpacePoint*		m_pColorCoordinates;
	CameraSpacePoint*		m_pCameraCoordinatesColor;
	CameraSpacePoint*		m_pCameraCoordinatesDepth;

	IMultiSourceFrameReader* m_pMultiSourceFrameReader;

	RGBQUAD*                m_pColorRGBX;
	const RGBA              colour;

	bool hasImageStream;
	golem::MSecTmU32 timeout;
};

CameraKinect::CameraKinect(golem::Context& context) : CameraDepth(context) {}

void CameraKinect::create(const Desc& desc) {
	desc.assertValid(Assert::Context("CameraKinect::Desc."));

	colour = desc.colour;
	streamTimeOut = desc.streamTimeOut;
	device.reset(new CameraKinectDevice(this, desc.properties[0], desc.index, colour, (int)streamTimeOut));
	property = device->getProperty();
	properties = desc.properties;
	device.reset();

	CameraDepth::create(desc); // throws
}

golem::U32 CameraKinect::getMode(const std::string& str) {
	if (str.compare("depth") == 0 || str.compare("d") == 0)
		return CameraKinect::MODE_DEPTH;
	if (str.compare("colour") == 0 || str.compare("color") == 0 || str.compare("rgb") == 0)
		return CameraKinect::MODE_COLOUR;
	if (str.compare("ir") == 0)
		return CameraKinect::MODE_IR;
	throw golem::Message(golem::Message::LEVEL_CRIT, "CameraKinect::getMode(): Unknown camera mode %s", str.c_str());
}

void CameraKinect::start() {
	device.reset(new CameraKinectDevice(this, property, index, colour, (int)streamTimeOut));
	static_cast<CameraKinectDevice*>(device.get())->start();
}

void CameraKinect::stop() {
	static_cast<CameraKinectDevice*>(device.get())->stop();
	device.reset();
}

//------------------------------------------------------------------------------
