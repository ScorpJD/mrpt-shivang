#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CMarkerDetection.h>
#include <mrpt/detectors/CDetectableObject.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/utils/CImage.h>
// #include <mrpt/system/threads.h> // sleep()

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <aruco/aruco.h>

PIMPL_IMPLEMENT(aruco::MarkerDetector);
PIMPL_IMPLEMENT(aruco::CameraParameters);

using namespace mrpt::detectors;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace std;

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
using namespace cv;
#endif

void CArucoDetectionPolicy::init_params(const mrpt::utils::CConfigFileBase &config)
{
	PIMPL_CONSTRUCT(aruco::MarkerDetector, m_aruco_detector);
	PIMPL_CONSTRUCT(aruco::CameraParameters, m_aruco_cam_param);
	aruco::MarkerDetector::Params m_params ;
	// load configuration values
	m_tag_family = config.read_string("ArucoDetectionOptions","tag_family","ARUCO");
	m_params._thresParam1 = config.read_double("ArucoDetectionOptions","threshParam1",7);
	m_params._thresParam2 = config.read_double("ArucoDetectionOptions","threshParam2",7);
	m_params._thresParam1_range = config.read_double("ArucoDetectionOptions","threshParam1_range",0);
	m_params._subpix_wsize = config.read_int("ArucoDetectionOptions","subpix_wsize",4);
	m_params._markerWarpSize = config.read_int("ArucoDetectionOptions","markerWarpSize",56);
	m_params._borderDistThres = config.read_float("ArucoDetectionOptions","borderDistThres",0.05);
	m_params._minSize = config.read_float("ArucoDetectionOptions","minSize",0.04);
	m_params._maxSize = config.read_float("ArucoDetectionOptions","maxSize",0.95);
	m_params._minSize_pix = config.read_int("ArucoDetectionOptions","minSize_pix",25);
	m_marker_size = config.read_float("ArucoDetectionOptions","markerSize",-1);
	PIMPL_GET_REF(aruco::MarkerDetector, m_aruco_detector).setParams(m_params);
	PIMPL_GET_REF(aruco::MarkerDetector, m_aruco_detector).setDictionary(m_tag_family, 0.f);
}

void CArucoDetectionPolicy::detectMarkers(const mrpt::obs::CObservation *obs, vector_detectable_object &detected)
{
	// Obtain image from generic observation
	const CImage *img = NULL;
	const mrpt::utils::TCamera *cameraParams;
	if(IS_CLASS(obs,CObservationImage))
	{
		const CObservationImage* o = static_cast<const CObservationImage*>(obs);
		img = &o->image;
		cameraParams = &o->cameraParams;
	}

	if(!img)
	{
		// mrpt::system::sleep(2);
		return;
	}
	if(!PIMPL_GET_REF(aruco::CameraParameters, m_aruco_cam_param).isValid()){
		cv::Mat camMat(3, 3, CV_32F);
		cv::Mat distCoeff(1, 5, CV_32F);
		cv::Size size;
		for(int i = 0; i < 3; i++)
		{
			float* camMati = camMat.ptr<float>(i);
			for(int j = 0; j < 3; j++)
				camMati[j] = cameraParams->intrinsicParams(i, j);
		}
		float* distCoeffPtr = distCoeff.ptr<float>(0);
		for(int i = 0; i < 5; ++i)
			distCoeffPtr[i] = cameraParams->dist(i);
		size.height = cameraParams->nrows;
		size.width = cameraParams->ncols;
		PIMPL_GET_REF(aruco::CameraParameters, m_aruco_cam_param).setParams(camMat, distCoeff, size);
	}
	// Convert to IplImage and copy it
	const IplImage *image = img->getAs<IplImage>();

	cv::Mat matImage = cv::cvarrToMat(image);
	vector<aruco::Marker> markers = PIMPL_GET_REF(aruco::MarkerDetector,
												 m_aruco_detector).detect(matImage,
												 PIMPL_GET_REF(aruco::CameraParameters, m_aruco_cam_param),
												 m_marker_size);

	for (unsigned int i = 0; i < markers.size(); i++)
	{
		CDetectableMarker::Ptr markerObj = CDetectableMarker::Ptr(new CDetectableMarker);
		markerObj->m_id = markers[i].id;
		for (unsigned int j = 0; j < markers[i].size(); j++){
			markerObj->corners.push_back(make_pair(markers[i][j].x, markers[i][j].y));
		}
		if(m_marker_size > 0 && PIMPL_GET_REF(aruco::CameraParameters, m_aruco_cam_param).isValid()){
			float* TvecPtr = markers[i].Tvec.ptr<float>(0);
			float* RvecPtr = markers[i].Rvec.ptr<float>(0);
			for(int j = 0; j < 3; ++j){
				markerObj->m_pose.m_coords(j) = TvecPtr[j];
				markerObj->m_pose.m_rotvec(j) = RvecPtr[j];
			}
		}
		detected.push_back((CDetectableObject::Ptr)markerObj);
	}
}
