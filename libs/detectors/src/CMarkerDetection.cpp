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
	PIMPL_GET_REF(aruco::MarkerDetector, m_aruco_detector).setParams(m_params);
	PIMPL_GET_REF(aruco::MarkerDetector, m_aruco_detector).setDictionary(m_tag_family, 0.f);
}

void CArucoDetectionPolicy::detectMarkers(const mrpt::obs::CObservation *obs, vector_detectable_object &detected)
{
	// Obtain image from generic observation
	const CImage *img = NULL;

	if(IS_CLASS(obs,CObservationImage))
	{
		const CObservationImage* o = static_cast<const CObservationImage*>(obs);
		img = &o->image;
	}

	if(!img)
	{
		// mrpt::system::sleep(2);
		return;
	}

	// Some needed preprocessing
	//const CImage img_gray( *img, FAST_REF_OR_CONVERT_TO_GRAY );

	// Convert to IplImage and copy it
	const IplImage *image = img->getAs<IplImage>();
	cv::Mat matImage = cv::cvarrToMat(image);
	vector<aruco::Marker> markers = PIMPL_GET_REF(aruco::MarkerDetector, m_aruco_detector).detect(matImage);

	for (unsigned int i = 0; i < markers.size(); i++)
	{
		CDetectableMarker::Ptr markerObj = CDetectableMarker::Ptr(new CDetectableMarker);
		markerObj->m_id = markers[i].id;
		for (unsigned int j = 0; j < markers[i].size(); j++){
			markerObj->corners.push_back(make_pair(markers[i][j].x, markers[i][j].y));
		}
		detected.push_back((CDetectableObject::Ptr)markerObj);
	}
}
