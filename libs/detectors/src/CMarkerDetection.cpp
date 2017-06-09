#include "detectors-precomp.h"  // Precompiled headers

#include <mrpt/detectors/CMarkerDetection.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
// #include <mrpt/system/threads.h> // sleep()

// Universal include for all versions of OpenCV
#include <mrpt/otherlibs/do_opencv_includes.h> 

#include <aruco/aruco.h>

using namespace mrpt::detectors;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace std;

#if MRPT_HAS_OPENCV && MRPT_OPENCV_VERSION_NUM>=0x200
using namespace cv;
#endif

#define M_MARKER_DETECTOR_PTR    (reinterpret_cast<aruco::MarkerDetector*>(m_marker_detector))
#define M_ARUCO_PARAMS_PTR    (reinterpret_cast<aruco::MarkerDetector::Params*>(m_params))

void CArucoDetectionPolicy::init_params(const mrpt::utils::CConfigFileBase &config)
{
	m_marker_detector = new aruco::MarkerDetector;
	// load configuration values
	m_tag_family 							= config.read_string("ArucoDetectionOptions","tag_family","ARUCO");
	M_ARUCO_PARAMS_PTR->_thresParam1 		= config.read_double("ArucoDetectionOptions","threshParam1",7);
	M_ARUCO_PARAMS_PTR->_thresParam2 		= config.read_double("ArucoDetectionOptions","threshParam2",7);
	M_ARUCO_PARAMS_PTR->_thresParam1_range 	= config.read_double("ArucoDetectionOptions","threshParam1_range",0);
	M_ARUCO_PARAMS_PTR->_subpix_wsize 		= config.read_int("ArucoDetectionOptions","subpix_wsize",4);
	M_ARUCO_PARAMS_PTR->_markerWarpSize 	= config.read_int("ArucoDetectionOptions","markerWarpSize",56);
	M_ARUCO_PARAMS_PTR->_borderDistThres 	= config.read_float("ArucoDetectionOptions","borderDistThres",0.05);
	M_ARUCO_PARAMS_PTR->_minSize 			= config.read_float("ArucoDetectionOptions","minSize",0.04);
	M_ARUCO_PARAMS_PTR->_maxSize 			= config.read_float("ArucoDetectionOptions","maxSize",0.95);
	M_ARUCO_PARAMS_PTR->_minSize_pix 		= config.read_int("ArucoDetectionOptions","minSize_pix",25);
	M_MARKER_DETECTOR_PTR->setParams(*M_ARUCO_PARAMS_PTR);
	M_MARKER_DETECTOR_PTR->setDictionary(m_tag_family, 0.f);
}

void CArucoDetectionPolicy::detectMarkers(const mrpt::obs::CObservation *obs, vector_detectable_object &detected)
{
	// Obtain image from generic observation
	const mrpt::utils::CImage *img = NULL;

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
	const CImage img_gray( *img, FAST_REF_OR_CONVERT_TO_GRAY );

	// Convert to IplImage and copy it
	const IplImage *image = img_gray.getAs<IplImage>();

	cv::Mat matImage = cv::cvarrToMat(image);

	vector<aruco::Marker> markers = M_MARKER_DETECTOR_PTR->detect(matImage);

	for (unsigned int i = 0; i < markers.size(); i++)
	{
		cout << markers[i] << endl;
		CDetectableMarker::Ptr markerObj = CDetectableMarker::Ptr(new CDetectableMarker());
		markerObj->m_id = markers[i].id;
		for (unsigned int j = 0; j < markers[i].size(); ++i){
			markerObj->corners.push_back(make_pair(markers[i][j].x, markers[i][j].y));
		}
		detected.push_back((CDetectableObject::Ptr)markerObj);
	}
}
