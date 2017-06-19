#ifndef CMarkerDetection_H
#define CMarkerDetection_H

#include <mrpt/detectors/CObjectDetection.h>
#include <mrpt/obs/CObservation.h>
#include <mrpt/utils/CConfigFileBase.h>
#include <mrpt/utils/pimpl.h>

PIMPL_FORWARD_DECLARATION(namespace aruco { class MarkerDetector; })
PIMPL_FORWARD_DECLARATION(namespace aruco { class CameraParameters; })

namespace mrpt
{
	namespace detectors
	{
		/** 
		  * \ingroup mrpt_detectors_grp
		  */
		class DETECTORS_IMPEXP CArucoDetectionPolicy
		{
		public:

			/** Initialize marker detection */
			void init_params(const mrpt::utils::CConfigFileBase &cfg );

			void detectMarkers(const mrpt::obs::CObservation *obs, vector_detectable_object &detected);

		protected:

			PIMPL_DECLARE_TYPE(aruco::MarkerDetector, m_aruco_detector);
			PIMPL_DECLARE_TYPE(aruco::CameraParameters, m_aruco_cam_param);
			std::string m_tag_family;
			float m_marker_size;

		}; // End of class


		template <class DETECTION_POLICY>
		class DETECTORS_IMPEXP CMarkerDetection: public DETECTION_POLICY, public CObjectDetection
		{
		public:
			void init(const mrpt::utils::CConfigFileBase &cfg)
			{
				DETECTION_POLICY::init_params(cfg);
			}

			void detectObjects_Impl( const mrpt::obs::CObservation *obs, vector_detectable_object &detected)
			{
				DETECTION_POLICY::detectMarkers(obs, detected);
			}
		};
		typedef CMarkerDetection<CArucoDetectionPolicy> CArucoMarkerDetection;
	}
}

#endif