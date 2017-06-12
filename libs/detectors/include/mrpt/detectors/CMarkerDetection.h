#ifndef CMarkerDetection_H
#define CMarkerDetection_H

#include <mrpt/detectors/CObjectDetection.h>

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

			void* m_marker_detector;
			void* m_params;
			std::string m_tag_family;

		}; // End of class


		template <class DETECTION_POLICY>
		class DETECTORS_IMPEXP CMarkerDetection: public DETECTION_POLICY, public CObjectDetection
		{
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