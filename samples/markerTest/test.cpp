#include <mrpt/obs/CObservationImage.h>
#include <mrpt/detectors.h>
using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::detectors;

#include <mrpt/examples_config.h>
#include <mrpt/mrpt_paths_config.h>
string myInitFile( MRPT_EXAMPLES_BASE_DIRECTORY + string("markerTest/MARKER_TEST.INI") );
string myCamFile( MRPT_SOURCE_BASE_DIRECTORY + string("/share/mrpt/datasets/markers/CAMERA_PARAM.INI"));


int main(int argc, char** argv){
	CObservationImage::Ptr obsImg = CObservationImage::Ptr(new CObservationImage);
	CImage img;
	string imgName;
	if(argc > 1){
		imgName = string(argv[1]);
	}
	else{
		cout << "Provide Filename while running" << endl;
		return 0;
	}
	if (!img.loadFromFile(imgName))
		throw std::runtime_error("Can't load demo image!");
	obsImg->image = img;

	//Load Camera Param
	CStringList camLst;	
	CConfigFileMemory camCfg; 
	camLst.loadFromFile(myCamFile);
	camCfg.setContent(camLst);
	obsImg->cameraParams.loadFromConfigFile(camCfg, string("CameraParams"));

	//Load detection parameters
	CStringList lst;	
	CConfigFileMemory cfg; 
	lst.loadFromFile(myInitFile);
	cfg.setContent(lst);
	CMarkerDetection<CArucoDetectionPolicy> det;
	det.init(cfg);

	vector_detectable_object detected;
	det.detectObjects( obsImg, detected );
	if ( detected.size() > 0 )
	{	
		for ( unsigned int i = 0; i < detected.size(); i++ )
		{
			ASSERT_( IS_CLASS(detected[i],CDetectableMarker ) )
			CDetectableMarker::Ptr obj = std::dynamic_pointer_cast<CDetectableMarker>( detected[i] );
			cout << "ID " << obj->m_id << endl;
			cout << "Corners";
			for( unsigned int j = 0; j < obj->corners.size(); ++j){
				cout << "X" << j << " " << obj->corners[j].first << " Y" << j << " " << obj->corners[j].second << endl;
			}
			cout << "Pose[x, y, z]"
				<< obj->m_pose.m_coords << endl;
			cout << "Pose[vx, vy, vz]"
				<< obj->m_pose.m_rotvec << endl;
		}
	}

}