#include <mrpt/obs/CObservationImage.h>
#include <mrpt/detectors.h>
using namespace std;
using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace mrpt::detectors;

#include <mrpt/examples_config.h>
string myInitFile( MRPT_EXAMPLES_BASE_DIRECTORY + string("markerTest/MARKER_TEST.INI") );

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
			cout << obj->m_id << endl;
			for( unsigned int j = 0; j < obj->corners.size(); ++j){
				cout << obj->corners[j].first << " " << obj->corners[j].second << endl;
			}
		}
	}

}