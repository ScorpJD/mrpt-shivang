#include <mrpt/detectors.h>
#include <mrpt/obs/CObservationImage.h>

#include <gtest/gtest.h>

using namespace mrpt;
using namespace mrpt::detectors;
using namespace mrpt::obs;
using namespace mrpt::utils;
using namespace std;

#include <mrpt/examples_config.h>
#include <mrpt/mrpt_paths_config.h>
string testInitFile( MRPT_EXAMPLES_BASE_DIRECTORY + string("markerTest/MARKER_TEST.INI") );
string testCamFile( MRPT_SOURCE_BASE_DIRECTORY + string("/share/mrpt/datasets/markers/CAMERA_PARAM.INI") );
string testImage( MRPT_SOURCE_BASE_DIRECTORY + string("/share/mrpt/datasets/markers/multiple_markers.jpg") );
string testFile( MRPT_SOURCE_BASE_DIRECTORY + string("/share/mrpt/datasets/markers/out.txt") );

template <class DETECTION_POLICY>
class MarkerDetection: public::testing::Test
{
public:
	vector_detectable_object markers;
	unsigned int markerSize;
	vector < int > ids;
	vector < vector < pair< float, float > > > corners;
	vector < vector < float > > poses;

	virtual void SetUp()
	{
		CMarkerDetection<DETECTION_POLICY> det;
		CObservationImage obsImg;
		CImage img;
		CStringList strLst;
		CConfigFileMemory cfgFile;

		try
		{
			if (!img.loadFromFile(testImage))
				throw std::runtime_error("Can't load demo image!");
			obsImg.image = img;
			strLst.loadFromFile(testCamFile);
			cfgFile.setContent(strLst);
			obsImg.cameraParams.loadFromConfigFile(cfgFile, string("CameraParams"));

			strLst.loadFromFile(testInitFile);
			cfgFile.setContent(strLst);
			det.init(cfgFile);
			det.detectObjects(&obsImg, markers);

			strLst.loadFromFile(testFile);
			cfgFile.setContent(strLst);
			markerSize = cfgFile.read_int("MarkerTestFile","markerSize",0, true);
			for(unsigned int i = 0; i < markerSize; ++i)
			{
				vector < float > array;
				vector < pair < float, float > > temp_corner;
				vector < float > temp_pose;
				int id = cfgFile.read_int("MarkerTestFile", string("id" + to_string(i)), 0, true);
				ids.push_back(id);
				cfgFile.read_vector("MarkerTestFile",string("corners" + to_string(i)),vector<float>(),array,true);
				if(array.size()%2 != 0)
					throw std::runtime_error("Expected vector in field 'corners' to be in pair");
				for(unsigned int j = 0; j < array.size(); j +=2)
					temp_corner.push_back(make_pair(array[j], array[j+1]));
				corners.push_back(temp_corner);
				cfgFile.read_vector("MarkerTestFile", string("pose" + to_string(i)), vector<float>(), temp_pose, true);
				if(temp_pose.size()<6 || temp_pose.size() > 6)
					throw std::runtime_error("Expected 6D pose in [x, y, z, vx, vy, vz] format");
				poses.push_back(temp_pose);
			}
		}
		catch (std::exception &e)
		{
			std::cout << "MRPT exception caught: " << e.what() << std::endl;
			return;
		}
		catch (...)
		{
			printf("Another exception!!");
			return;
		}
	}

	virtual void TearDown()
	{
	}
};

typedef MarkerDetection<CArucoDetectionPolicy> ArucoDetection;

TEST_F(ArucoDetection, CornersAndPoseTest)
{
	ASSERT_EQ(markerSize, markers.size());
	for(unsigned int i = 0; i < markers.size(); ++i)
	{
		ASSERT_(IS_CLASS(markers[i],CDetectableMarker))
		CDetectableMarker::Ptr marker = std::dynamic_pointer_cast<CDetectableMarker>(markers[i]);
		ASSERT_EQ(ids[i], marker->m_id);
		for(unsigned int j = 0; j < marker->corners.size(); ++j){
			ASSERT_NEAR(corners[i][j].first, marker->corners[j].first, 1e-2);
			ASSERT_NEAR(corners[i][j].first, marker->corners[j].first, 1e-2);
		}
		ASSERT_NEAR(marker->m_pose.m_coords(0), poses[i][0], 1e-5);
		ASSERT_NEAR(marker->m_pose.m_coords(1), poses[i][1], 1e-5);
		ASSERT_NEAR(marker->m_pose.m_coords(2), poses[i][2], 1e-5);
		ASSERT_NEAR(marker->m_pose.m_rotvec(0), poses[i][3], 1e-5);
		ASSERT_NEAR(marker->m_pose.m_rotvec(1), poses[i][4], 1e-5);
		ASSERT_NEAR(marker->m_pose.m_rotvec(2), poses[i][5], 1e-5);
	}
}
