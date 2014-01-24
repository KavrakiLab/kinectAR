#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fstream>

#include <sns.h>
#include <amino.h>
#include <ach.h>
#include <syslog.h>

#include <cv.h>
#include <cxcore.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/segmentation/impl/organized_multi_plane_segmentation.hpp>

#include <MarkerDetector.h>
#include <GlutViewer.h>

class KinectMarker 
{
public:
	// constructor
	KinectMarker();

	// Update the marker data
	void Update(alvar::MarkerData* newData);
	
	// sort the corner points
	void SortCornerPoints();
	
	// returns sorted corner points
	inline std::vector<alvar::PointDouble> GetCornerPoints() {return sortedMarkerCornersImg;}
	
	// set the point cloud corresponding to this marker
	inline void SetPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr pcl) {pcloud = pcl;}
	
	// calculates the normal vector
	void GetNormalVector();
	
	// print out the alvar pose
	void PrintAlvarPose();
	
	// print out the alvar pose
	void PrintKinectPose();
	
	// to quaternion conversion
	void ToQuaternion(double m[3][3], double* quat);

	// calculate the point cloud
	void CalculatePointCloud(cv::Mat depthMap);
	
	inline double* GetKinectQuat() {return kinectQuat;}
	inline Eigen::Vector3f GetKinectPos() {return kinectPos;}
	
protected:
	// fit plane into a point cloud
	void FitPlane();
	
	// turn a point into a cv::Mat
	cv::Mat PointToMat(pcl::PointXYZRGBA p);
	cv::Mat PointToMat(pcl::PointXYZ p);
	
	std::vector<alvar::PointDouble> markerPointsImg;	
	std::vector<alvar::PointDouble> markerCornersImg;
	std::vector<alvar::PointDouble> sortedMarkerCornersImg;
	std::vector<alvar::PointDouble> markerCornersCentered;
	std::vector<alvar::PointDouble> markerPoints3D;

	alvar::MarkerData* alvarData;

	alvar::PointDouble position;
	pcl::PointXYZ normal;
	CvMat orientation;
	
	// current point cloud of all marker points
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcloud;
	std::vector<pcl::PointXYZRGBA> markerCorners3D;
	
	// the current alvar pose
	alvar::Pose alvarPose;
	
	// the current kinect quat
	Eigen::Vector3f kinectPos;
	double kinectQuat[4];
	
	bool visible;
};
