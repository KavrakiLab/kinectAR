/* -*- mode: C++; c-basic-offset: 8; indent-tabs-mode: t;  -*- */
#include "kinectMarker.h"
#include <math.h>

//using namespace cv;

int sign(double x)
{
	return (x > 0)? 1 : -1;
}

KinectMarker::KinectMarker()
{
	visible = false;
	useOSG  = false;
}

void KinectMarker::AddToSceneGraph(osg::Group *root)
{
	useOSG = true;
	transf = new osg::PositionAttitudeTransform();
	xTrans = new osg::PositionAttitudeTransform();
	yTrans = new osg::PositionAttitudeTransform();
	zTrans = new osg::PositionAttitudeTransform();

	// zuerst gruppenknoten fuer auto
	root->addChild(transf);

	// dann fgen wir eine box als geometrie ein
	geometry = new osg::Geode;
	shape    = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),2.8f,2.8f,0.2f));
	shape->setColor(osg::Vec4(1,1,1,0));
	geometry->addDrawable(shape);

	transf->addChild(geometry);

	// add the helper bodies
	osg::Geode* xgeom = new osg::Geode;
	osg::ShapeDrawable* xshape    = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),0.5f));
	xshape->setColor(osg::Vec4(1,0,0,0));
	xgeom->addDrawable(xshape);

	osg::Geode* zgeom = new osg::Geode;
	osg::ShapeDrawable* zshape    = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),0.5f));
	zshape->setColor(osg::Vec4(0,0,1,0));
	zgeom->addDrawable(zshape);

	osg::Geode* ygeom = new osg::Geode;
	osg::ShapeDrawable* yshape    = new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(0.0f,0.0f,0.0f),0.5f));
	yshape->setColor(osg::Vec4(0,1,0,0));
	ygeom->addDrawable(yshape);

	xTrans->addChild(xgeom);
	yTrans->addChild(ygeom);
	zTrans->addChild(zgeom);

	transf->addChild(xTrans);
	transf->addChild(yTrans);
	transf->addChild(zTrans);
}

void KinectMarker::Update(alvar::MarkerData* newData)
{
	markerPointsImg.clear();
	markerCornersImg.clear();
	markerCorners3D.clear();

	// set the pointer to incoming data
	alvarData = newData;

	// see if visible
	if(alvarData->GetError() == 0.0)
		visible = true;
	else
	{
		visible = false;
	}


	// the current alvar pose
	alvarPose = newData->pose;



	markerCornersImg      = newData->marker_corners_img;
	markerCornersCentered = newData->marker_corners;
	//markerPointsImg       = newData->ros_marker_points_img;
	markerPointsImg       = newData->marker_points;

	// blow up maker points in image
	if(visible)
	{
		for(int i = -4; i < 4; i+=2)
			for(int j = -4; j < 4; j+= 2)
				for(int k = 0; k < 80; k++)
				{
					alvar::PointDouble p = markerPointsImg[k];
					p.x += i;
					p.y += j;
					markerPointsImg.push_back(p);
				}
	}

	// sort corner points
	SortCornerPoints();

	if(useOSG && visible)
	{
		// set position
		osg::Vec3 pos;
		pos[0] = alvarPose.translation[0];
		pos[1] = alvarPose.translation[1];
		pos[2] = alvarPose.translation[2];
			//pos[i] = kinectPos[i];

		// set quaternion
		osg::Quat quat;

		double tmp[4];
		CvMat q = cvMat(4, 1, CV_64F, tmp);
		alvarPose.GetQuaternion(&q);
		double* alvar_quat = (double*)q.data.ptr;
		quat[0] = alvar_quat[1];
		quat[1] = alvar_quat[2];
		quat[2] = alvar_quat[3];
		quat[3] = alvar_quat[0];

		transf->setPosition( pos );
		transf->setAttitude( quat );

	}
	else if(useOSG)
	{
		transf->setPosition(osg::Vec3(0,0,0));
	}
}

/*void KinectMarker::CalculatePointCloudOpenNI(cv::Mat depthMap)
{
	float wx, wy, wz;
		cv::Point3f point = newPointArea3D[k][i][j].getCenter();
		openni::CoordinateConverter::convertDepthToWorld(depthStream,
		    point.x,
		    point.y,
		    point.z,
		    &wx,
		    &wy,
		    &wz);
}*/

pcl::PointCloud<pcl::PointXYZ>::Ptr KinectMarker::CalculateCorner3D(cv::Mat depthMap)
{
	// create a point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// set the depth intrinsics
	cv::Mat depthIntrinsics;
	cv::Mat depthIntrinsicsInverse;

	int modex = 640;
	int modey = 480;

	double fx_v = (modex + modey) * 0.5;
	double fy_v = fx_v;
	double cx_v = modex * 0.5;
	double cy_v = modey * 0.5;

	cx_v = 3.1182863810012958e+02; // +/- 0.015
	cy_v = 2.4457008413833782e+02; // +/- 0.015
	fx_v = 5.8661298611948848e+02; // +/- 0.085
	fy_v = 5.8619682256008525e+02; // +/- 0.096

	double calibData[16] = {
		fx_v,	0.0,	cx_v,	0.0,
		0.0,	fy_v,   cy_v,	0.0,
		0.0,	0.0,	1.0,	0.0,
		0.0,	0.0,	0.0,	1.0
	};

	depthIntrinsics = cv::Mat(4, 4, CV_64F, &calibData);
	depthIntrinsicsInverse = depthIntrinsics.inv();

	// Fill in the cloud data
	cloud->width  = 4;
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	for (int j = 0; j < sortedMarkerCornersImg.size(); j++)
	{
		int w = sortedMarkerCornersImg[j].x;
		int h = sortedMarkerCornersImg[j].y;

		// scale x and y to 640x480, because we found them in highres image
		w /= 2;
		h /= 2;

		//cv::Point3f tmp(w,h,1.0);
		cv::Mat tmpPoint(4,1,CV_64F);
		tmpPoint.at<double>(0,0)=w;
		tmpPoint.at<double>(1,0)=h;
		tmpPoint.at<double>(2,0)=1.0;
		tmpPoint.at<double>(3,0)=1.0;

		// calculate the depth in meters
		double distance;
		if(w > -1 && h > -1) distance = depthMap.at<unsigned short>(h,w)/10.0;
		else distance = 0;

		// check if within range of sweet spot
		if(distance > 0 && distance < 300.0)
		{
			cv::Mat pointDepth3D  = depthIntrinsicsInverse * (tmpPoint * (distance));

			pcl::PointXYZ point3D;
			point3D.x = pointDepth3D.at<double>(0,0);
			point3D.y = pointDepth3D.at<double>(1,0);
			point3D.z = pointDepth3D.at<double>(2,0);

			cloud->points[j].x = point3D.x;
			cloud->points[j].y = point3D.y;
			cloud->points[j].z = point3D.z;
		}

	}
	scorner3D = cloud;
	return cloud;
}

void KinectMarker::CalculatePointCloud(cv::Mat depthMap)
{
	// create a point cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

	// set the depth intrinsics
	cv::Mat depthIntrinsics;
	cv::Mat depthIntrinsicsInverse;

	int modex = 640;
	int modey = 480;

	double fx_v = (modex + modey) * 0.5;
	double fy_v = fx_v;
	double cx_v = modex * 0.5;
	double cy_v = modey * 0.5;

	//cx_v = 3.1182863810012958e+02; // +/- 0.015
	//cy_v = 2.4457008413833782e+02; // +/- 0.015
	//fx_v = 5.8661298611948848e+02; // +/- 0.085
	//fy_v = 5.8619682256008525e+02; // +/- 0.096

	double calibData[16] = {
		fx_v,	0.0,	cx_v,	0.0,
		0.0,	fy_v,   cy_v,	0.0,
		0.0,	0.0,	1.0,	0.0,
		0.0,	0.0,	0.0,	1.0
	};

	depthIntrinsics = cv::Mat(4, 4, CV_64F, &calibData);
	depthIntrinsicsInverse = depthIntrinsics.inv();

	// append sortedMarkerCornersImg to
	markerPointsImg.insert(markerPointsImg.end(), sortedMarkerCornersImg.begin(), sortedMarkerCornersImg.end());

	// Fill in the cloud data
	cloud->width  = markerPointsImg.size();
	cloud->height = 1;
	cloud->points.resize (cloud->width * cloud->height);

	// calculate the 3d location of inlier points
	pcl::PointXYZ center;
	cv::Mat sumVec = cv::Mat(3, 1, CV_64F);

	for (int j = 0; j < markerPointsImg.size(); j++)
	{
		int w = markerPointsImg[j].x;
		int h = markerPointsImg[j].y;

		// scale x and y to 640x480, because we found them in highres image
		w /= 2;
		h /= 2;

		//cv::Point3f tmp(w,h,1.0);
		cv::Mat tmpPoint(4,1,CV_64F);
		tmpPoint.at<double>(0,0)=w;
		tmpPoint.at<double>(1,0)=h;
		tmpPoint.at<double>(2,0)=1.0;
		tmpPoint.at<double>(3,0)=1.0;

		// calculate the depth in meters
		double distance;
		if(w > -1 && h > -1) distance = depthMap.at<unsigned short>(h,w)/10.0;
		else distance = 0;

		// check if within range of sweet spot
		if(distance > 0 && distance < 300.0)
		{
			cv::Mat pointDepth3D  = depthIntrinsicsInverse * (tmpPoint * (distance));

			pcl::PointXYZ point3D;
			point3D.x = pointDepth3D.at<double>(0,0);
			point3D.y = pointDepth3D.at<double>(1,0);
			point3D.z = pointDepth3D.at<double>(2,0);

			cloud->points[j].x = point3D.x;
			cloud->points[j].y = point3D.y;
			cloud->points[j].z = point3D.z;

			cv::Mat tmp = PointToMat(point3D);
			sumVec = sumVec+tmp;

			for(int t = 0; t < 0; t++)
				if(!std::isfinite(tmp.at<double>(t,0)))
				{
					std::cout << "Found NAN!" << std::endl;
					continue;

				}
		}
		else
		{
			if(!std::isfinite(w) || std::isfinite(h))
			{
				cloud->points[j].x = 0;
				cloud->points[j].y = 0;
				cloud->points[j].z = 0;
				continue;
			}
			std::cout << w << " " << h << " " << (j > markerPointsImg.size() - 5)  << std::endl;
			distance = alvarPose.translation[2];
			cv::Mat pointDepth3D  = depthIntrinsicsInverse * (tmpPoint * (distance));

			pcl::PointXYZ point3D;
			point3D.x = pointDepth3D.at<double>(0,0);
			point3D.y = pointDepth3D.at<double>(1,0);
			point3D.z = pointDepth3D.at<double>(2,0);

			cloud->points[j].x = point3D.x;
			cloud->points[j].y = point3D.y;
			cloud->points[j].z = point3D.z;

			cv::Mat tmp = PointToMat(point3D);
			sumVec = sumVec+tmp;

			for(int t = 0; t < 0; t++)
				if(!std::isfinite(tmp.at<double>(t,0)))
				{
					std::cout << "Found NAN!" << std::endl;
					continue;
				}
		}

	}

	// set the internal point cloud
	pcloud = cloud;

	sumVec = sumVec/(double)pcloud->size();
	//std::cout << "sum: " << sumVec.at<double>(0,0) << " " << sumVec.at<double>(1,0) << " " << sumVec.at<double>(2,0) << std::endl;
}

void KinectMarker::SortCornerPoints()
{
	// create vector
	if(sortedMarkerCornersImg.size() < 4)
		sortedMarkerCornersImg = std::vector<alvar::PointDouble>(4);

	for(int i = 0; i < markerCornersImg.size(); i++)
	{
		// 1,1
		if(markerCornersCentered[i].x > 0 && markerCornersCentered[i].y > 0)
			sortedMarkerCornersImg[0] = markerCornersImg[i];
		// -1,1
		if(markerCornersCentered[i].x < 0 && markerCornersCentered[i].y > 0)
			sortedMarkerCornersImg[1] = markerCornersImg[i];
		// -1,-1
		if(markerCornersCentered[i].x < 0 && markerCornersCentered[i].y < 0)
			sortedMarkerCornersImg[2] = markerCornersImg[i];
		// 1,-1
		if(markerCornersCentered[i].x > 0 && markerCornersCentered[i].y < 0)
			sortedMarkerCornersImg[3] = markerCornersImg[i];
	}
	/*std::cout << sortedMarkerCornersImg.size() << std::endl;
	for(int i = 0; i < 4; i++)
	{
		std::cout << "M " << alvarData->GetId() << " : " << sortedMarkerCornersImg[i].x << " " << sortedMarkerCornersImg[i].y << std::endl;
	}*/
}

void KinectMarker::GetNormalVector()
{
	if(!visible)
		return;

	// get the marker corners
	int pclSize = scorner3D->points.size();

	pcl::PointXYZ c1 = scorner3D->points[0];
	pcl::PointXYZ c2 = scorner3D->points[1];
	pcl::PointXYZ c3 = scorner3D->points[2];
	pcl::PointXYZ c4 = scorner3D->points[3];

	double a=0, b=0, c=0, d=0;
	pcl::ModelCoefficients::Ptr coeff = fitPlane2(pcloud);
	extractFrame(*coeff, c1, c2, c3, c4);
}

void KinectMarker::PrintKinectPose()
{
	if(!visible)
		return;

	std::cout << "Kinect  (Pos): " << kinectPos[0] << " " << kinectPos[1] << " " << kinectPos[2] << std::endl;
	std::cout << "Kinect (Quat): " << kinectQuat[0] << " " << kinectQuat[1] << " " << kinectQuat[2] << " " << kinectQuat[3] << std::endl;
}

void KinectMarker::PrintAlvarPose()
{
	if(!visible)
		return;

	double tmp[4];
	CvMat q = cvMat(4, 1, CV_64F, tmp);
	alvarPose.GetQuaternion(&q);
	double* alvar_quat = (double*)q.data.ptr;

	SNS_LOG( LOG_DEBUG,  "Alvar\t[ %f\t%f\t%f\t%f\t|\t%f\t%f\t%f ]\n",
		 alvar_quat[1], alvar_quat[2], alvar_quat[3], alvar_quat[0],
		 alvarPose.translation[0] , alvarPose.translation[1] , alvarPose.translation[2] );
}

void KinectMarker::ToQuaternion(double  m[3][3], double* quat)
{
	double qw = std::sqrt(std::max(0.0, 1 + m[0][0] + m[1][1] + m[2][2]) ) / 2.0;
	double qx = std::sqrt(std::max(0.0, 1 + m[0][0] - m[1][1] - m[2][2]) ) / 2.0;
	double qy = std::sqrt(std::max(0.0, 1 - m[0][0] + m[1][1] - m[2][2]) ) / 2.0;
	double qz = std::sqrt(std::max(0.0, 1 - m[0][0] - m[1][1] + m[2][2]) ) / 2.0;

	qx *= sign(qx * (m[2][1] - m[1][2]) );
	qy *= sign(qy * (m[0][2] - m[2][0]) );
	qz *= sign(qz * (m[1][0] - m[0][1]) );

	quat[0] = qw;
	quat[1] = qx;
	quat[2] = qy;
	quat[3] = qz;
}

// Extract and normalize plane coefficients
int KinectMarker::getCoeffs(const pcl::ModelCoefficients& coeffs, double* a, double* b,
		 double* c, double* d)
  {
    if(coeffs.values.size() != 4)
      return -1;
    const double s = coeffs.values[0]*coeffs.values[0] +
      coeffs.values[1]*coeffs.values[1] + coeffs.values[2]*coeffs.values[2];
    if(fabs(s) < 1e-6)
      return -1;
    *a = coeffs.values[0]/s;
    *b = coeffs.values[1]/s;
    *c = coeffs.values[2]/s;
    *d = coeffs.values[3]/s;
    return 0;
}

// Project point onto plane
osg::Vec3 KinectMarker::project(pcl::PointXYZ p, const double a, const double b,
		 const double c, const double d)
{
    const double t = a*p.x + b*p.y + c*p.z + d;
    osg::Vec3 res;
    res[0] = p.x-t*a;
    res[1] = p.y-t*b;
    res[2] = p.z-t*c;

    return res;
}

int KinectMarker::extractFrame (const pcl::ModelCoefficients& coeffs,
		 const pcl::PointXYZ p1, const pcl::PointXYZ p2,
		 const pcl::PointXYZ p3, const pcl::PointXYZ p4)
{
	// Get plane coeffs and project points onto the plane
	double a=0, b=0, c=0, d=0;
	if(getCoeffs(coeffs, &a, &b, &c, &d) < 0)
	return -1;

		if(c > 0)
		{
			a = -a;
			b = -b;
			c = -c;
		}

	osg::Vec3 q1 = project(p1, a, b, c, d);
	osg::Vec3 q2 = project(p2, a, b, c, d);
	osg::Vec3 q3 = project(p3, a, b, c, d);
	osg::Vec3 q4 = project(p4, a, b, c, d);

	// Make sure points aren't the same so things are well-defined
	if((q2-q1).length() < 1e-3)
	return -1;

	// (inverse) matrix with the given properties
	osg::Vec3 v = (q2-q1);
	v.normalize();
	//std::cout << "v1: " << v[0] << " " << v[1] << " " << v[2] << std::endl;
	osg::Vec3 v2 = (q3-q4);
	v2.normalize();
	//std::cout << "v2: " << v2[0] << " " << v2[1] << " " << v2[2] << std::endl;
	//v = (v + v2)/2.0;

	/*	std::cout << "------------" << alvarData->GetId() << std::endl;
		std::cout << q1[0]<< " " << q1[1] << " " << q1[2] << std::endl;
		std::cout << q2[0]<< " " << q2[1] << " " << q2[2] << std::endl;
		std::cout << q3[0]<< " " << q3[1] << " " << q3[2] << std::endl;
		std::cout << q4[0]<< " " << q4[1] << " " << q4[2] << std::endl;
		std::cout << "------------" << std::endl;	*/


	osg::Vec3 n(a, b, c);
	osg::Vec3 w = -v ^ n;

	// initialize rotation matrix
	double  m[3][3] = {
		{v[0], v[1], v[2]},
		{w[0], w[1], w[2]},
		{n[0], n[1], n[2]}
	};

	// Possibly flip things based on third point
	osg::Vec3 diff = (q4-q3);
	diff.normalize();

	//xTrans->setPosition(v*5.0);
	//yTrans->setPosition(w*5.0);
	//zTrans->setPosition(n*5.0);


	//std::cout << "New Normal: " << n[0] << " " << n[1] << " " << n[2] << std::endl;

	//ROS_INFO_STREAM("w = " << w << " and d = " << diff);
	/*if (w * diff < 0)
	{
		//ROS_INFO_STREAM("Flipping normal based on p3. Current value: " << m);
		m[0][1] = -m[0][1];
		m[1][1] = -m[1][1];
		m[2][1] = -m[2][1];

		m[0][2] = -m[0][2];
		m[1][2] = -m[1][2];
		m[2][2] = -m[2][2];
		//ROS_INFO_STREAM("New value: " << m);
	}*/

	// Invert and return
	//retmat = m.inverse();
	//cerr << "Frame is " << retmat << endl;

	double kquat[4];
	ToQuaternion(m, kquat);

	osg::Quat quat;
	quat[0] = kquat[1];
	quat[1] = kquat[2];
	quat[2] = kquat[3];
	quat[3] = kquat[0];

	kinQuat = quat*osg::Quat(osg::DegreesToRadians(180.0), osg::Vec3(0,0,1));
	kinQuat[3] = -kinQuat[3];

	// set the kinectQuat
	kinectQuat[0] = kinQuat[0];
	kinectQuat[1] = kinQuat[1];
	kinectQuat[2] = kinQuat[2];
	kinectQuat[3] = kinQuat[3];

	//transf->setAttitude(kinQuat);

	return 0;
}

pcl::ModelCoefficients::Ptr  KinectMarker::fitPlane2(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::ModelCoefficients::Ptr coeff (new pcl::ModelCoefficients);
	//boost::shared_ptr<std::vector<int> > inliers (new std::vector<int>);
	pcl::PointIndices::Ptr inliers=boost::make_shared<pcl::PointIndices>();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.03);
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coeff);

	pcl::ExtractIndices<pcl::PointXYZ> extracter;
	extracter.setInputCloud(cloud);
	extracter.setIndices(inliers);
	extracter.setNegative(false);
	extracter.filter(*cloud_p);

	Eigen::Vector4f centroid;
	Eigen::Vector3f centroid2;
	pcl::PointCloud<pcl::PointXYZRGBA> pointcl;
	copyPointCloud(*pcloud, pointcl);

	pcl::compute3DCentroid(*pcloud, centroid);
	centroid2[0] = centroid[0];
	centroid2[1] = centroid[1];
	centroid2[2] = centroid[2];

	kinectPos = centroid2;

	return coeff;
}

void KinectMarker::FitPlane()
{
	// create random sample consensus
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pcloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
	ransac.setDistanceThreshold(0.01);
	ransac.computeModel();
	boost::shared_ptr<std::vector<int> > inliers (new std::vector<int>);
	ransac.getInliers(*inliers);

	// model coefficients
	Eigen::VectorXf coeff;
	ransac.getModelCoefficients(coeff);

	// project the points onto a plane
	Eigen::Vector4f centroid;
	Eigen::Vector3f centroid2;
	pcl::PointCloud<pcl::PointXYZRGBA> pointcl;
	copyPointCloud(*pcloud, pointcl);

	pcl::compute3DCentroid(*pcloud, centroid);
	centroid2[0] = centroid[0];
	centroid2[1] = centroid[1];
	centroid2[2] = centroid[2];

	//Eigen::Vector3f centroid = Eigen::Vector3f (pcloud.getCentroid());
	Eigen::Vector4f model2   = Eigen::Vector4f ( coeff[0],
						   coeff[1],
						   coeff[2],
						   coeff[3]);

	// viewpoint
	Eigen::Vector3f vp(0.0, 0.0, 0.0);
	pcl::PointCloud<pcl::PointXYZRGBA> projected = projectToPlaneFromViewpoint (pointcl, model2, centroid2, vp);


	std::ofstream file("test.txt");
	if(file.is_open())
	{
		//std::cerr << "Cloud after projection: " << std::endl;
		for (size_t i = 0; i < projected.points.size(); ++i)
			file << "    " << projected.points[i].x << " "
				<< projected.points[i].y << " "
				<< projected.points[i].z << std::endl;
	}
	coeff.normalized();

	// not pointed in direction of camera flip it
	if(coeff[2]>0)
	{
		coeff = -coeff;
	}
	normal.x =  coeff[0];
	normal.y =  coeff[1];
	normal.z =  coeff[2];

	//if(coeff[0] == coeff[0])
	std::cout << "n: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << std::endl;

	// store the 3d positions of the marker
	for (size_t i = projected.points.size()-4; i < projected.points.size (); ++i)
	{
		//projected.points[i].z = 0;
		markerCorners3D.push_back(projected.points[i]);
	}
	kinectPos = centroid2;

	file.close();
}

cv::Mat KinectMarker::PointToMat(pcl::PointXYZRGBA p)
{
	cv::Mat m(3,1,CV_64F);
	m.at<double>(0,0) = p.x;
	m.at<double>(1,0) = p.y;
	m.at<double>(2,0) = p.z;
	return m;
}

cv::Mat KinectMarker::PointToMat(pcl::PointXYZ p)
{
	cv::Mat m(3,1,CV_64F);
	m.at<double>(0,0) = p.x;
	m.at<double>(1,0) = p.y;
	m.at<double>(2,0) = p.z;
	return m;
}

void KinectMarker::DrawCoordinateSys()
{
	osg::Geode* xAxis = new osg::Geode();
	osg::ShapeDrawable* xShape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(2.5f,0.0f,0.0f),5.0f,1.0f,1.0f));
	xShape->setColor(osg::Vec4(1,0,0,0));
	xAxis->addDrawable(xShape);

	osg::Geode* yAxis = new osg::Geode();
	osg::ShapeDrawable* yShape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,2.5f,0.0f),1.0f,5.0f,1.0f));
	yShape->setColor(osg::Vec4(0,1,0,0));
	yAxis->addDrawable(yShape);

	osg::Geode* zAxis = new osg::Geode();
	osg::ShapeDrawable* zShape = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,2.5f),1.0f,1.0f,5.0f));
	zShape->setColor(osg::Vec4(0,0,1,0));
	zAxis->addDrawable(zShape);

	transf->addChild(xAxis);
	transf->addChild(yAxis);
	transf->addChild(zAxis);
}
