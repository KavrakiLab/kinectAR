#include "kinectMarker.h"
#include <math.h> 

//using namespace cv;

int sign(double x)
{
	return (x > 0)? 1 : -1;
}

KinectMarker::KinectMarker()
{
}

void KinectMarker::AddToSceneGraph(osg::Group *root)
{
	useOSG = true;
	transf = new osg::PositionAttitudeTransform();

	// zuerst gruppenknoten fuer auto
	root->addChild(transf);
	
	
	// dann fgen wir eine box als geometrie ein
	geometry = new osg::Geode;
	shape    = new osg::ShapeDrawable(new osg::Box(osg::Vec3(0.0f,0.0f,0.0f),2.8f,2.8f,0.2f));
	shape->setColor(osg::Vec4(1,0,0,0));
	geometry->addDrawable(shape);
	
	transf->addChild(geometry);
}

void KinectMarker::Update(alvar::MarkerData* newData)
{
	markerPointsImg.clear();
	markerCornersImg.clear();
	markerCorners3D.clear();
	
	// set the pointer to incoming data
	alvarData = newData;
	
	// the current alvar pose
	alvarPose = newData->pose;

	markerCornersImg      = newData->marker_corners_img;
	markerCornersCentered = newData->marker_corners;
	markerPointsImg       = newData->ros_marker_points_img;

	// sort corner points
	SortCornerPoints();
	
	if(useOSG)
	{
		// set position
		osg::Vec3 pos;
		for(int i = 0; i < 3; i++)
			pos[i] = kinectPos[i];
			
		// set quaternion
		osg::Quat quat;
		quat[0] = kinectQuat[1];
		quat[1] = kinectQuat[2];
		quat[2] = kinectQuat[3];
		quat[3] = kinectQuat[0];
		
		double tmp[4];
		CvMat q = cvMat(4, 1, CV_64F, tmp);
		alvarPose.GetQuaternion(&q);
		double* alvar_quat = (double*)q.data.ptr;
		quat[0] = alvar_quat[1];
		quat[1] = alvar_quat[2];
		quat[2] = alvar_quat[3];
		quat[3] = alvar_quat[0];
		
		if(alvarData->GetError() < 0.02)
		{
			transf->setPosition( pos -osg::Vec3(0,0,0));
			std::cout << "Error: " << alvarData->GetError() << std::endl;
			transf->setAttitude( quat );
		}
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
		0.0,	fy_v, 	cy_v,	0.0,
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
	int counter = 0;
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
		if(w > 0 && h > 0) distance = depthMap.at<unsigned short>(h,w)/10.0;
		else distance = 0;

		// check if within range of sweet spot
		if(distance > 0 && distance < 300.0)
		{
			cv::Mat pointDepth3D  = depthIntrinsicsInverse * (tmpPoint * (distance));
			
			pcl::PointXYZ point3D;
			point3D.x = pointDepth3D.at<double>(0,0); 
			point3D.y = pointDepth3D.at<double>(1,0); 
			point3D.z = pointDepth3D.at<double>(2,0); 
			
			cloud->points[counter].x = point3D.x;
			cloud->points[counter].y = point3D.y;
			cloud->points[counter].z = point3D.z;
			
			cv::Mat tmp = PointToMat(point3D);
			sumVec = sumVec+tmp;

			for(int t = 0; t < 0; t++)
				if(!std::isfinite(tmp.at<double>(t,0)))
				{
					std::cout << "Found NAN!" << std::endl;
					continue;
					
				}
				
			counter++;
		}
		else
		{
			distance = alvarPose.translation[2];
			cv::Mat pointDepth3D  = depthIntrinsicsInverse * (tmpPoint * (distance));
			
			pcl::PointXYZ point3D;
			point3D.x = pointDepth3D.at<double>(0,0); 
			point3D.y = pointDepth3D.at<double>(1,0); 
			point3D.z = pointDepth3D.at<double>(2,0); 
			
			cloud->points[counter].x = point3D.x;
			cloud->points[counter].y = point3D.y;
			cloud->points[counter].z = point3D.z;
			
			cv::Mat tmp = PointToMat(point3D);
			sumVec = sumVec+tmp;
			
			for(int t = 0; t < 0; t++)
				if(!std::isfinite(tmp.at<double>(t,0)))
				{
					std::cout << "Found NAN!" << std::endl;
					continue;
				}
				
			counter++;
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
}

void KinectMarker::GetNormalVector()
{
	FitPlane();
	
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointcl(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr out(new pcl::PointCloud<pcl::PointXYZRGBA>);
	copyPointCloud(*pcloud, *pointcl);
	copyPointCloud(*pcloud, *out);
	
	// project point cloud with PCA
	/*pcl::PCA<pcl::PointXYZRGBA> pcaC;
	pcaC.setInputCloud(pointcl);
	pcaC.project(*pointcl, *out);
	Eigen::Matrix3f eigenVec = pcaC.getEigenVectors();
	
	std::cout << "------------------" << std::endl;
	for(int i =0; i < 3; i++)
	{
		for(int j =0; j < 3; j++)
		{
			std::cout << eigenVec(i, j) << " ";
		}
		std::cout << std::endl;
	}
	std::cout << "------------------" << std::endl;
	
	std::ofstream file("test4.txt");
	if(file.is_open())
	{
		//std::cerr << "Cloud after projection: " << std::endl;
		for (size_t i = 0; i < out->points.size(); ++i)
			file << "    " << out->points[i].x << " " 
				<< out->points[i].y << " " 
				<< out->points[i].z << std::endl;
	}
	file.close();*/
	
	
	// now we have the normal -> let's calculate the vectors from each corner
	cv::Mat c1 = PointToMat(markerCorners3D[0]);
	cv::Mat c2 = PointToMat(markerCorners3D[1]);
	cv::Mat c3 = PointToMat(markerCorners3D[2]);
	cv::Mat c4 = PointToMat(markerCorners3D[3]);
	
	// calculate new center
	cv::Mat zDim = PointToMat(normal);
	cv::Mat newCenter = (c1+c2+c3+c4)/4.0;
	
	// flip zDim
	for(int i = 0; i < 3; i++)
		zDim.at<double>(i,0) = -zDim.at<double>(i,0);
	
	// vectors in the plane
	cv::Mat xDim = (c3-c2);
	xDim = -xDim / norm(xDim);
	std::cout << "Corner Points: " << std::endl;
	std::cout << c1.at<double>(0,0)<< " " << c1.at<double>(1,0) << " " <<c1.at<double>(2,0)<< std::endl;
	std::cout << c2.at<double>(0,0)<< " " << c2.at<double>(1,0) << " " <<c2.at<double>(2,0)<< std::endl;
	std::cout << c3.at<double>(0,0)<< " " << c3.at<double>(1,0) << " " <<c3.at<double>(2,0)<< std::endl;
	std::cout << c4.at<double>(0,0)<< " " << c4.at<double>(1,0) << " " <<c4.at<double>(2,0)<< std::endl;
	std::cout << "------------" << std::endl;
	
	
	// get x dim from PCA
	/*cv::Mat xDim(3,1,CV_64F); 
	for(int i = 0; i < 3; i++)
		xDim.at<double>(i,0) = eigenVec(0,i);*/
	
	// print out the vector in the plane 
	//std::cout << "xDim: " << xDim.at<double>(0,0) << " " << xDim.at<double>(1,0) << " " << xDim.at<double>(2,0) << std::endl;
	cv::Mat yDim = xDim.cross(zDim);
	
	//std::cout << "yDim: " << yDim.at<double>(0,0) << " " << yDim.at<double>(1,0) << " " << yDim.at<double>(2,0) << std::endl;
	//std::cout << "Perpendicularity: " << xDim.dot(zDim) << " " << yDim.dot(xDim) << std::endl;
	
	// create rotation matrix
	double  m[3][3];
	for(int i= 0; i <3; i++)
		m[0][i] = yDim.at<double>(i,0);
	
	for(int i= 0; i <3; i++)
		m[1][i] = xDim.at<double>(i,0);
	
	for(int i= 0; i <3; i++)
		m[2][i] = zDim.at<double>(i,0);

	
	
	// get the current pose as a quaternion
	ToQuaternion(m, kinectQuat);
	
	std::ofstream file2("test2.txt");
	file2.is_open();
	
	printf("kin:\n");
	aa_dump_mat(stdout, m[0], 3,3 );
	//assert(aa_tf_isrotmat(m[0]));	
	
	file2 << kinectPos[0] + 5.0*zDim.at<double>(0,0)<< " " << kinectPos[1] + 5.0*zDim.at<double>(1,0) << " " << kinectPos[2] + 5.0*zDim.at<double>(2,0)<< std::endl;
	file2 << kinectPos[0] + 5.0*yDim.at<double>(0,0)<< " " << kinectPos[1] + 5.0*yDim.at<double>(1,0) << " " << kinectPos[2] + 5.0*yDim.at<double>(2,0)<< std::endl;	
	file2 << kinectPos[0] + 5.0*xDim.at<double>(0,0)<< " " << kinectPos[1] + 5.0*xDim.at<double>(1,0) << " " << kinectPos[2] + 5.0*xDim.at<double>(2,0)<< std::endl;
	file2.close();
}

void KinectMarker::PrintKinectPose()
{
	//std::cout << "k: " << kinectPos[0] << " " << kinectPos[1] << " " << kinectPos[2] << std::endl;
	//std::cout << "k: " << kinectQuat[0] << " " << kinectQuat[1] << " " << kinectQuat[2] << " " << kinectQuat[3] << std::endl;
}

void KinectMarker::PrintAlvarPose()
{
	//std::cout << "a: " << alvarPose.translation[0] << " " << alvarPose.translation[1] << " " << alvarPose.translation[2] << std::endl;
	double tmp[4];
	CvMat q = cvMat(4, 1, CV_64F, tmp);
	alvarPose.GetQuaternion(&q);
	double* alvar_quat = (double*)q.data.ptr;
	std::cout << "a: " << alvar_quat[0] << " " << alvar_quat[1] << " " << alvar_quat[2] << " " << alvar_quat[3] << std::endl;
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

void KinectMarker::FitPlane()
{
	// create random sample consensus
	pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(pcloud));
	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model);
	ransac.setDistanceThreshold(.05);
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
	if(coeff[2]<0)
	{
		coeff = -coeff;
	}		
	normal.x = coeff[0];
	normal.y = coeff[1];
	normal.z = coeff[2];
	
	//if(coeff[0] == coeff[0])
	//	std::cout << "n: " << coeff[0] << " " << coeff[1] << " " << coeff[2] << std::endl;
	
	// store the 3d positions of the marker
	for (size_t i = projected.points.size()-4; i < projected.points.size (); ++i)
	{
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

/*void KinectMarker::FitPlane()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corners (new pcl::PointCloud<pcl::PointXYZ>);
	
	
	// OLDCODE
	

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_corners (new pcl::PointCloud<pcl::PointXYZ>);
	
	// Create a set of planar coefficients with X=Y=0,Z=1
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	coefficients->values.resize (4);
	coefficients->values[0] = coefficients->values[1] = 1.0;
	coefficients->values[2] = 1.0;
	coefficients->values[3] = 1;

	// Create the filtering object
	pcl::ProjectInliers<pcl::PointXYZ> proj;
	proj.setModelType (pcl::SACMODEL_PLANE);
	proj.setInputCloud (pcloud);
	proj.setModelCoefficients (coefficients);
	proj.filter (*cloud_projected);

	
	pcl::ModelCoefficients::ConstPtr res (new pcl::ModelCoefficients ());
	res = proj.getModelCoefficients();
	
	
	
}*/

