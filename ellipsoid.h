/*
    Keyframe extraction with semantic graphs in assembly processes.
    Copyright (C) 2016-2025  Grigorios Piperagkas

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/



#ifndef ELLIPSOID_H
#define ELLIPSOID_H
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <stdlib.h>
#include <time.h> 
#include <exception>
#include <fstream>
#include <float.h>
#include <string>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio/io_service.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

class ellipsoid
{
	
private:
	int i,j;
    int m, cnt;
    float uu,vv;
	double theta;
	double beta;
    pcl::PointCloud<pcl::PointXYZ>::Ptr incloud, cloudfin, ellicloud;
    //pcl::PointCloud<pcl::PointXYZ> incloud;
    pcl::PCLPointCloud2::Ptr cloud;
    pcl::PCLPointCloud2::Ptr cloudfil;
	Eigen::Matrix3d M;
	Eigen::Vector3d z;
	Eigen::VectorXd u;
	Eigen::MatrixXd U;//diag(u)
	Eigen::VectorXd e;//ones
	Eigen::VectorXd t;
	Eigen::MatrixXd T;//diag(t)
	Eigen::MatrixXd A; // nxm matrix with columns a1, a2 ,.... am : the point cloud
    Eigen::VectorXd ai;
	Eigen::Matrix3d M_2;
	Eigen::MatrixXd jach; //mxm size
	Eigen::MatrixXd Sigma;
	Eigen::VectorXd Du; //directions
	Eigen::VectorXd Dt; //directions
	Eigen::VectorXd r1;
	Eigen::VectorXd r2;
	Eigen::VectorXd hu;
	Eigen::MatrixXd Hadprod; //hadamard product
	
public:
    Eigen::MatrixXd Rot;
	Eigen::Matrix3d Q; //ellipsoid matrix
    Eigen::Matrix3d RotQ; //rotated ellipsoid matrix
    Eigen::Vector3d cnew; //moved the center by an offset
	Eigen::Vector3d c; //ellipsoid vector for center
	std::vector<double> axesLength; //get the abs value
	Eigen::Matrix3cd axes;
	std::vector<double> axesAngles;
	double roll;
	double pitch;
	double yaw;
    ellipsoid(float leafsize, std::string file);
	void DRNcore();
	void DRNgetDirection();
	void DRNinit();
    void threaded_h(int ii, Eigen::VectorXd ai);
	void setResults();
	bool DRNstoppingCriteria();
    void rotateQ(std::vector<float> ypr);
    void moveC(std::vector<float> center);
    void visualize();

	~ellipsoid();
	
};



#endif
