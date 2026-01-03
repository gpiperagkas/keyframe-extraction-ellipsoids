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


#include "ellipsoid.h"

using namespace std;
using namespace Eigen;

#define R 0.99
#define E1 0.95
#define E2 0.95
#define THR 0.95
#define N 3 
#define PI 3.14159265358979323846
#define ST 0.1
#define MAXITER 40
#define NTHREADS 4
//#define LEAFSIZE 0.003


ellipsoid::ellipsoid(float leafsize, std::string file)
{
    incloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    cloudfin = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    ellicloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ> ());
    cloud = pcl::PCLPointCloud2::Ptr (new pcl::PCLPointCloud2);
    cloudfil = pcl::PCLPointCloud2::Ptr (new pcl::PCLPointCloud2);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file, *incloud) == -1) //* load the file
		{
            PCL_ERROR ("Couldn't read file \n");
		}
    pcl::toPCLPointCloud2 (*incloud, *cloud);
    pcl::VoxelGrid< pcl::PCLPointCloud2  > sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (leafsize, leafsize, leafsize);
    sor.filter (*cloudfil);

    pcl::fromPCLPointCloud2 (*cloudfil, *cloudfin);
	//set number of points included
    m=cloudfin->points.size();
	u.resize(m);
	U.resize(m,m);
    e = VectorXd::Ones(m);
	t.resize(m);
	T.resize(m,m);
    A.resize(N,m);
    ai.resize(N);
    for (size_t t=0;t<cloudfin->points.size();t++)
	{
        A(0,t)= cloudfin->points[t].x;
        A(1,t)= cloudfin->points[t].y;
        A(2,t)= cloudfin->points[t].z;
	}
	jach.resize(m,m);
	Du.resize(m);
	Dt.resize(m);
	r1.resize(m);
	r2.resize(m);
	hu.resize(m);
}


void ellipsoid::DRNcore()
{
	VectorXd chu;
	VectorXd cht;
    bool stopNow=false;
    double val;
    bool check=true;
    cnt=0;


    cout <<" m= "<< m <<endl;

    //initialize
	this->DRNinit();

    while(!this->DRNstoppingCriteria())
	{
		//iterations here!
        val = (u.transpose()*t);
        theta = 4*(1/double(10*m))*val;
		
		// get directions
		this->DRNgetDirection();
		

        check=true;
		//step size
//        beta =0.01;
//        chu = u+(beta*Du);
//        cht = t+(beta*Dt);
//        for (i=0;i<m;i++)
//            if ((chu(i)<0)||(cht(i)<0))
//            {
//                check=false;
//                beta = beta/10;
//                break;
//            }

//        while ((check)&&(beta<1))
//        {
//            beta = beta + ST;
//            chu = u+(beta*Du);
//            cht = t+(beta*Dt);
//            for (i=0;i<m;i++)
//                if ((chu(i)<0)||(cht(i)<0))
//                {
//                    check=false;
//                    beta = beta - ST;
//                }


//        }

//        if (R*beta<1)
//            beta=R*beta;
//        else
//            beta=1;
        //beta=0.7;

        stopNow=false;

        beta=0.1;
        //Step!
        u = u + (beta*Du);
        t = t + (beta*Dt);
        for (i=0;i<m;i++)
            if ((u(i)<0)||(t(i)<0))
            {
                u = u - (beta*Du);
                t = t - (beta*Dt);
                stopNow = true;
                break;
            }

        if (stopNow)
        {
            beta=0.01;
            u = u + (beta*Du);
            t = t + (beta*Dt);
        }

        U= u.asDiagonal();

        //CREATE THREADPOOL FOR PARALLEL COMPUTATION OF H
        /*
         * Create an asio::io_service and a thread_group (through pool in essence)
         */


        boost::asio::io_service ioService;

        // Add work to ioService.
            for (i=0;i<m;i++)
            {
                for (j=0;j<N;j++)
                    ai(j)=A(j,i);
                ioService.post(boost::bind( &ellipsoid::threaded_h, this, i, ai));
            }


        // Now that the ioService has work, use a pool of threads to service it.
        boost::thread_group threadpool;
        for (unsigned int il = 0; il < NTHREADS; il++)
        {
          threadpool.create_thread(boost::bind(
              &boost::asio::io_service::run, &ioService));
        }

        // Once all work has been completed (thread_func invoked ls1 times), the
        // threads in the threadpool will be completed and can be joined.
        threadpool.join_all();



        //compute hu here
        //		for (i=0;i<m;i++)
        //		{
        //            for (j=0;j<N;j++)
        //				ai(j)=A(j,i);
        //			hu(i)= ((ai - (1/(e.transpose()*u))*A*u).transpose())*
        //					(2*((A*U*A.transpose())- (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose())).inverse()
        //					*(ai - (1/(e.transpose()*u))*A*u);
        //		}
        cnt++;
        cout << "Iteration: "<<cnt <<endl;
	}
	
	this->setResults(); //return u Q c
	
}


void ellipsoid::DRNgetDirection()
{
	U= u.asDiagonal();
	T= t.asDiagonal();

	//compute M^-2
	M_2= 2*(A*U*A.transpose() - (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose());
	//compute M
    M=((2*(A*U*A.transpose() - (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose())).sqrt()).inverse();
	//compute Sigma
	Sigma = (A- (1/(e.transpose()*u))*A*u*e.transpose()).transpose()*(M*M)*(A- (1/(e.transpose()*u))*A*u*e.transpose());
	//compute Hadamard product of Sigma
	Hadprod.resize(Sigma.rows(),Sigma.cols());
	for (i=0;i<Sigma.rows();i++)
		for (j=0;j<Sigma.cols();j++)
			Hadprod(i,j)=Sigma(i,j)*Sigma(i,j);
	
	//compute Jacobian
	jach = -2*(((1/(e.transpose()*u))*Sigma)*Hadprod);
	
	//get the directions
	r1 = e - t - hu;
	r2 = theta*e -U*t;
	Du = (jach - U.inverse()*T).inverse()*(r1-(U.inverse()*r2));
	Dt = (U.inverse()*r2)-(U.inverse()*T*Du);

}


void ellipsoid::threaded_h(int ii, Eigen::VectorXd aii)
{

    hu(ii)= ((aii - (1/(e.transpose()*u))*A*u).transpose())*
            (2*((A*U*A.transpose())- (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose())).inverse()
            *(aii - (1/(e.transpose()*u))*A*u);

}


void ellipsoid::DRNinit()
{
	// DRN algorithm initialization
	bool positivity=true;
	double alpha;
    u=(N/double(2*m))*e;
	U= u.asDiagonal();

    boost::asio::io_service ioService;

    // Add work to ioService.
        for (i=0;i<m;i++)
        {
            for (j=0;j<N;j++)
                ai(j)=A(j,i);
            ioService.post(boost::bind( &ellipsoid::threaded_h, this, i, ai));
        }


    // Now that the ioService has work, use a pool of threads to service it.
    boost::thread_group threadpool;
    for (unsigned int il = 0; il < NTHREADS; il++)
    {
      threadpool.create_thread(boost::bind(
          &boost::asio::io_service::run, &ioService));
    }

    // Once all work has been completed (thread_func invoked ls1 times), the
    // threads in the threadpool will be completed and can be joined.
    threadpool.join_all();
    //multi-threading end

    for (i=0;i<m;i++)
    {
        if (hu(i)>THR*e(i))
            positivity=false;
    }

    //    //compute hu
    //	for (i=0;i<m;i++)
    //	{
    //        for (j=0;j<N;j++)
    //			ai(j)=A(j,i);
    //		hu(i)= ((ai - (1/(e.transpose()*u))*A*u).transpose())*
    //				(2*((A*U*A.transpose())- (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose())).inverse()
    //				*(ai - (1/(e.transpose()*u))*A*u);
    //		if (hu(i)>THR*e(i))
    //			positivity=false;

    //	}

	if (positivity)
		t= e- hu;
	else
	{
		//rescale to ensure feasibility
		alpha= hu(0);
		for (i=1;i<m;i++)
			if (hu(i)>alpha)
				alpha=hu(i);
		alpha=alpha/THR;

		u=alpha*u;
		U= u.asDiagonal();

        boost::asio::io_service ioService;

        // Add work to ioService.
            for (i=0;i<m;i++)
            {
                for (j=0;j<N;j++)
                    ai(j)=A(j,i);
                ioService.post(boost::bind( &ellipsoid::threaded_h, this, i, ai));
            }


        // Now that the ioService has work, use a pool of threads to service it.
        boost::thread_group threadpool;
        for (unsigned int il = 0; il < NTHREADS; il++)
        {
          threadpool.create_thread(boost::bind(
              &boost::asio::io_service::run, &ioService));
        }

        // Once all work has been completed (thread_func invoked ls1 times), the
        // threads in the threadpool will be completed and can be joined.
        threadpool.join_all();

        //multi-threading end

        //compute hu
        //		for (i=0;i<m;i++)
        //		{
        //            for (j=0;j<N;j++)
        //				ai(j)=A(j,i);
        //			hu(i)= ((ai - (1/(e.transpose()*u))*A*u).transpose())*
        //					(2*((A*U*A.transpose())- (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose())).inverse()
        //					*(ai - (1/(e.transpose()*u))*A*u);
        //		}

		t= e - hu;
		
	}
	
	
	

}


void ellipsoid::setResults()
{
	//compute Q, c
	complex<double> ll;
	Vector3cd ev;

	
	U= u.asDiagonal();
    M=((2*(A*U*A.transpose() - (1/(e.transpose()*u))*A*u*u.transpose()*A.transpose())).sqrt()).inverse();
	Q = M*M;
	z = (1/(e.transpose()*u))*M*A*u;
	c= M.inverse()*z;
	
	EigenSolver<MatrixXd> es(Q);
	for (i=0;i<3;i++)
	{
		axesLength.push_back(0);
		ll=es.eigenvalues()[i];
		axesLength.at(i) = 1/std::sqrt(std::abs(ll));

	}
	axes = es.eigenvectors();
	ev(0)= axes.col(0)[0];
	ev(1)= axes.col(0)[1];
	ev(2)= axes.col(0)[2];
//    cout << atan (ev(1).real()/ev(0).real())*180/PI <<endl;
//    cout << atan (ev(2).real()/ev(0).real())*180/PI <<endl;
//    cout << atan (ev(2).real()/ev(1).real())*180/PI <<endl;
    //yaw = atan (ev(2).real()/ev(1).real())*180/PI;
    yaw = atan (ev(1).real()/ev(0).real())*180/PI;
	ev(0)= axes.col(1)[0];
	ev(1)= axes.col(1)[1];
	ev(2)= axes.col(1)[2];
//    cout << atan (ev(1).real()/ev(0).real())*180/PI <<endl;
//    cout << atan (ev(2).real()/ev(0).real())*180/PI <<endl;
//    cout << atan (ev(2).real()/ev(1).real())*180/PI <<endl;
    //roll = atan (ev(2).real()/ev(0).real())*180/PI;
    roll = atan (ev(1).real()/ev(2).real())*180/PI;
	ev(0)= axes.col(2)[0];
	ev(1)= axes.col(2)[1];
	ev(2)= axes.col(2)[2];
//    cout << atan (ev(1).real()/ev(0).real())*180/PI <<endl;
//    cout << atan (ev(2).real()/ev(0).real())*180/PI <<endl;
//    cout << atan (ev(2).real()/ev(1).real())*180/PI <<endl;
    pitch = atan (ev(2).real()/ev(0).real())*180/PI;
    //pitch = atan (ev(1).real()/ev(0).real())*180/PI;

}


bool ellipsoid::DRNstoppingCriteria()
{
	//check stopping criteria
    double val;
    double vall;
    val = std::sqrt((e-hu-t).transpose()*(e-hu-t));
    vall = u.transpose()*t;
    cout << "val 1= "<< val << " and val 2="<< vall <<endl;

    if (((val<=E1)&&(vall<=E2))||(cnt>=MAXITER))
    {
        cout << "val 1= "<< val << " and val 2="<< vall <<endl;
        cout<< "OK!"<<endl;
		return true;

    }
    else
    {
       // cout << "val 1= "<< val << " and val 2="<< vall <<endl;
		return false;
    }
			
}



void ellipsoid::rotateQ(std::vector<float> ypr)
{
    //g := rotx
    //b := roty
    //a := rotz
    Matrix3d Rx;
    Matrix3d Ry;
    Matrix3d Rz;
    //MatrixXd Rot;
    double a,b,g;

    g=ypr.at(0);
    b=ypr.at(1);
    a=ypr.at(2);

    Rx(0,0) = 1; Rx(0,1) = 0; Rx(0,2) = 0;
    Rx(1,0) = 0; Rx(1,1) = cos(g); Rx(1,2) = -sin(g);
    Rx(2,0) = 0; Rx(2,1) = sin(g); Rx(2,2) = cos(g);

    Ry(0,0) = cos(b); Ry(0,1) = 0; Ry(0,2) = sin(b);
    Ry(1,0) = 0; Ry(1,1) = 1; Ry(1,2) = 0;
    Ry(2,0) = -sin(b); Ry(2,1) = 0; Ry(2,2) = cos(b);

    Rz(0,0) = cos(a); Rz(0,1) = -sin(a); Rz(0,2) = 0;
    Rz(1,0) = sin(a); Rz(1,1) = cos(a); Rz(1,2) = 0;
    Rz(2,0) = 0; Rz(2,1) = 0; Rz(2,2) = 1;

    //Rot = Rz*Ry*Rx;
    Rot=Rx*Ry*Rz;
    //RotQ = Rot.transpose()*Q*Rot;
    RotQ = Rot*Q*Rot.transpose();
}


void ellipsoid::moveC(std::vector<float> center)
{
       cnew(0) = c(0) + center.at(0);
       cnew(1) = c(1) + center.at(1);
       cnew(2) = c(2) + center.at(2);

       cout << "center =" << cnew(0) << " "<<cnew(1)<< " "<<cnew(2) <<endl;
}

void ellipsoid::visualize()
{
    pcl::PointXYZ point;
    //produce a point cloud from ellipsoid
    uu=0;
    while (uu<2*PI)
    {
        vv=0;
        while (vv<PI)
        {
            point.x = axesLength.at(0)*std::cos(uu)*std::sin(vv) + c(0);
            point.y = axesLength.at(1)*std::sin(uu)*std::sin(vv) + c(1);
            point.z = axesLength.at(2)*std::cos(vv) + c(2);
            ellicloud->points.push_back(point);
            vv = vv + 2*PI/30;
        }
        uu = uu + 2*PI/50;
    }

//    Eigen::Affine3f transformx = Eigen::Affine3f::Identity();
//    transformx.rotate (Eigen::AngleAxisf ( yaw*PI/180 , Eigen::Vector3f::UnitX ()));
//    Eigen::Affine3f transformy = Eigen::Affine3f::Identity();
//    transformy.rotate (Eigen::AngleAxisf ( pitch*PI/180, Eigen::Vector3f::UnitY ()));
//    Eigen::Affine3f transformz = Eigen::Affine3f::Identity();
//    transformz.rotate (Eigen::AngleAxisf ( roll*PI/180, Eigen::Vector3f::UnitZ ()));

//    // Executing the transformation
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedx (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedy (new pcl::PointCloud<pcl::PointXYZ> ());
//    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedz (new pcl::PointCloud<pcl::PointXYZ> ());

//    // You can either apply transform_1 or transform_2; they are the same
//    pcl::transformPointCloud (*ellicloud, *transformedx, transformx);
//    pcl::transformPointCloud (*transformedx, *transformedy, transformy);
//    pcl::transformPointCloud (*transformedy, *transformedz, transformz);



    pcl::visualization::CloudViewer viewer ("Ellipsoid Viewer");
    viewer.showCloud (cloudfin, "model");
    viewer.showCloud (ellicloud, "ellipsoid");

    while (!viewer.wasStopped ())
    {
    }
}



