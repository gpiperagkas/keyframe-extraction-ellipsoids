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


#ifndef IMAGERGB_H
#define IMAGERGB_H
#include <iostream>
#include <string>
#include <vector>
#include <exception>
#include <math.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/SVD>
#include <unsupported/Eigen/MatrixFunctions>

class imagergb
{
private:
    std::string path;
    int i,j,k,l, sx,sy,sc, im;
    int *r; //red color of image
    int *g; // green # # #
    int *b; //blue # # #
    int *lbls;//matrix of labels of segments
    int *rr;
    int *gg;
    int *bb;
    int *listhorizontal=NULL; //list of horizontal segment sequence
    int *listvertical=NULL; //list of vertical segment sequence
    int *horlines=NULL; //nr of lines for each segment - horizontal scan
    int *vertlines=NULL; //nr of lines for each segment - vertical scan
    int *ct=NULL; //touching hint counter
    int *co=NULL; //overlapping hint counter
    cv::Mat binary;

//    std::vector < std::vector<cv::Point2i > > blobs;
    Eigen::Matrix3d Q1;
    Eigen::Matrix3d Q2;
    Eigen::Vector3d C1;
    Eigen::Vector3d C2;
    Eigen::Vector3d xstarrr; //margin between xstar of Q2 to center of Q1, c1
    Eigen::VectorXd xstarr1;
    Eigen::VectorXd xstarr2;
    double marr;
    double truemarg;
    double semiaxon;

public:
    int feature; //simple=0, ellipsoids=1
    double *normct=NULL; // normalized hints counter touching
    double *normco=NULL; // normalized hints counter overlapping
    double *normcel=NULL; //normalized hints for ellipsoids touching
    int *psize; //size of segment in pixels, we also use it for presence or absence of segment
    std::vector<int> parstate;
    std::vector<int> finstate;
    imagergb(const std::string pathim[],int imn);
    imagergb(cv::Mat frame, int imn, cv::Mat labels);
    imagergb(cv::Mat frame, int imn, cv::Mat labels, Eigen::Matrix3d q1,
             Eigen::Matrix3d q2, Eigen::Vector3d c1, Eigen::Vector3d c2);
    void scanSegmented();
    void ElliMargin(int imar);
    void ElliAxon();
    void ElliAngles(int mode, std::vector<int> finst);
    void scan();
    void setSegRelations();
    void calcNormHints();
    void printhints();
    ~imagergb();


};

#endif // IMAGERGB_H
