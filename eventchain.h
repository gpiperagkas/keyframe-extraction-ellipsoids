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



#ifndef EVENTCHAIN_H
#define EVENTCHAIN_H
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <fstream>
#include <float.h>
#include <string>
#include <eigen3/Eigen/Dense>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include <boost/thread/thread.hpp>
#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include <boost/asio/io_service.hpp>

#include "imagergb.h"
#include "tracker.h"
#include "xmlparser.h"
#include "xmlhandparser.h"
#include "ellipsoid.h"


class eventchain
{
private:
    int i,j,k;
    int xroi, yroi, wroi, zroi;
    int sc, nel;
//    std::string rgbfname;
    std::string maskfname;
//    std::string tosave;
    //std::string vdfname="putting_01.mpg";
    int **osec;
    int **dsec1;
    int **dsec2;
    int *keyframeflag;
    std::vector<imagergb*> sequence;
    double thresholdd;
    double thresholdo;
    double **eigenval; 
    std::vector<int> x;
    std::vector<int> y;
    std::vector<int> r1;
    std::vector<int> r2;
    std::vector< std::vector<int> > ksi1;
    std::vector< std::vector<int> > ksi2;
    std::vector<ellipsoid*> objects;
    int assembly_type;

public:
    std::vector<int> finst;
    int n;
    int nini;
    eventchain(const std::string path[], int nn );
    eventchain();
    void printtoFile(cv::Mat labels);
    void buildInitChain();
    void calcEigenValues(int nim);
    void extractKeyframes();
    void derivativeSEC();
    void constructCSEC();
    void visualizeKeyframes();
    bool loadFromFile(const std::string& full_filename, float *coeff, int num_parameters);
    void extractVideo();
    void printOSEC();
    void printCSEC();
    ~eventchain();
};

#endif // EVENTCHAIN_H
