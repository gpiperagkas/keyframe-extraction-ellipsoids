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

#ifndef TRACKER_H
#define TRACKER_H
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>


class tracker
{
private:
    int i,j;
    std::vector<int> pcentrx;
    std::vector<int> pcentry;
    std::vector<int> nx;
    std::vector<int> ny;
    std::vector<int> pnx;
    std::vector<int> pny;

    std::vector<int> nrlabelsp;

    //int *pcentx; //previous frame centroid
   // int *pcenty; //previous frame centroid
 //   int *ndx; //rectangle x dimension nr of pixels
 //   int *ndy; // rectangle y dimension nr of pixels

public:
    std::vector<int> cn;
    std::vector<int> nrlabelsn;
    int ns;
    cv::Mat newmarkers;
    std::vector<int> newcentrx;
    std::vector<int> newcentry;
    //int *centx;//centroidx
    //int *centy; //centroidy
    tracker();
    void initMarkers(cv::Mat &markers);
    double getDist(int l, int ll);
    void findCentroids(cv::Mat markers, cv::Mat pmarkers);
    void findClosestNeighbors(cv::Mat markers, cv::Mat pmarkers);
    ~tracker();

};

#endif // TRACKER_H
