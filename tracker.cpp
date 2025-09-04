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


#include "tracker.h"

using namespace std;
using namespace cv;

tracker::tracker()
{
 //   ns=5;
//    centx = new int[5];
//   centy = new int[5];
 //   pcentx = new int[5];
//    pcenty = new int[5];
//    ndx = new int[5];
 //   ndy = new int[5];

}


void tracker::initMarkers(cv::Mat &markers)
{
    int count;
    int point;
    count =0;
    for (i=0;i<markers.rows; i++)
    {
        for (j=0;j<markers.cols;j++)
        {
            point = markers.at<uchar>(i,j);
            if (point ==3 )
                markers.at<uchar>(i,j) = 2;
        }
    }

}


double tracker::getDist(int l, int ll)
{
    if ((newcentrx.at(l) == 0)&&(newcentry.at(l)==0))
        return 1000;
    else
        return sqrt(((newcentrx.at(l)-pcentrx.at(ll))*(newcentrx.at(l)-pcentrx.at(ll))+ ((newcentry.at(l)-pcentry.at(ll))*(newcentry.at(l)-pcentry.at(ll)))));

/*
    if ((centx[l]==0)&&(centy[l]==0))
        return 10000;
    //else if ((pcentx[ll]==0)||(pcenty[ll]==0))
       // return 1000;
    else
        return sqrt(((centx[l]-pcentx[ll])*(centx[l]-pcentx[ll])) + ((centy[l]-pcenty[ll])*(centy[l]-pcenty[ll])));
*/
}



void tracker::findCentroids(cv::Mat markers, cv::Mat pmarkers)
{

    int point, ppoint;
    int tempx, tempy, tnx,tny;
    int found;
    nrlabelsn.push_back(0);
    nrlabelsp.push_back(0);
    for (i=0;i<markers.rows;i++)
    {
        for (j=0;j<markers.cols;j++)
        {
            point = markers.at<uchar>(i,j);
            found=0;

            for (unsigned int k=0;k<nrlabelsn.size();k++)
                if (point > nrlabelsn.at(k))
                {

                    for (unsigned int oo=0;oo<nrlabelsn.size();oo++)
                        if (point==nrlabelsn.at(oo))
                            found=1;

                    if (found==0)
                            nrlabelsn.push_back(point);
                }

            ppoint = pmarkers.at<uchar>(i,j);
            found =0;

            for (unsigned int k=0;k<nrlabelsp.size();k++)
                if (ppoint > nrlabelsp.at(k))
                {
                    for (unsigned int oo=0;oo<nrlabelsp.size();oo++)
                        if (ppoint==nrlabelsp.at(oo))
                            found=1;
                    if (found==0)
                            nrlabelsp.push_back(ppoint);
                }
        }
    }

//    cout << "NRLABELSN:"<<endl;
//    for (unsigned int nri = 0;nri<nrlabelsn.size();nri++)
//        cout << nrlabelsn.at(nri) << " ";
//    cout << endl;

//    cout << "NRLABELSP:"<<endl;
//    for (unsigned int nri=0;nri<nrlabelsp.size();nri++)
//        cout <<nrlabelsp.at(nri) << " ";
//    cout <<endl;

    for (unsigned int oi=0;oi<nrlabelsn.size();oi++)
    {
        newcentrx.push_back(0);
        newcentry.push_back(0);
        nx.push_back(0);
        ny.push_back(0);
    }
    for (unsigned int oi=0;oi<nrlabelsp.size();oi++)
    {
        pcentrx.push_back(0);
        pcentry.push_back(0);
        pnx.push_back(0);
        pny.push_back(0);
    }

    for (i=0;i<markers.rows;i++)
    {
        for (j=0;j<markers.cols;j++)
        {
            point = markers.at<uchar>(i, j);
            ppoint = pmarkers.at<uchar>(i, j);

            for (unsigned int oi=0;oi<newcentrx.size();oi++)
            {
                if (point == nrlabelsn.at(oi))
                {
                    newcentrx.at(oi)= newcentrx.at(oi)+j;
                    newcentry.at(oi)= newcentry.at(oi)+i;
                    nx.at(oi)++;
                    ny.at(oi)++;
                }
            }

            for (unsigned int oi=0;oi<pcentrx.size();oi++)
            {
                if (ppoint == nrlabelsp.at(oi))
                {
                    pcentrx.at(oi) = pcentrx.at(oi) +j;
                    pcentry.at(oi) = pcentry.at(oi) +i;
                    pnx.at(oi)++;
                    pny.at(oi)++;

                }
            }
        }
    }

    for (unsigned int oi=0;oi<newcentrx.size();oi++)
    {
        if (nx.at(oi)>0)
        {
            newcentrx.at(oi) = newcentrx.at(oi)/nx.at(oi);
            newcentry.at(oi) = newcentry.at(oi)/ny.at(oi);
        }
            else
            {
                newcentrx.at(oi)=0;
                newcentry.at(oi)=0;
            }
    }

    for (unsigned int oi=0;oi<pcentrx.size();oi++)
    {
        if (pnx.at(oi)>0)
        {
            pcentrx.at(oi) = pcentrx.at(oi)/pnx.at(oi);
            pcentry.at(oi) = pcentry.at(oi)/pny.at(oi);
        }
        else
        {
            pcentrx.at(oi)=0;
            pcentry.at(oi)=0;
        }
   }

}


void tracker::findClosestNeighbors(cv::Mat markers, cv::Mat pmarkers)
{
    double min;
    double dist;
    int size_min;
    int size_max;
    double dist1, dist2;
    int diff;
    int point;


    size_max = newcentrx.size();
    if (pcentrx.size()>size_max)
        size_max = pcentrx.size();

    size_min = pcentrx.size();
    if (newcentrx.size() < size_min)
        size_min = newcentrx.size();



    newmarkers = cv::Mat::zeros(markers.size(), CV_8UC1);

    if ((newcentrx.size()==pcentrx.size())||(newcentrx.size()<pcentrx.size()))
    {
        for (unsigned int oi=0;oi<newcentrx.size();oi++)
            cn.push_back(0);

        for (unsigned int oi=0; oi<newcentrx.size(); oi++)
        {
        min =1000;

        for (unsigned int oo=0; oo<newcentrx.size(); oo++)
        {
            dist = this->getDist(oi,oo);
            if (dist<min)
            {
                min = dist;
                cn.at(oi)=nrlabelsp.at(oo);
            }
          }
        }

        for (i=0;i<markers.rows;i++)
            {
            for (j=0;j<markers.cols;j++)
                {
                    point = markers.at<uchar>(i,j);
                    int index1=0;

                    for (unsigned int k=0;k<nrlabelsn.size();k++)
                            if (point==nrlabelsn.at(k))
                                    index1=k;


                    newmarkers.at<uchar>(i,j) =cn.at(index1);

                }
            }

    }
    else if (newcentrx.size()>pcentrx.size())
    {
        for (unsigned int oi=0;oi<newcentrx.size();oi++)
            cn.push_back(0);

        vector<double> distV;

        for (unsigned int oi=0; oi<newcentrx.size(); oi++)
        {
        min =1000;

        for (unsigned int oo=0; oo<pcentrx.size(); oo++)
        {
            dist = this->getDist(oi,oo);
            if (dist<min)
            {
                min = dist;
                cn.at(oi)=nrlabelsp.at(oo);
            }
        }
        distV.push_back(min);

        }

        int cn_max=0;
        for (unsigned int oi=0;oi<cn.size();oi++)
            if(cn.at(oi)>cn_max)
                cn_max=cn.at(oi);

         for (unsigned int oi=0;oi<cn.size()-1;oi++){
              for (unsigned int oo=oi+1;oo<cn.size();oo++)
                  if(cn.at(oi)==cn.at(oo)){
                      if (distV.at(oi)>distV.at(oo))
                          cn.at(oi)=cn_max+1;
                      else
                          cn.at(oo)=cn_max+1;
                  }

         }


        for (i=0;i<markers.rows;i++)
            {
            for (j=0;j<markers.cols;j++)
                {

                point = markers.at<uchar>(i,j);
                int index1=0;

                for (unsigned int k=0;k<nrlabelsn.size();k++)
                        if (point==nrlabelsn.at(k))
                                index1=k;


                    newmarkers.at<uchar>(i,j) =cn.at(index1);
                }
            }
    }
//    else if (newcentrx.size()<pcentrx.size())
//    {
//        for (unsigned int oi=0;oi<newcentrx.size();oi++)
//            cn.push_back(0);

//        for (unsigned int oi=0; oi<newcentrx.size(); oi++)
//        {
//            min =1000;

//            for (unsigned int oo=0; oo<pcentrx.size(); oo++)
//            {
//                 dist = this->getDist(oi,oo);
//                 if (dist<min)
//                 {
//                     min = dist;
//                     cn.at(oi)=nrlabelsp.at(oo);
//                 }
//            }
//        }

//        for (i=0;i<markers.rows;i++)
//            {
//            for (j=0;j<markers.cols;j++)
//                {

//                point = markers.at<uchar>(i,j);
//                int index3=0;

//                for (unsigned int k=0;k<nrlabelsn.size();k++)
//                        if (point==nrlabelsn.at(k))
//                                index3=k;


//                    newmarkers.at<uchar>(i,j) =cn.at(index3);
//                }
//            }

//    }
}

/*
void tracker::findCentroids(cv::Mat markers, cv::Mat pmarkers) // find centroids of segmented objects
{
    int nx[ns];
    int ny[ns];
    int pnx[ns];
    int pny[ns];
    int point, ppoint;


    for (i=0;i<ns;i++) // for 5 labels most
    {
        nx[i]=0;
        ny[i]=0;
        centx[i]=0;
        centy[i]=0;
        pnx[i]=0;
        pny[i]=0;
        pcentx[i]=0;
        pcenty[i]=0;
    }

    for (i=0;i<markers.rows; i++)
    {
        for (j=0;j<markers.cols; j++)
        {

            point = markers.at<uchar>(i, j);
            ppoint = pmarkers.at<uchar>(i, j);

            for (k=0;k<ns;k++)
            {
                if (point == k) // check background and segments
                {
                    centx[k] = centx[k]+j;
                    centy[k] = centy[k]+i;
                    nx[k]++;
                    ny[k]++;
                }
                if (ppoint == k) // the same for previous frame
                {
                    pcentx[k] = pcentx[k]+j;
                    pcenty[k] = pcenty[k]+i;
                    pnx[k]++;
                    pny[k]++;
                }
            }
        }
    }

    for (i=0;i<ns;i++)
    {

        if (nx[i]>0)
        {
            centx[i]= centx[i]/nx[i];  //centroids of each label
            centy[i]= centy[i]/ny[i];
        }
        else if (nx[i]==0)
        {
            centx[i]=0;
            centy[i]=0;
        }
        if (pnx[i]>0)
        {
            pcentx[i]= pcentx[i]/pnx[i];  //centroids of each label
            pcenty[i]= pcenty[i]/pny[i];
        }
        else if (pnx[i]==0)
        {
            pcentx[i]=0;
            pcenty[i]=0;
        }
    }

}
*/

/*
void tracker::findClosestNeighbors(cv::Mat markers, cv::Mat pmarkers)
{
    int dx[ns];
    int dy[ns];
    double dists[ns][ns];
    int cn[ns]; // closest neighbor
    double min;
    int point;

    for (i=0;i<ns;i++)
    {
        for (j=0;j<ns;j++)
        {

            dists[i][j]=this->getDist(i,j);
        }
    }

    for (i=0;i<ns;i++) //find closest neighbor from previous frame
    {
        min=1000;//dists[i][0];
        cn[i]=0;
        for (j=0;j<ns;j++)
        {
            if (dists[i][j]<min)
            {
                min = dists[i][j];
                cn[i]=j;
            }
        }

      //  dx[i]=centx[i]-pcentx[cn[i]];
     //   dy[i]=centy[i]-pcenty[cn[i]];
    }
    newmarkers = cv::Mat::zeros(markers.size(), CV_8UC3);
    //move the previous labeled segmented object to the new position
    for (k=0;k<ns;k++)
    {

    for (i=0;i<markers.rows;i++)
        {
        for (j=0;j<markers.cols;j++)
            {

                point = markers.at<uchar>(i,j);
                newmarkers.at<uchar>(i,j) = cn[point];

            /*

                if ((markers.at<uchar>(i,j)!=0)&&(i-dx[k]<markers.rows)&&(j-dy[k]< markers.cols)&&(i-dx[k]>0)&&(j-dy[k]>0))
                {
                    newmarkers.at<uchar>(i,j) = pmarkers.at<uchar>(i-dx[k],j-dy[k]);
                }else{
                    newmarkers.at<uchar>(i,j) = 0;
                }

            }
        }
    }
}
*/

/*
void findlabels(cv::Mat markers, cv::Mat pmarkers)
{
    int point, ppoint;
    int falselabls[ns];
    double dists[ns][ns];
    int cn[ns];
    double min;



    for (i=0;i<ns;i++) // for 5 labels most
    {
        nx[i]=0;
        ny[i]=0;
        centx[i]=0;
        centy[i]=0;
        pnx[i]=0;
        pny[i]=0;
        pcentx[i]=0;
        pcenty[i]=0;
    }

    for ( i=0;i<ns;i++)
        falselabls[i]=0; //set to background

    for (k=0;k<ns;k++)
    {
        for ( i=0;i<markers.rows; i++)
        {
            for (j=0;j<markers.cols;j++)
            {
                //find centroid of label k
                point = markers.at<uchar>(i, j);
                ppoint = pmarkers.at<uchar>(i, j);
                if (point == k) // check background and segments
                {
                    centx[k] = centx[k]+i;
                    centy[k] = centy[k]+j;
                    nx[k]++;
                    ny[k]++;
                }
                if (ppoint == k) // the same for previous frame
                {
                    pcentx[k] = pcentx[k]+i;
                    pcenty[k] = pcenty[k]+j;
                    pnx[k]++;
                    pny[k]++;

                }
            }
        }
    }

    for (i=0;i<ns;i++)
    {

        if (nx[i]>0)
        {
            centx[i]= centx[i]/nx[i];  //centroids of each label
            centy[i]= centy[i]/ny[i];
        }
        else if (nx[i]==0)
        {
            centx[i]=0;
            centy[i]=0;
        }
        if (pnx[i]>0)
        {
            pcentx[i]= pcentx[i]/pnx[i];  //centroids of each label of previous frame
            pcenty[i]= pcenty[i]/pny[i];
        }
        else if (pnx[i]==0)
        {
            pcentx[i]=0;
            pcenty[i]=0;
        }
    }


    for (i=0;i<ns;i++)
        for (j=0;j<ns;j++)
            dists[i][j]=this->getDist(i,j);

    for (i=0;i<ns;i++) //find closest neighbor from previous frame
    {
        min=dists[i][0];
        cn[i]=0;
        for (j=1;j<ns;j++)
        {
            if (dists[i][j]<min)
            {
                min = dists[i][j];
                cn[i]=j;
            }
        }

        dx[i]=centx[i]-pcentx[cn[i]];
        dy[i]=centy[i]-pcenty[cn[i]];
    }


    for (k=0;k<ns;k++)
    {
        for (i=0;i<markers.rows;i++)
        {
            for (j=0;j<markers.cols;j++)
            {

                point = markers.at<uchar>(i,j);
                newmarkers.at<uchar>(i,j) = cn[point];

            }
        }
    }



}
*/

tracker::~tracker()
{
 //   delete[] centx;
 //   delete[] centy;
  //  delete[] pcentx;
  //  delete[] pcenty;
}
