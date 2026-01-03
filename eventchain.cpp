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



#include "eventchain.h"

#define EPSILON (2 * FLT_EPSILON)
#define  THRD 0.00000000000000001  //0.017  //0.02 //0.015
#define THRO 0.01 //0.01  //0.012
#define NTHREADS 4


using namespace std;
using namespace Eigen;
using namespace cv;



eventchain::eventchain(const std::string path[], int nn )
{
    sc=5;
    n=nn;
    thresholdd=0.00000001;
    sequence.reserve(n);


    keyframeflag = new int[n];
    keyframeflag[0]=1;
    for (i=1;i<n;i++)
        keyframeflag[i]=0;


    for (i=0;i<n;i++)
    {
        imagergb *im = new imagergb(path,i);
        sequence.push_back(im);

    }



    x.reserve(sc);
    y.reserve(sc);

    //original semantic event chain
    osec = new int*[n];
    for (i=0;i<n;i++)
        osec[i]=new int[sc*sc];

    dsec1 = new int*[n-1];
    for (i=0;i<(n-1);i++)
        dsec1[i]= new int[sc*sc];

    dsec2 = new int*[n-1];
    for (i=0;i<(n-1);i++)
        dsec2[i]=new int[sc*sc];

    eigenval = new double*[sc];
    for (i=0;i<sc;i++)
        eigenval[i]=new double[n];

    //initialize eigenvalues
    for (i=0;i<sc;i++)
        for (j=0;j<n;j++)
            eigenval[i][j]=0;

}


eventchain::eventchain()
{

//    double va1,va2,dott;
//    Vector3d axx;
//    Vector3d axx2;
//    Vector3d rotax1;
//    Vector3d rotax2;
    cv::Mat frame;
//    cv::Mat pmarkers;
//    cv::Mat markers;
//    cv::Mat savergb;
    std::vector<float> ypr2;
    std::vector<float> center2;
    int numparameters;
    x.reserve(1500);
    y.reserve(1500);


    // READ POINT CLOUDS FROM MODELS and FIND THE FITTING ELLIPSOID
    std::string par_filename;
    ostringstream pfilename;
    ifstream fileob0;

    std::string fileob1="model_pcb.pcd";
    std::string fileob2="model_case.pcd";
    ellipsoid * ob1 = new ellipsoid(0.008, fileob1);
    ellipsoid * ob2 = new ellipsoid(0.008, fileob2);

    objects.push_back(ob1);
    objects.push_back(ob2);

    //DUAL REDUCED NEWTON for ELLIPSOID FITTING
    objects.at(0)->DRNcore();
    objects.at(1)->DRNcore();

//    axx(0)=0;
//    axx(1)=1;
//    axx(2)=0;

//    axx2(0)=1;
//    axx2(1)=0;
//    axx2(2)=0;
    numparameters=7;
    //READ FILE FOR OB 2 POSE AND POSITION
    float * coeff_ob2;
    float * coeff_ob1;
    coeff_ob2=new float[numparameters];

    fileob0.open ("bins/object_detection.txt");
    if (!fileob0.is_open()) return;

    std::string word;
    std::string fff;
    fff="0";


//    while (fileob0 >> word)
//    {
//        if (word.compare(fff)==0)
//        {
//            //coeff_ob1[0]=atof(word.c_str());
//            int jl=0;
//            while (fileob0 >> word)
//            {
//                coeff_ob2[jl]=atof(word.c_str());
//                jl++;
//            }
//        }
//    }
//    for (int io=0;io<3;io++)
//        center2.push_back(coeff_ob2[io]);

//    for (int io=3;io<6;io++)
//        if (io==5)
//            ypr2.push_back(coeff_ob2[io]);
//        else if (io==4)
//            ypr2.push_back(coeff_ob2[io]+1.57);
//        else if (io==3)
//            ypr2.push_back(coeff_ob2[io]+1.57);

    std::string filename_ob = "bins/object_detection.txt" ;
       std::cout << "KeyframeExtractor: Object Detection: "<<filename_ob<<std::endl;


    std::ifstream f(filename_ob.c_str());

     if(!f.is_open())
     {
         std::cerr<<"KeyframeExtractor: Unable to find object detection file: "<<filename_ob<<std::endl;
         //return false;
     }
     //if(!f) return false;

     int id = -1;
     int obj_from = 0;
     float x,y,z,rx,ry,rz;
     while(f >> id >> x >> y >> z >> rx >> ry >> rz) {
         if(id == obj_from) break;
     }
     f.close();


     coeff_ob2[0] = x;
     coeff_ob2[1] = y;
     coeff_ob2[2] = z;
     coeff_ob2[3] = rx;
     coeff_ob2[4] = ry;
     coeff_ob2[5] = rz;
     for(int yy=0; yy<6; yy++)
        std::cout<< "Coeff obj2["<<yy<<"] = "<< coeff_ob2[yy]<<std::endl;

 //    std::string temp1;
 //    getline(std::cin, temp1);


     for (int io=0;io<3;io++)
         center2.push_back(coeff_ob2[io]);

     for (int io=3;io<6;io++)
         ypr2.push_back(coeff_ob2[io]);
    n=0;

    maskfname = "mask/mask_0000000.png";
//    rgbfname = "rgb/rgb_0000000.jpg";

    frame = imread(maskfname, CV_LOAD_IMAGE_GRAYSCALE);
//    savergb = imread(rgbfname, CV_LOAD_IMAGE_COLOR);
    int *frg;

    frg = new int[frame.rows*frame.cols];


    while (!frame.empty())
    {
        //here apply segmentation with watershed on frame and labeling of segments


        cv::Mat labels;
        cv::Mat mylabels;
        cv::Mat filtered;
        std::vector<float> ypr1;
        std::vector<float> center1;


        mylabels = cv::Mat::zeros(frame.size(), CV_8UC1);

        filtered =cv::Mat::zeros(frame.size(), CV_8UC3);

        for (int ii=0;ii<mylabels.rows; ii++)
        {
            for (int jj=0;jj<mylabels.cols;jj++)
            {
                frg[ii*frame.cols+jj]=frame.at<uchar>(ii,jj);

                if (frg[ii*frame.cols + jj]==0)
                {
                    mylabels.at<uchar>(ii,jj) = 0;
                    filtered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(0,0,0);

                }else if (frg[ii*frame.cols +jj]==1)
                {
                    mylabels.at<uchar>(ii,jj) = 1;
                    filtered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(250,0,0);

                }else if (frg[ii*frame.cols + jj] ==2)
                {
                    mylabels.at<uchar>(ii,jj) = 2;
                    filtered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(210,210,210);

                }else if (frg[ii*frame.cols + jj]==3)
                 {
                    mylabels.at<uchar>(ii,jj) = 3;
                    filtered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(180,190,220);

                }else if (frg[ii*frame.cols + jj]==4)
                {
                    mylabels.at<uchar>(ii,jj) = 4;
                    filtered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(0,190,0);
                }
            }
        }

 //       markers= mylabels;
//        namedWindow("segmented");
//        imshow("segmented", filtered);
//        waitKey(200);



//            markers.convertTo(markers,CV_8UC1);

            //initialize tracker and run basic routines
//            tracker * track = new tracker();

//            if (n==0)
//            {
//                pmarkers = markers;

//            }
//            track->findCentroids(markers,pmarkers);
//            track->findClosestNeighbors(markers,pmarkers);


            //NOW READ BIN FILES AND GET POSITIONS AND POSES OF OBJECT


            //load parameters from bin files

            // 7 parameters for object
            numparameters = 7;

            coeff_ob1=new float[numparameters];


            pfilename.str("");

            pfilename << n;

            if (n==0)
            {
                par_filename = "bins/coeffobj01_0000000.bin";
            }else if (n<10)
            {
                par_filename = "bins/coeffobj01_000000"+ pfilename.str() + ".bin";
            }else if (n<100)
            {
                par_filename = "bins/coeffobj01_00000" + pfilename.str() + ".bin";
            }else
            {
                par_filename = "bins/coeffobj01_0000" + pfilename.str()+ ".bin";
            }

            if (this->loadFromFile(par_filename, coeff_ob1, numparameters))
            {
                //cout << "object 2 parameters loaded"<<endl;
                for (int io=0;io<2;io++)
                    center1.push_back(coeff_ob1[io]/coeff_ob1[6]);
                center1.push_back((-1)*coeff_ob1[2]/coeff_ob1[6]);
                ypr1.push_back(((-1)*coeff_ob1[3]));
                ypr1.push_back((-1)*coeff_ob1[4]);
                ypr1.push_back(coeff_ob1[5]);


            }
            else
            {
                cout << "object 2 parameters loading went wrong!"<<endl;
            }




            //rotate each ellipsoid and move to center according to tracker's output
            objects.at(0)->rotateQ(ypr1);
            objects.at(1)->rotateQ(ypr2);
            objects.at(0)->moveC(center1);
            objects.at(1)->moveC(center2);

//            rotax1 = objects.at(0)->Rot*axx2;
//            rotax2 = objects.at(1)->Rot*axx;

//            va1=std::sqrt((rotax1(0)*rotax1(0)) +
//                          (rotax1(1)*rotax1(1)) +
//                          (rotax1(2)*rotax1(2)));
//            va2=std::sqrt((rotax2(0)*rotax2(0)) +
//                          (rotax2(1)*rotax2(1)) +
//                          (rotax2(2)*rotax2(2)));

//            dott = ((rotax1(0)*rotax2(0)) + (rotax1(1)*rotax2(1)) + (rotax1(2)*rotax2(2)));
//            cout << "FRAME: "<<n <<" AXES angle: "<< (180/3.14)*std::acos(std::abs(dott/(va1*va2))) <<endl;



            //now use the rotated ellipsoids as input to imagergb


            imagergb *im = new imagergb(frame, n, mylabels,
                                        objects.at(0)->RotQ,
                                        objects.at(1)->RotQ,
                                        objects.at(0)->cnew,
                                        objects.at(1)->cnew);
                //imagergb *im = new imagergb(frame, n, mylabels); //bring labels image with it
            //and keep every frame in memory
            sequence.push_back(im);

            for (int i=0;i<3;i++)
                finst.push_back(-1);

//            pmarkers = markers;//track->newmarkers;


            n++;

            ostringstream nn2;
            nn2 << n;

            if (n<10)
            {
                maskfname = "mask/mask_000000"+ nn2.str() + ".png";
            }else if (n<100)
            {
                maskfname = "mask/mask_00000"+ nn2.str() + ".png";
            }else
            {
                maskfname = "mask/mask_0000"+ nn2.str() + ".png";
            }

            frame = imread(maskfname, CV_LOAD_IMAGE_GRAYSCALE);

            //         tosave = "/home/gpiperagkas/SARAFun_kf/"+ rgbfname;
            //              imwrite(tosave,savergb);



            //         if (n<10)
            //         {
            //             rgbfname = "rgb/rgb_000000"+ nn2.str() + ".jpg";
            //         }else if (n<100)
            //         {
            //             rgbfname = "rgb/rgb_00000"+ nn2.str() + ".jpg";
            //         }else
            //         {
            //             rgbfname = "rgb/rgb_0000"+ nn2.str() + ".jpg";
            //         }


            mylabels.release();
            filtered.release();
            ypr1.clear();
            center1.clear();



    }


    nini=n;

    delete[] frg;

    sc=15;
    assembly_type = 0; //folding

    keyframeflag = new int[nini];
    keyframeflag[0]=1;
    for (i=1;i<nini;i++)
        keyframeflag[i]=0;



    //original semantic event chain
    osec = new int*[nini];
    for (i=0;i<nini;i++)
        osec[i]=new int[sc*sc];

    dsec1 = new int*[nini-1];
    for (i=0;i<(nini-1);i++)
        dsec1[i]= new int[sc*sc];

    dsec2 = new int*[nini-1];
    for (i=0;i<(nini-1);i++)
        dsec2[i]=new int[sc*sc];

    eigenval = new double*[sc];
    for (i=0;i<sc;i++)
        eigenval[i]=new double[nini];

    //initialize eigenvalues
    for (i=0;i<sc;i++)
        for (j=0;j<nini;j++)
            eigenval[i][j]=0;

    delete [] coeff_ob1;
    ypr2.clear();
    center2.clear();
    delete [] coeff_ob2;

}




void eventchain::printtoFile(cv::Mat labels)
{
          cv::FileStorage file("labels.xml", cv::FileStorage::WRITE);
          file <<"labels" <<labels;
}




void eventchain::buildInitChain()
{
    int found;
    int flx, fly, counter;
    int parres;
    int ns;

    bool parmaj=false;
    bool parmid=false;
    bool parmin=false;

        for (i=0;i<(nini);i++) //initialize
            for (j=0;j<(sc);j++)
                for (k=0;k<sc;k++)
                    osec[i][j*sc+k] = 0;


        // detect parallelism option from last frame by default: finstate
        ns = sequence.size();
        sequence.at(ns-1)->ElliAngles(-1, finst);
        finst.at(0) = sequence.at(ns-1)->finstate.at(0);
        finst.at(1) = sequence.at(ns-1)->finstate.at(1);
        finst.at(2) = sequence.at(ns-1)->finstate.at(2);
        cout << "finstate: "<< sequence.at(ns-1)->finstate.at(0) << " "<<
                               sequence.at(ns-1)->finstate.at(1) << " "<<
                                sequence.at(ns-1)->finstate.at(2)<<endl;

        std::cout << "calcNORMHINTS 0" <<std::endl;

        //START PARALLELIZATION OF IMAGE SCANNING
        boost::asio::io_service ioService;

        // Add work to ioService.
        for (unsigned int ll=0; ll < sequence.size(); ll++)
        {
             ioService.post(boost::bind( &imagergb::scanSegmented, sequence.at(ll)));

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

        boost::asio::io_service ioService2;

        for (unsigned int ll=0; ll< sequence.size();ll++)
            ioService2.post(boost::bind( &imagergb::setSegRelations, sequence.at(ll)));

        boost::thread_group threadpool2;
        for (unsigned int il = 0; il < NTHREADS; il++)
        {
          threadpool2.create_thread(boost::bind(
              &boost::asio::io_service::run, &ioService2));
        }
        threadpool2.join_all();
        boost::asio::io_service ioService3;

        for (unsigned int ll=0; ll< sequence.size();ll++)
            ioService3.post(boost::bind( &imagergb::calcNormHints, sequence.at(ll)));

        boost::thread_group threadpool3;
        for (unsigned int il = 0; il < NTHREADS; il++)
        {
          threadpool3.create_thread(boost::bind(
              &boost::asio::io_service::run, &ioService3));
        }

        threadpool3.join_all();

        //END PARALLELIZATIN OF IMAGE SCANNING

//        for (unsigned int ll=0; ll < sequence.size(); ll++)
//        {   //scan all images in the vector and save the data

//            sequence.at(ll)->scanSegmented();

//            sequence.at(ll)->setSegRelations();

//            sequence.at(ll)->calcNormHints();


//        }

        //sequence.at(80)->printhints();

        for (unsigned int ll=0; ll<sequence.size(); ll++)
        {
            parmaj=false;
            parmid=false;
            parmin=false;
            for (i=0;i<(sc);i++)
            {
                for (j=0;j<sc;j++)
                {
                    if ((i<5)&&(j<5)) //for dummy images i, j < 4
                    {

                        if ((sequence.at(ll)->psize[i]==0)||(sequence.at(ll)->psize[j]==0))
                            osec[ll][i*sc+j]=11; //absense of segment
                        else
                        {

                            if (sequence.at(ll)->normct[i*sc+j]>THRD)
                                osec[ll][i*sc+j] = 2; //touching

                            if (sequence.at(ll)->normco[i*sc+j]>THRO)
                                osec[ll][i*sc+j] = 1; //overlapping

                            if ((sequence.at(ll)->normco[i*sc+j]<THRO)&&(sequence.at(ll)->normct[i*sc+j]<THRD))
                                osec[ll][i*sc+j] = 0; //no edge between them

                        }
                    }else
                    {
                        osec[ll][i*sc+j]=11;

                    }
                    if (sequence.at(ll)->feature==1) // if we have ellipsoids
                    {
                        if (sequence.at(ll)->normcel[i*sc+j]==0)
                        {
                            osec[ll][i*sc+j]=0;
                        }
                        else if (sequence.at(ll)->normcel[i*sc+j]==1)
                        {
                            if (sequence.at(ll)->normco[i*sc+j]>THRO)
                                osec[ll][i*sc+j]=1;

                            else if (sequence.at(ll)->normct[i*sc+j]>THRD)
                                osec[ll][i*sc+j]=2;

                        }

                    }

                }
            }
            if (sequence.at(ll)->feature==1)
            {


                // suppose ellipsoids are for objects 2 & 3 fixed
                // check paralellism of  ellipsoids
                //mode 0: major axis parallel
                //mode 1: middle axis parallel
                //mode 2: minor axis parallel
                //mode 3: all axes parallel
                sequence.at(ll)->ElliAngles(0, finst);

                if (sequence.at(ll)->parstate.at(0) != -1)
                {
                    osec[ll][2*sc+3]= 3;  //major axis parallel according to schema provided
                    osec[ll][3*sc+2]= 3;
                    parmaj=true;
                }

                if (sequence.at(ll)->parstate.at(1) != -1)
                {
                    osec[ll][2*sc+3]= 4;  //middle axis parallel according to schema provided
                    osec[ll][3*sc+2]= 4;
                    parmid=true;
                }

                if (sequence.at(ll)->parstate.at(2) != -1)
                {
                    osec[ll][2*sc+3]= 5;  //minor axis parallel according to schema provided
                    osec[ll][3*sc+2]= 5;
                    parmin=true;
                }
                if ((parmin) && (parmid) && (parmaj))
                {
                    osec[ll][2*sc+3]= 6;  //all axes parallel according to schema provided
                    osec[ll][3*sc+2]= 6;
                }
                //one axis + relation
                if (((parmin)||(parmid)||(parmaj))&&
                        (sequence.at(ll)->normco[2*sc+3]>THRO)&&
                        (sequence.at(ll)->normcel[2*sc+3]==1))
                {
                    osec[ll][2*sc+3]= 9;

                }else if (((parmin)||(parmid)||(parmaj))&&
                          (sequence.at(ll)->normco[3*sc+2]>THRO)&&
                          (sequence.at(ll)->normcel[3*sc+2]==1))
                {
                    osec[ll][3*sc+2]=9;

                }else if (((parmin)||(parmid)||(parmaj))&&
                          (sequence.at(ll)->normct[2*sc+3]>THRD)&&
                          (sequence.at(ll)->normcel[2*sc+3]==1))
                {
                    osec[ll][2*sc+3]=7;
                    osec[ll][3*sc+2]=7;
                }else if (((parmin)||(parmid)||(parmaj))&&
                          (sequence.at(ll)->normct[3*sc+2]>THRD)&&
                          (sequence.at(ll)->normcel[3*sc+2]==1))
                {
                    osec[ll][2*sc+3]=7;
                    osec[ll][3*sc+2]=7;
                }

                //all axes + relation
                if (((parmin)&&(parmid)&&(parmaj))&&
                        (sequence.at(ll)->normco[2*sc+3]>THRO)&&
                        (sequence.at(ll)->normcel[2*sc+3]==1))
                {
                    osec[ll][2*sc+3]=10;

                }else if (((parmin)&&(parmid)&&(parmaj))&&
                          (sequence.at(ll)->normco[3*sc+2]>THRO)&&
                          (sequence.at(ll)->normcel[3*sc+2]==1))
                {
                    osec[ll][3*sc+2]=10;

                }else if (((parmin)&&(parmid)&&(parmaj))&&
                          (sequence.at(ll)->normct[2*sc+3]>THRD)&&
                          (sequence.at(ll)->normcel[2*sc+3]==1))
                {
                    osec[ll][2*sc+3]=8;
                    osec[ll][3*sc+2]=8;
                }else if (((parmin)&&(parmid)&&(parmaj))&&
                          (sequence.at(ll)->normct[3*sc+2]>THRD)&&
                          (sequence.at(ll)->normcel[3*sc+2]==1))
                {
                    osec[ll][2*sc+3]=8;
                    osec[ll][3*sc+2]=8;
                }
            }
        }

        found=0;
        for (i=0;i<nini;i++) //build coordinates vectors of nodes
        {
            for (j=0;j<sc;j++)
            {
                for (k=0;k<sc;k++)//j;k++)
                {

                    if ((osec[i][j*sc+k]!=11))//&&(osec[i][j*sc+k]!=1))
                    {
                        for (unsigned int ll=0; ll<x.size();ll++)
                        {
                            if ((x.at(ll)==j)&&(y.at(ll)==k))
                                found=1;
                            else
                                found=0;
                        }

                        if (found==0)
                        {
                            x.push_back(j);
                            y.push_back(k);
                        }

                    }
                }
            }
        }




   //erase duplicates from coordinates  -- (!) has some outliers -> 4 passes to clear
   for (i=0;i<4;i++)
   {
      counter=1;
      for (unsigned int ll=0;ll<x.size();ll++)
      {
          flx=x.at(ll);
          fly=y.at(ll);

          for (unsigned int kk=counter;kk<x.size();kk++)
          {
              if ((x.at(kk)==flx)&&(y.at(kk)==fly))
              {
                  x.erase(x.begin()+kk); //erase the kk element
                  y.erase(y.begin()+kk);
              }
          }
          counter++;
      }
   }



}


void eventchain::calcEigenValues(int nim) //calculate eigenvalues of each frame
{
       MatrixXd m(sc,sc);     //adjacency matrix for each frame




       for (i=0;i<sc;i++)
       {
           for (j=0;j<sc;j++)
           {
               m(i,j)=osec[nim][i*sc+j];
           }
       }


      //  SelfAdjointEigenSolver<MatrixXd> eigensolver(m);

        EigenSolver<MatrixXd> eigensolver(m);
        complex<double> E;

        //if (eigensolver.info() != Success) abort();

        for( i = 0; i < m.rows(); ++i)
        {
            E=eigensolver.eigenvalues().col(0)[i];
           // eigenval[i][nim] = eigensolver.eigenvalues().col(0)[i];
          //  cout << E << endl;
            if (E.imag()==0)
            {
                eigenval[i][nim] = E.real();
            }else
            {
                // if it has complex eigenvalues, multiply with the adjoint
                eigenval[i][nim] = (E.real()*E.real()) + (E.imag()*E.imag());
            }


        }

}


void eventchain::extractKeyframes()
{
    keyframeflag[0]=1;

    int keyn=0;
        for (i=0;i<(sc-1);i++)
        {
            for (j=1;j<nini;j++)
            {
                //percentage = std::abs(eigenval[i][j]-eigenval[i][j-1])/eigenval[i][j];
                if (eigenval[i][j]!=eigenval[i][j-1])
                {
                    keyframeflag[j]=1;
/*

                    for (int ii=i+1;ii<(sc);ii++)
                    {
                        if (eigenval[ii][j]!=eigenval[ii][j-1])
                            keyframeflag[j]=1;

                    }
*/
                }
            }
        }

        for (i=2;i<nini;i++)
        {
            if (keyframeflag[i]==1)
            {
                if ((keyframeflag[i-1]==1)||(keyframeflag[i-2]==1))
                {
                    keyframeflag[i-1]=0;
                    keyframeflag[i-2]=0;
                }
            }
        }

        for (i=0;i<nini;i++)
            if (keyframeflag[i]==1)
                keyn++;

        ofstream keyframeout;
        keyframeout.open("/home/gpiperagkas/SARAFun_kf/keyframes_list.txt");
        for (i=0; i<nini;i++)
            if (keyframeflag[i]==1)
                keyframeout << i << endl;




    for (i=0;i<nini;i++)
        cout << keyframeflag[i] << " ";
    cout << endl;
}

void eventchain::derivativeSEC() //construct derivative of SEC
{

    //initialize dsec
    for (i=0;i<(nini-1);i++)
    {
        for (j=0;j<sc;j++)
        {
            for (k=0;k<sc;k++)
            {
                dsec1[i][j*sc+k]=0;
                dsec2[i][j*sc+k]=0;

            }
        }
    }

    for (i=0;i<(nini-2);i++) //rows
    {
          for (j=0;j<sc;j++) //cols
          {
             for (k=0;k<sc;k++) //cols
             {
                 dsec1[i][j*sc+k] = osec[i][j*sc+k];
                 dsec2[i][j*sc+k] = osec[i+1][j*sc+k];

             }
          }
     }
}

void eventchain::constructCSEC() //construct compressed SEC based on x and y coordinates
{

    for(unsigned int i = 0; i<x.size(); i++)
    {
        vector<int> r1;
        vector<int> r2;
        for(int j = 0; j<(nini-1); j++)
        {

            if (dsec1[j][x.at(i)*sc+y.at(i)]!=dsec2[j][x.at(i)*sc+y.at(i)])
            {
                r1.push_back(dsec1[j][x.at(i)*sc+y.at(i)]);
                r2.push_back(dsec2[j][x.at(i)*sc+y.at(i)]);
            }
        }
        ksi1.push_back(r1);
        ksi2.push_back(r2);

    }

    cout << "size of ksi: " << ksi1.size()<<endl;
    for (unsigned int ll=0;ll<ksi1.size();ll++)
    {
        cout << x.at(ll) << " " << y.at(ll) << ": ";
        for (unsigned int kk=0;kk<ksi1[ll].size();kk++)
        {
            cout << ksi1[ll][kk] << ksi2[ll][kk] <<  " ";
        }
        cout << endl;
    }

}

void eventchain::visualizeKeyframes()
{
       int nf=0;
       bool lhandExists=true;
       bool rhandExists=true;
       int numparameters;
       std::string par_filename;
       ostringstream pfilename;
       std::string keyname;
       std::string keyfilename;
       std::string keyframenm;
       ifstream fileob0;
       float *coeff_ob2;
       std::vector<double> posob2;
       std::vector<double> quatob2;
       cv::Mat keyframe;
//       if( !viscap1.isOpened() )
//               cout << "Error when reading capture video";

//       viscap1 >> keyframe;

       fileob0.open ("bins/object_detection.txt");
       if (!fileob0.is_open()) return;

       std::string word;
       std::string fff;
       fff="0";

       numparameters=7;
       coeff_ob2=new float[numparameters];

       std::string filename_ob = "bins/object_detection.txt" ;
          std::cout << "KeyframeExtractor: Object Detection: "<<filename_ob<<std::endl;


       std::ifstream f(filename_ob.c_str());

        if(!f.is_open())
        {
            std::cerr<<"KeyframeExtractor: Unable to find object detection file: "<<filename_ob<<std::endl;
            //return false;
        }
        //if(!f) return false;

        int id = -1;
        int obj_from = 0;
        float x,y,z,rx,ry,rz;
        while(f >> id >> x >> y >> z >> rx >> ry >> rz) {
            if(id == obj_from) break;
        }
        f.close();


        coeff_ob2[0] = x;
        coeff_ob2[1] = y;
        coeff_ob2[2] = z;
        coeff_ob2[3] = rx;
        coeff_ob2[4] = ry;
        coeff_ob2[5] = rz;
        for(int yy=0; yy<6; yy++)
           std::cout<< "Coeff obj2["<<yy<<"] = "<< coeff_ob2[yy]<<std::endl;

    //    std::string temp1;
    //    getline(std::cin, temp1);


        for (int io=0;io<3;io++)
           posob2.push_back(coeff_ob2[io]);

        for (int io=3;io<6;io++)
           quatob2.push_back(coeff_ob2[io]);




//       while (fileob0 >> word)
//       {
//           if (word.compare(fff)==0)
//           {
//               //coeff_ob1[0]=atof(word.c_str());
//               int jl=0;
//               while (fileob0 >> word)
//               {
//                   coeff_ob2[jl]=atof(word.c_str());
//                   jl++;
//               }
//           }
//       }
//       for (int io=0;io<3;io++)
//           posob2.push_back(coeff_ob2[io]);

//       for (int io=3;io<6;io++)
//           quatob2.push_back(coeff_ob2[io]);

       keyframenm = "mask/mask_0000000.png";
       keyframe = imread(keyframenm,CV_LOAD_IMAGE_GRAYSCALE);
       while(!keyframe.empty())
       {
           if (keyframeflag[nf]==1)
           {

               ostringstream convert;
               convert << nf;
               keyname = convert.str();
             //  cout << keyname << endl;
              // namedWindow(keyname, WINDOW_AUTOSIZE);
              // imshow(keyname, keyframe);
               //waitKey(1000);
               keyfilename = "/home/gpiperagkas/SARAFun_kf/" + keyname + ".jpg";

               cv::Mat filteredd = cv::Mat::zeros(keyframe.size(), CV_8UC3);
               int *fr;
               fr = new int[keyframe.rows*keyframe.cols];

               for (int ii=0;ii<keyframe.rows; ii++)
               {
                   for (int jj=0;jj<keyframe.cols;jj++)
                   {
                       fr[ii*keyframe.cols+jj]=keyframe.at<uchar>(ii,jj);

                       if (fr[ii*keyframe.cols + jj]==0)
                       {
                           filteredd.at<cv::Vec3b>(ii,jj) = cv::Vec3b(0,0,0);

                       }else if (fr[ii*keyframe.cols +jj]==1)
                       {
                           filteredd.at<cv::Vec3b>(ii,jj) = cv::Vec3b(250,0,0);

                       }else if (fr[ii*keyframe.cols + jj] ==2)
                       {
                           filteredd.at<cv::Vec3b>(ii,jj) = cv::Vec3b(210,210,210);

                       }else if (fr[ii*keyframe.cols + jj]==3)
                        {
                           filteredd.at<cv::Vec3b>(ii,jj) = cv::Vec3b(180,190,220);

                       }else if (fr[ii*keyframe.cols + jj]==4)
                       {
                           filteredd.at<cv::Vec3b>(ii,jj) = cv::Vec3b(0,190,0);
                       }
                   }
               }


               imwrite(keyfilename,filteredd);
               filteredd.release();
               delete[] fr;
           }


           //PRINT XML FILE HERE



           std::vector<double> posob1;
           std::vector<double> quatob1;
           std::vector<double> inposl;
           std::vector<double> inqual;
           std::vector<double> inposr;
           std::vector<double> inquar;
           std::vector<double> CPl;
           std::vector<double> CFl;
           std::vector<double> CPr;
           std::vector<double> CFr;

           std::vector<int> RHandOI;
           std::vector<int> LHandOI;
           for (unsigned int lk=0;lk<2;lk++)
           {
               RHandOI.push_back(0);
               LHandOI.push_back(0);
           }
           int ObjInt;
           std::string InstrDescr= "test 000";
           float *coeff_ob1;
           float *coeff_hand1;
           float *coeff_hand2;

           //load parameters from bin files

            // 7 parameters for object, 48 for hands
            numparameters = 7;

            coeff_ob1=new float[numparameters];



            //SET SEMANTICS for XML-- 1 is right hand, 2 is objA, 3 is objB, 4 is left hand
            ObjInt = osec[nf][2*sc+3];
            if (osec[nf][2*sc+3]==1)
                ObjInt = 1;
            else if (osec[nf][3*sc+2]==1)
                ObjInt =11;
            if (osec[nf][2*sc+0]==11)
                ObjInt = 12;
            if (osec[nf][3*sc+0]==11)
                ObjInt = 13;

            if (osec[nf][1*sc+2]==11)
                RHandOI[0]=11;
            else if (osec[nf][1*sc+3]==11)
                RHandOI[0]=11;
            else if ((osec[nf][1*sc+2]==0)&&(osec[nf][1*sc+3]==0))
                RHandOI[0]=0;
            else if ((osec[nf][1*sc+2]==2)||(osec[nf][1*sc+3]==2))
                RHandOI[0]=2;

            if (osec[nf][4*sc+2]==11)
                LHandOI[0]=11;
            else if (osec[nf][4*sc+3]==11)
                LHandOI[0]=11;
            else if ((osec[nf][4*sc+2]==0)&&(osec[nf][4*sc+3]==0))
                LHandOI[0]=0;
            else if ((osec[nf][4*sc+2]==2)||(osec[nf][4*sc+3]==2))
                LHandOI[0]=2;

            if ((RHandOI[0]==11)||(RHandOI[0]==0))
                RHandOI[1]=3;
            else if (osec[nf][1*sc+2]==2)
                RHandOI[1]=1;
            else if (osec[nf][1*sc+3]==2)
                RHandOI[1]=2;

            if ((LHandOI[0]==11)||(LHandOI[0]==0))
                LHandOI[1]=3;
            else if (osec[nf][4*sc+2]==2)
                LHandOI[1]=1;
            else if (osec[nf][4*sc+3]==2)
                LHandOI[1]=2;

           // par_filename = "coeffobj01_0000002.bin";
//            if (this->loadFromFile(par_filename, coeff_ob1, numparameters))
//            {
               // cout << "object 1 parameters loaded"<< endl;


//            }
//            else
//            {
//                cout << "object 0 parameters loading went wrong!"<<endl;
//            }

            pfilename.str("");

            pfilename << nf;

            if (nf==0)
            {
                par_filename = "bins/coeffobj01_0000000.bin";
            }else if (nf<10)
            {
                par_filename = "bins/coeffobj01_000000"+ pfilename.str() + ".bin";
            }else if (nf<100)
            {
                par_filename = "bins/coeffobj01_00000" + pfilename.str() + ".bin";
            }else
            {
                par_filename = "bins/coeffobj01_0000" + pfilename.str()+ ".bin";
            }
//            par_filename = "coeffobj01_0000002.bin";

            if (this->loadFromFile(par_filename, coeff_ob1, numparameters))
            {
                //cout << "object 2 parameters loaded"<<endl;
                for (int io=0;io<2;io++)
                    posob1.push_back(coeff_ob1[io]/coeff_ob1[6]);
                posob1.push_back((-1)*coeff_ob1[2]/coeff_ob1[6]);
                quatob1.push_back((-1)*coeff_ob1[3]);
                quatob1.push_back((-1)*coeff_ob1[4]);
                quatob1.push_back(coeff_ob1[5]);

            }
            else
            {
                cout << "object 2 parameters loading went wrong!"<<endl;
            }

            numparameters=48;
            coeff_hand1 = new float[numparameters];
            coeff_hand2 = new float[numparameters];
            //par_filename = "coeffl_0000002.bin";
            pfilename.str("");

            pfilename << nf;

            if (nf==0)
            {
                par_filename = "bins/coeffl_0000000.bin";
            }else if (nf<10)
            {
                par_filename = "bins/coeffl_000000"+ pfilename.str() + ".bin";
            }else if (nf<100)
            {
                par_filename = "bins/coeffl_00000" + pfilename.str() + ".bin";
            }else
            {
                par_filename = "bins/coeffl_0000" + pfilename.str()+ ".bin";
            }

            if (!this->loadFromFile(par_filename, coeff_hand1, numparameters))
            {
                cout << "left hand parameters loading went wrong!" << endl;
                lhandExists=false;
                for (int ll=0;ll<6;ll++)
                    coeff_hand1[ll]=0;
            }

//            par_filename = "coeffl_0000002.bin";

            pfilename.str("");

            pfilename << nf;

            if (nf==0)
            {
                par_filename = "bins/coeffr_0000000.bin";
            }else if (nf<10)
            {
                par_filename = "bins/coeffr_000000"+ pfilename.str() + ".bin";
            }else if (nf<100)
            {
                par_filename = "bins/coeffr_00000" + pfilename.str() + ".bin";
            }else
            {
                par_filename = "bins/coeffr_0000" + pfilename.str()+ ".bin";
            }

            if (!this->loadFromFile(par_filename, coeff_hand2, numparameters))
            {
                cout << "right hand parameters loading went wrong!"<< endl;
                rhandExists=false;
                for (int ll=0;ll<6;ll++)
                    coeff_hand2[ll]=0;

            }

            //write xml files for left and right hands
            if (lhandExists)
            {
                xmlhandparser *handlparser = new xmlhandparser(nf, coeff_hand1, 0);
                handlparser->setKeyFrame(nf,1.25);
                handlparser->setHandpars();
                handlparser->setClosingData(nf);
            }
            if (rhandExists)
            {
                xmlhandparser *handrparser = new xmlhandparser(nf, coeff_hand2, 1);
                handrparser->setKeyFrame(nf,1.25);
                handrparser->setHandpars();
                handrparser->setClosingData(nf);
            }

           for (int io=0;io<3;io++)
           {
//               posob1.push_back(10.0);
//               posob2.push_back(20.0);
               inposl.push_back(35.5);
               inposr.push_back(43.3);
           }

           for (int io=0;io<4;io++)
           {
//               quatob1.push_back(90);
//               quatob2.push_back(45.5);
               inqual.push_back(30);
               inquar.push_back(10);
           }

           for (int io=0;io<9;io++)
           {
               if (io<3)
                   CPl.push_back(12.4);
               else
                   CPl.push_back(100.5);
               CPr.push_back(21.5);
               CFl.push_back(50);
               CFr.push_back(88);
           }


           xmlparser *xml = new xmlparser(nf,"assembly.mpg", "samplexml2.xml", posob1, quatob1, posob2, quatob2,
                                          inposl, inqual, inposr, inquar, CPl, CFl, CPr, CFr, coeff_hand1, coeff_hand2,
                                          InstrDescr, assembly_type ,ObjInt, RHandOI, LHandOI);
           xml->setKeyFrame(1,25.4);
           xml->setSemantics();
           xml->setCurrAction("assembly.mpg", "Putting one object over the other",2, 30, 210, 40, 220);
           //xml->setCameraSensor("Xtion", 0, 1175);
           xml->setObjects(2);
           xml->setInstructor();
           xml->setGraspingState();
           xml->setCLosingData(nf);

           xmlparser *xmlinput = new xmlparser("/home/gpiperagkas/SARAFun_kf/kf0.xml");
           xmlinput->getSemantics();
           xmlinput->getCurrentActionpars();
           xmlinput->getObjects();
           xmlinput->getInstructor(); //to-do: fix hands with all parameters
           xmlinput->getGrasping();


           posob1.clear();
           quatob1.clear();
           inposl.clear();
           inqual.clear();
           inposr.clear();
           inquar.clear();
           CPl.clear();
           CFl.clear();
           CPr.clear();
           CFr.clear();
           RHandOI.clear();
           LHandOI.clear();
           //delete[] coeff_ob1;
           delete[] coeff_ob1;
           delete[] coeff_hand1;
           delete[] coeff_hand2;


           //sec->printOSEC();
//           viscap1 >> keyframe;
           nf++;
           ostringstream nnn;
           nnn << nf;

           if (nf<10)
           {
               keyframenm = "mask/mask_000000"+ nnn.str() + ".png";
           }
           else if (nf<100)
           {
               keyframenm = "mask/mask_00000"+ nnn.str() + ".png";
           }else
           {
               keyframenm = "mask/mask_0000"+ nnn.str() + ".png";
           }

           keyframe = imread(keyframenm,CV_LOAD_IMAGE_GRAYSCALE);

       }
       posob2.clear();
       quatob2.clear();
}



bool eventchain::loadFromFile(const std::string& full_filename, float *coeff, int num_parameters)
{
        //load parameters from bin file
        std::ifstream file(full_filename.c_str(), std::ios::in | std::ios::binary);
        if (!file.is_open()) {
          return false;
        }
        file.seekg(0, std::ios::beg);
        // Make sure this isn't a blank file (indicating no hands on the screen)
//        coeff.resize(num_parameters);
        file.read(reinterpret_cast<char*>(coeff),
          num_parameters * sizeof(coeff[0]));
        file.close();
//        if ((coeff[0] < EPSILON) && (coeff[1] < EPSILON) && (coeff[2] < EPSILON))
//           {
//             return false;
//            }
        return true;

}



void eventchain::extractVideo()
{
    cv::Mat inframe;
    cv::Mat outframe;

    int ni;

    VideoWriter outputVideo;

    cv::Mat pmarkersa;
    ni=0;
    std::string irgbfname;
    irgbfname = "mask/mask_0000000.png";
    inframe = imread(irgbfname, CV_LOAD_IMAGE_GRAYSCALE);
    outputVideo.open("/home/gpiperagkas/SARAFun_kf/00_outputvideo.avi",CV_FOURCC('M','J','P','G'), 10 ,Size(inframe.cols,inframe.rows) , true);

    if (!outputVideo.isOpened())
        cout << "ERROR VIDEO IS NOT OPENED FOR WRITING!" << endl;

    int *ifrg;
    ifrg = new int[inframe.rows*inframe.cols];


    while(!inframe.empty())
    {

        cv::Mat labelsa;
        cv::Mat markersa;
        cv::Mat mylabelsa;
        cv::Mat ifiltered;


        mylabelsa = cv::Mat::zeros(inframe.size(), CV_8UC1);

        ifiltered =cv::Mat::zeros(inframe.size(), CV_8UC3);
        //simple labels drawing

        for (int ii=0;ii<inframe.rows; ii++)
        {
            for (int jj=0;jj<inframe.cols;jj++)
            {
                ifrg[ii*inframe.cols+jj]=inframe.at<uchar>(ii,jj);
                if (ifrg[ii*inframe.cols + jj]==0) //background
                {
                    mylabelsa.at<uchar>(ii,jj) = 0;
                    ifiltered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(0,0,0);

                }else if (ifrg[ii*inframe.cols +jj]==1) //hand
                {
                    mylabelsa.at<uchar>(ii,jj) = 1;
                    ifiltered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(250,0,0);

                }else if (ifrg[ii*inframe.cols + jj] ==2) //pcb
                {
                    mylabelsa.at<uchar>(ii,jj) = 2;
                    ifiltered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(210,210,210);

                }else if (ifrg[ii*inframe.cols + jj]==3) //case
                 {
                    mylabelsa.at<uchar>(ii,jj) = 3;
                    ifiltered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(180,190,220);

                }else if (ifrg[ii*inframe.cols + jj]==4)
                {
                    mylabelsa.at<uchar>(ii,jj) = 4;
                    ifiltered.at<cv::Vec3b>(ii,jj) = cv::Vec3b(0,190,0);
                }
            }
        }



        markersa= mylabelsa;

            markersa.convertTo(markersa,CV_8UC1);

            //initialize tracker and run basic routines
            tracker * tracka = new tracker();
            if (ni==0)
            {
                pmarkersa = markersa;

            }
            tracka->findCentroids(markersa,pmarkersa);
            tracka->findClosestNeighbors(markersa,pmarkersa);


        labelsa = cv::Mat::zeros(markersa.size(), CV_8UC1);
       labelsa=mylabelsa;//tracka->newmarkers;


        double alpha=0.5;
        double beta;
        cv::Mat dstn = cv::Mat::zeros(inframe.size(), CV_8UC3);
        dstn=ifiltered; //3c

        //        beta = ( 1.0 - alpha );
//         addWeighted( inframe, alpha, ifiltered, beta, 0.0, dstn);

         for (unsigned int k=0;k< tracka->cn.size();k++)
         {
             std::ostringstream str;
            str << tracka->cn.at(k);
             //cout << "STR: " << str.str()  << " " << tracka->cn.at(k) << endl;
             cv::putText(dstn , str.str() , cv::Point(tracka->newcentrx.at(k),tracka->newcentry.at(k)), CV_FONT_HERSHEY_PLAIN,0.8, CV_RGB(250,0,0),1,8);

         }

        int startlinex;
        int startliney;
        int endlinex;
        int endliney;


        for (int i=0;i<sc;i++)
        {
            int indexi=-1;
            for (int k=0;k<tracka->nrlabelsn.size();k++){
                if (tracka->nrlabelsn.at(k)==i)
                    indexi=k;

            }
            if (indexi<0)
                continue;

            for (int j=0;j<sc;j++)
            {
                int indexj=-1;
                for (int k=0;k<tracka->nrlabelsn.size();k++){
                    if (tracka->nrlabelsn.at(k)==j)
                        indexj=k;

                }
                if (indexj<0)
                    continue;
                if ((osec[ni][i*sc+j]==2)||(osec[ni][i*sc+j]==1))
                {


                    if ((indexi< tracka->newcentrx.size())&&(indexj< tracka->newcentry.size()))
                    {

                    startlinex = tracka->newcentrx.at(indexi);
                    startliney = tracka->newcentry.at(indexi);



                    endlinex = tracka->newcentrx.at(indexj);
                    endliney = tracka->newcentry.at(indexj);


                    line( dstn,cv::Point(startlinex,startliney),cv::Point(endlinex, endliney), Scalar( 0, 255, 0 ),1,8);
                    }

                 }
                if ((osec[ni][i*sc+j]>=3)&&(osec[ni][i*sc+j]<=10))
                {


                    if ((indexi< tracka->newcentrx.size())&&(indexj< tracka->newcentry.size()))
                    {

                    startlinex = tracka->newcentrx.at(indexi);
                    startliney = tracka->newcentry.at(indexi);



                    endlinex = tracka->newcentrx.at(indexj);
                    endliney = tracka->newcentry.at(indexj);


                    line( dstn,cv::Point(startlinex,startliney),cv::Point(endlinex, endliney), Scalar( 180, 0, 140 ),1,8);
                    }

                 }
            }
        }

        outframe = dstn;
         outputVideo << outframe;


         pmarkersa = mylabelsa;  //tracka->newmarkers;
         ni++;
         ostringstream nni2;
         nni2 << ni;

         if (ni<10)
         {
             irgbfname = "mask/mask_000000"+ nni2.str() + ".png";
         }else if (ni<100)
         {
             irgbfname = "mask/mask_00000"+ nni2.str() + ".png";
         }else
         {
             irgbfname = "mask/mask_0000"+ nni2.str() + ".png";
         }


         inframe = imread(irgbfname,CV_LOAD_IMAGE_GRAYSCALE);
    }

    delete[] ifrg;


}



void eventchain::printOSEC()
{

    for (i=0;i<nini;i++)
    {
        for (j=0;j<sc;j++)
        {
            for (k=0;k<sc;k++)
            {
                cout << osec[i][j*sc+k] << " ";
            }

        }
             cout << endl;
    }

    for (i=0;i<(nini-1);i++)
    {
        for (j=0;j<sc;j++)
        {
            for (k=0;k<sc;k++)
            {
                cout << dsec1[i][j*sc+k] << dsec2[i][j*sc+k] << " ";
            }
        }
        cout << endl;
    }

    for (unsigned int kk=0; kk<x.size(); kk++)
    {
        cout << x.at(kk) << " " << y.at(kk) << endl;
    }

}

void eventchain::printCSEC()
{

}

eventchain::~eventchain()
{
    for (i=0;i<nini;i++)
        delete osec[i];
    delete[] osec;
    for (i=0;i<(nini-1);i++)
        delete dsec1[i];
    delete[] dsec1;
    for (i=0;i<(nini-1);i++)
        delete dsec2[i];
    delete[] dsec2;
    for (i=0;i<(sc);i++)
        delete eigenval[i];
    delete[] eigenval;

}
