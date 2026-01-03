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


#include "imagergb.h"

using namespace std;
using namespace cv;
using namespace Eigen;

#define THREL 0.0001
#define THRANGLE 0.75
#define THRANMJ_DET 0.90
#define THRANMD_DET 0.90
#define THRANMN_DET 0.90
#define THRANMJ 0.96
#define THRANMD 0.96
#define THRANMN 0.96
#define THRT -0.8

imagergb::imagergb(const string pathim[], int imn)
{

    Mat img;
    path = pathim[imn];
    img = imread(path, CV_LOAD_IMAGE_COLOR);

    sx=img.rows;
    sy=img.cols;
    sc=15;

    r= new int[sx*sy];
    g= new int[sx*sy];
    b= new int[sx*sy];

    for (i=0;i<sx;i++)
        for (j=0;j<sy;j++)
        {
            Vec3b clr = img.at<Vec3b>(i,j);
            r[i*sy+j]=int(clr.val[2]);
            g[i*sy+j]=int(clr.val[1]);
            b[i*sy+j]=int(clr.val[0]);
        }


    listhorizontal = new int[sx*sc];

    listvertical = new int[sy*sc];

    horlines = new int[sx*sc];

    vertlines = new int[sy*sc];

    ct = new int[sc*sc];

    normct = new double[sc*sc];

    co = new int[sc*sc];

    normco = new double[sc*sc];


    psize = new int[5]; //for four segments -> 0:white, 1:red, 2:green , 3:blue
}


//SIMPLE ANALYSIS USING MASKS
imagergb::imagergb(cv::Mat frame, int imn, cv::Mat labels)
{
    sx=labels.rows;
    sy=labels.cols;
    sc=15;

    feature=0;

    listhorizontal = new int[sx*sc];

    listvertical = new int[sy*sc];


    lbls=new int[sx*sy];

    for (i=0;i<sx;i++)
    {
        for (j=0;j<sy;j++)
        {

            lbls[i*sy+j] = labels.at<uchar>(i,j);
            if (lbls[i*sy+j]>14)
                lbls[i*sy+j]=0;

        }
    }

    cv::Mat output = cv::Mat::zeros(labels.size(), CV_8UC1);

    horlines = new int[sx*sc];

    vertlines = new int[sy*sc];

    ct = new int[sc*sc];

    normct = new double[sc*sc];

    co = new int[sc*sc];

    normco = new double[sc*sc];

    psize = new int[11]; //
}

//CASE OF TWO OBJECTS, ASSEMBLY MODELLING WITH ELLIPSOIDS
imagergb::imagergb(cv::Mat frame, int imn, cv::Mat labels, Matrix3d q1,
         Matrix3d q2, Vector3d c1, Vector3d c2)

{

    feature =1;
    im=imn;
    for (int ii=0;ii<3;ii++)
    {
        C1(ii)=c1(ii);
        C2(ii)=c2(ii);
        for (int jj=0;jj<3;jj++)
        {
            Q1(ii,jj)=q1(ii,jj);
            Q2(ii,jj)=q2(ii,jj);
        }
    }


    sx=labels.rows;
    sy=labels.cols;
    sc=15;



    listhorizontal = new int[sx*sc];

    listvertical = new int[sy*sc];


    lbls=new int[sx*sy];

    for (i=0;i<sx;i++)
    {
        for (j=0;j<sy;j++)
        {

            lbls[i*sy+j] = labels.at<uchar>(i,j);
            if (lbls[i*sy+j]>14)
                lbls[i*sy+j]=0;

        }
    }

    cv::Mat output = cv::Mat::zeros(labels.size(), CV_8UC1);

    horlines = new int[sx*sc];

    vertlines = new int[sy*sc];

    ct = new int[sc*sc];

    normct = new double[sc*sc];

    co = new int[sc*sc];

    normco = new double[sc*sc];

    normcel = new double[sc*sc];

    psize = new int[11]; //

    for (i=0;i<3;i++)
    {
        parstate.push_back(-1);
        finstate.push_back(-1);
    }

}

void imagergb::scanSegmented() // use labels of connectedComponents for spatial relations
{

    int before;
    int skipflag;
    int skipgap=2;
    int trackind;


//backgound =0;
    //initialize pixels size counter
    for (i=0;i<11;i++)
        psize[i]=0;

    //initialize nr of lines
    for (i=0;i<sx;i++)
        for (j=0;j<sc;j++)
            horlines[i*sc+j]=0;


    for (i=0;i<sy;i++)
        for (j=0;j<sc;j++)
            vertlines[i*sc+j]=0;


    //initialize vectors to background
    for (i=0;i<sx;i++)
        for (j=0;j<sc;j++)
             listhorizontal[i*sc+j]=0;

    for (i=0;i<sy;i++)
        for (j=0;j<sc;j++)
            listvertical[i*sc+j]=0;

    for (i=0; i<sy;i++) //scan vertical based on labels
    {
        before=lbls[i*sx+0];
        listvertical[i*sc+0]=0;
        trackind=1;
        for (j=1;j<sx;j++)
        {

            vertlines[i*sc+trackind]++;
            psize[lbls[j*sy+i]]++;
            skipflag=0;

            if (lbls[j*sy+i]==0)
            {
                for (int y=j;y<(j+skipgap);y++)
                {
//                    if ((j+skipgap)>=(sx-2))
//                        break;

                    if ((y<(sx-1))&&(j>0))
                    {
                       if ((lbls[y*sy+i]==0)&&(lbls[(j-1)*sy+i]!=0)&&(lbls[(y+1)*sy+i]!=0))
                         {
                          skipflag=1;
                         }
                         else
                        {
                          skipflag=0;
                        }
                    }
                 }
            }
            if (skipflag==1)
                continue;


            if ((lbls[j*sy+i]!=before)&&(i!=lbls[j*sy+i]))
            {
                listvertical[i*sc + trackind]=lbls[j*sy+i];
                trackind++;
            }

            before = lbls[j*sy+i];
        }

    }


    for (i=0;i<sx;i++)  //scan horizontal based on labels
    {
        before=lbls[i*sy+0];
        listhorizontal[i*sc+0]=0;
        trackind=1;
        for (j=1;j<sy;j++)
        {
            horlines[i*sc+trackind]++;
            psize[lbls[i*sy+j]]++;

            skipflag=0;

            if (lbls[i*sy+j]==0)
            {
                for (int y=j;y<(j+skipgap);y++)
                {
//                    if ((j+skipgap+1)>=(sy-2))
//                        break;

                    if ((y<(sy-1))&&(j>0))
                    {
                         if ((lbls[i*sy+y]==0)&&(lbls[i*sy+(j-1)]!=0)&&(lbls[i*sy+(y+1)]!=0))
                         {
                          skipflag=1;
                          }
                         else
                         {
                          skipflag=0;
                         }
                    }
                }
            }
            if (skipflag==1)
                continue;

            if ((lbls[i*sy+j]!=before)&&(i!=lbls[i*sy+j]))
            {
                listhorizontal[i*sc+trackind]=lbls[i*sy+j];
                trackind++;
            }

            before = lbls[i*sy+j];
        }

    }

}

void imagergb::setSegRelations()
{
    //set the relation matrices for touching and overlapping based on hints counters
    const int changes=13;
    int vlist[changes];
    int hlist[changes];
    int flag1,flag2;
    int indx,indy;
    int iii;

    try{

    for (i=0;i<changes;i++)
    {
        hlist[i]=0;
        vlist[i]=0;
    }

    for (i=0;i<sc;i++)
        for (j=0;j<sc;j++)
            ct[i*sc+j]=0; //initialize touching hints counter

    for (i=0;i<sc;i++)
        for (j=0;j<sc;j++)
            co[i*sc+j]=0; //initialize overlapping hints counter

    for (i=0;i<sx;i++)
    {
        for (j=0;j<changes;j++)
            hlist[j]=0;

        hlist[0]=listhorizontal[i*sc+0];
        for (j=1;j<changes;j++)
        {
            if (listhorizontal[i*sc+j]!=hlist[j-1]) // touching for horizontal
            {
                hlist[j]=listhorizontal[i*sc+j];
                indx=listhorizontal[i*sc+(j-1)];
                indy=listhorizontal[i*sc+j];
                ct[indx*sc+indy]=ct[indx*sc+indy] + 1;

            }
        }


        //now overlapping for horizontal


        for (j=0;j<(changes-1);j++)
        {
            flag1=hlist[j];
            for (int l=j+1;l<changes;l++)
            {
                if (hlist[l]!=hlist[j])
                {
                    for (int k=l+1;k<changes;k++)
                    {
                        if (hlist[k]==flag1)
                        {
                            indx=hlist[j];
                            indy=hlist[l];
                            if (indx!=indy)
                                co[indx*sc+indy]=co[indx*sc+indy]+1;
                        }
                    }
                }
            }
        }



//        flag1=hlist[0];
//        flag2=hlist[1]; //set for two overlapping maximum

//        for (j=2;j<changes;j++) //2 11
//        {
//            if ((hlist[j]==flag2))//&&(j>2))
//            {
//                indx=hlist[j-1];
//                indy=hlist[j];
//                co[indx*sc+indy]=co[indx*sc+indy]+1;

//            }else if ((hlist[j]==flag1)&&(j>3))
//            {
//                indx=hlist[j-2];
//                indy=hlist[j];
//                co[indx*sc+indy]=co[indx*sc+indy]+1;

//                if (hlist[j-1]==flag2)
//                {
//                    indx=hlist[j-1];
//                    indy=hlist[j];
//                    co[indx*sc+indy]=co[indx*sc+indy]+1;

//                }
//            }else if (hlist[j]==flag1)
//            {
//                indx=hlist[j-1];
//                indy=hlist[j];
//                co[indx*sc+indy]=co[indx*sc+indy]+1;

//            }

//        }
    }

    //now count for vertical

    for (iii=0;iii < sy;iii++)
    {

        for (j=0;j<changes;j++)
            vlist[j]=0;

        vlist[0] = listvertical[iii*sc+0];
        for (j=1;j<11;j++)
        {

            if (listvertical[iii*sc+j]!=vlist[j-1]) // touching for vertical
            {

                vlist[j]=listvertical[iii*sc+j];

                indx=listvertical[iii*sc+(j-1)];

                indy=listvertical[iii*sc+j];

                ct[indx*sc+indy]=ct[indx*sc+indy] + 1;
            }

        }

        //now overlapping for vertical


        for (j=0;j<(changes-1);j++)
        {
            flag1=vlist[j];
            for (int l=j+1;l<changes;l++)
            {
                if (vlist[l]!=vlist[j])
                {
                    for (int k=l+1;k<changes;k++)
                    {
                        if (vlist[k]==flag1)
                        {
                            indx=vlist[j];
                            indy=vlist[l];
                            if (indx!=indy)
                                co[indx*sc+indy]=co[indx*sc+indy]+1;
                        }
                    }
                }
            }
        }

//        flag1=vlist[0];
//        flag2=vlist[1]; //set for two overlapping maximum

//        for (j=2;j<changes;j++)
//        {
//            if ((vlist[j]==flag2)&&(j>2))
//            {
//                indx=vlist[j-1];

//                indy=vlist[j];

//                co[indx*sc+indy]=co[indx*sc+indy]+1;


//            }else if ((vlist[j]==flag1)&&(j>3))
//            {

//                indx=vlist[j-2];
//                indy=vlist[j];

//                co[indx*sc+indy]=co[indx*sc+indy]+1;

//                if (vlist[j-1]==flag2)
//                {

//                    indx=vlist[j-1];

//                    indy=vlist[j];

//                    co[indx*sc+indy]=co[indx*sc+indy]+1;

//                }
//            }else if (vlist[j]==flag1)
//            {

//                indx=vlist[j-1];
//                indy=vlist[j];
//                co[indx*sc+indy]=co[indx*sc+indy]+1;

//            }

//        }

    }


    }catch (const std::exception& ww) {
        std::cerr << "exception caught: " << ww.what() << std::endl;
    }

}

void imagergb::calcNormHints() //calculate normalized hint counters for overlapping and touching
{
    double min;
    double marg,marg2;
    bool touching=false;
    const int changes=13;
    //initialize normco normct
    for (i=0;i<sc;i++)
    {
        for (j=0;j<sc;j++)
        {
            normct[i*sc+j]=0;
            normco[i*sc+j]=0;
            if (feature==1)
            {
                normcel[i*sc+j]=-1;
            }
        }
    }
    feature=1;
    //check ellipsoids for labels 2 & 3
    if (feature==1)
    {
        this->ElliMargin(1);
        this->ElliMargin(2);
        truemarg = std::sqrt(((this->xstarr1(0)-this->xstarr2(0))*(this->xstarr1(0)-this->xstarr2(0)))+
                             ((this->xstarr1(1)-this->xstarr2(1))*(this->xstarr1(1)-this->xstarr2(1)))+
                             ((this->xstarr1(2)-this->xstarr2(2))*(this->xstarr1(2)-this->xstarr2(2))));
        cout << "FRAME "<<im<< " MARGIN= "<< marr << endl;
        cout <<"TRUE MARGIN= "<<truemarg <<endl;

        this->ElliAxon();

        if (marr>=THRT)
            touching=false;
//        else if ((marr<=THRT)&&(truemarg<THREL*2*semiaxon))
//            touching=false;
        else if ((marr<THRT))//&&(truemarg>=THREL*2*semiaxon))
            touching=true;



        if (!touching)
        {
            normcel[2*sc+3] = 0;
            normcel[3*sc+2] = 0;
        }
        else if (touching)
        {
            normcel[2*sc+3] = 1;
            normcel[3*sc+2] = 1;
        }
    }

    for (i=0;i<sc;i++)
    {
        for (j=0;j<sc;j++)
        {
            if ((i<changes)&&(j<changes))
            {

                if (psize[i]<psize[j])
                    min=psize[i];
                else
                    min=psize[j];

                if (min>0)
                {
                    normct[i*sc+j] = double(ct[i*sc+j])/min;
                    normco[i*sc+j] = double(co[i*sc+j])/min;
                }else{
                    normct[i*sc+j] = 0;
                    normco[i*sc+j] = 0;
                }

            }
        }
    }
}




//FIND MARGIN BETWEEN TWO ELLIPSOIDS, BASED ON E. RIMON & S. BOYD
// if positive, ellipsoids do not touch
// if zero, they touch at one point
// if negative they touch normally

void imagergb::ElliMargin(int imar)
{
    MatrixXd C;
    MatrixXd c;
    VectorXd b;
    MatrixXd I;
    MatrixXd bbt;
    MatrixXd eigall;
    VectorXd xstar;
    MatrixXd xx;
    complex<double> ll;
    double mineig;
    double checkval;
    double mar;



    if (imar==1)
    {
        checkval = ((C1 - C2).transpose()*Q2*(C1 - C2)) - 1;

        C=(Q2.sqrt()).inverse()*Q1*(Q2.sqrt()).inverse();
        c=Q2.sqrt()*(C1-C2);
    }else if (imar==2)
    {
        checkval = ((C2 - C1).transpose()*Q1*(C2 - C1)) - 1;

        C=(Q1.sqrt()).inverse()*Q2*(Q1.sqrt()).inverse();
        c=Q1.sqrt()*(C2-C1);
    }
    b = C*c;
   // b=(Q2.sqrt()).inverse()*Q1*(C1-C2);


    bbt = -(b*b.transpose());
    I = MatrixXd::Identity(3,3);

    eigall.resize(6,6);
    for (int ii=0;ii<6;ii++)
    {
        for (int jj=0;jj<6;jj++)
        {
            if ((ii<3)&&(jj<3))
                eigall(ii,jj) = C(ii,jj);
            else if ((ii<3)&&(jj>=3))
                eigall(ii,jj) = -I(ii,(jj-3));
            else if ((ii>=3)&&(jj<3))
                eigall(ii,jj)=bbt((ii-3),jj);
            else if ((ii>=3)&&(jj>=3))
                eigall(ii,jj)= C((ii-3),(jj-3));
        }
    }
    mineig = 10000000;
    EigenSolver<MatrixXd> l(eigall);
    for (i=0;i<6;i++)
    {

        ll=l.eigenvalues()[i];
        if (ll.imag()==0)
        {
            if (ll.real()<mineig)
                mineig=ll.real();
        }
    }
    xx= C-(mineig*I);
    xstar = xx.inverse()*b;
    if (imar==1)
    {
        xstarr1 = (Q2.sqrt()).inverse()*xstar + C2;
        marr = (((xstarr1 - C1).transpose()*Q1*(xstarr1 - C1)) -1);
        xstarrr = xstarr1 - C1;
    }else if (imar==2)
    {
        xstarr2 = (Q1.sqrt()).inverse()*xstar + C1;
        //marr = (((xstarr2 - C2).transpose()*Q2*(xstarr2 - C2)) -1);
        xstarrr = xstarr2 - C2;
    }
}

void imagergb::ElliAxon()
{
    std::vector<double> axL1;
    std::vector<double> axL2;
    complex<double> ll;
    complex<double> ll1;
    complex<double> ll2;
    double max1, max2;

    EigenSolver<MatrixXd> es1(Q1);
    EigenSolver<MatrixXd> es2(Q2);
    max1 = 0.000000000001;
    max2 = 0.000000000001;
    for (int i=0;i<3;i++)
    {
        axL1.push_back(0);
        axL2.push_back(0);
        ll=es1.eigenvalues()[i];
        axL1.at(i) = 1/std::sqrt(std::abs(ll));
        if (axL1.at(i)>max1)
            max1= axL1.at(i);

        ll=es2.eigenvalues()[i];
        axL2.at(i) = 1/std::sqrt(std::abs(ll));
        if (axL2.at(i)>max2)
            max2= axL2.at(i);
     }

     semiaxon = (max1+max2)/2;

}

//RETURN value IF THE TWO ELLIPSOIDS ARE PARALLEL. TAKE DOT PRODUCT OF
// EIGENVECTORS
//mode -1: detect which pairs of axes are parallel at last frame
// mode 0: find parallelism according to last frame
void imagergb::ElliAngles(int mode, std::vector<int> finst)
{
    Matrix3cd ax1;
    Matrix3d ax1r;
    Matrix3cd ax2;
    Matrix3d ax2r;
    std::vector<double> axL1;
    std::vector<double> axL2;
    complex<double> ll;
    complex<double> ll1;
    complex<double> ll2;
    double max1, max2, med1, med2, min1, min2;
    int maxind1, maxind2;
    int medind1, medind2;
    int minind1, minind2;
    double val1, val2, dot;
    bool hlp;
    int chk;

    EigenSolver<MatrixXd> es1(Q1);
    EigenSolver<MatrixXd> es2(Q2);

    for (int i=0;i<3;i++)
    {
        axL1.push_back(0);
        axL2.push_back(0);
    }
    ll=es1.eigenvalues()[0];
    ll1=es1.eigenvalues()[1];
    ll2=es1.eigenvalues()[2];
    axL1.at(0) = 1/std::sqrt(std::abs(ll));
    axL1.at(1) = 1/std::sqrt(std::abs(ll1));
    axL1.at(2) = 1/std::sqrt(std::abs(ll2));
    ll=es2.eigenvalues()[0];
    ll1=es2.eigenvalues()[1];
    ll2=es2.eigenvalues()[2];
    axL2.at(0) = 1/std::sqrt(std::abs(ll));
    axL2.at(1) = 1/std::sqrt(std::abs(ll1));
    axL2.at(2) = 1/std::sqrt(std::abs(ll2));

    max1=0.00000000001;
    max2=0.00000000001;
    for (int i=0;i<3;i++)
    {
        if (axL1.at(i)>max1)
        {
            max1= axL1.at(i);
            maxind1=i;
        }
        if (axL2.at(i)>max2)
        {
            max2= axL2.at(i);
            maxind2=i;
        }

    }

    if (((axL1.at(0)>axL1.at(1))&&(axL1.at(0)<axL1.at(2)))||
            ((axL1.at(0)>axL1.at(2))&&(axL1.at(0)<axL1.at(1))))
    {
        medind1=0;
        med1 = axL1.at(0);
    }else if (((axL1.at(1)>axL1.at(0))&&(axL1.at(1)<axL1.at(2)))||
              ((axL1.at(1)>axL1.at(2))&&(axL1.at(1)<axL1.at(0))))
    {
        medind1=1;
        med1 = axL1.at(1);
    }else if (((axL1.at(2)>axL1.at(0))&&(axL1.at(2)<axL1.at(1)))||
              ((axL1.at(2)>axL1.at(1))&&(axL1.at(2)<axL1.at(0))))
    {
        medind1=2;
        med1 = axL1.at(2);
    }

    if (((axL2.at(0)>axL2.at(1))&&(axL2.at(0)<axL2.at(2)))||
            ((axL2.at(0)>axL2.at(2))&&(axL2.at(0)<axL2.at(1))))
    {
        medind2=0;
        med2 = axL2.at(0);
    }else if (((axL2.at(1)>axL2.at(0))&&(axL2.at(1)<axL2.at(2)))||
              ((axL2.at(1)>axL2.at(2))&&(axL2.at(1)<axL2.at(0))))
    {
        medind2=1;
        med2 = axL2.at(1);
    }else if (((axL2.at(2)>axL2.at(0))&&(axL2.at(2)<axL2.at(1)))||
              ((axL2.at(2)>axL2.at(1))&&(axL2.at(2)<axL2.at(0))))
    {
        medind2=2;
        med2 = axL2.at(2);
    }

    min1 = 10000000;
    min2 = 10000000;
    for (int i=0;i<3;i++)
    {
        if (axL1.at(i)<min1)
        {
            min1= axL1.at(i);
            minind1=i;
        }
        if (axL2.at(i)<min2)
        {
            min2= axL2.at(i);
            minind2=i;
        }

    }

    ax1 = es1.eigenvectors();
    ax1r = ax1.real();
    ax2 = es2.eigenvectors();
    ax2r = ax2.real();

    if (mode==-1) // define final states of parallelism
    {
        hlp=false;
        // maj || maj ========= state 0
        val1 = std::sqrt((ax1r(0,maxind1)*ax1r(0,maxind1)) +
                         (ax1r(1,maxind1)*ax1r(1,maxind1)) +
                         (ax1r(2,maxind1)*ax1r(2,maxind1)));

        val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                         (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                         (ax2r(2,maxind2)*ax2r(2,maxind2)));

        dot = (ax1r(0,maxind1)*ax2r(0,maxind2)) + (ax1r(1,maxind1)*ax2r(1,maxind2)) + (ax1r(2,maxind1)*ax2r(2,maxind2));
        cout << "COS_MAJ_MAJ=" << (std::abs(dot/(val1*val2))) << endl;
        if (std::abs(dot/(val1*val2)) > THRANMJ_DET)
        {
            finstate.at(0)=0;
            hlp=true;
        }


        // maj || med ============ state 1
        val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                         (ax2r(1,medind2)*ax2r(1,medind2)) +
                         (ax2r(2,medind2)*ax2r(2,medind2)));

        dot = (ax1r(0,maxind1)*ax2r(0,medind2)) + (ax1r(1,maxind1)*ax2r(1,medind2)) + (ax1r(2,maxind1)*ax2r(2,medind2));

        if (std::abs(dot/(val1*val2)) > THRANMJ_DET)
        {
            finstate.at(0)=1;
            hlp=true;
        }

        // maj || min ============ state 2
        val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                         (ax2r(1,minind2)*ax2r(1,minind2)) +
                         (ax2r(2,minind2)*ax2r(2,minind2)));

        dot = (ax1r(0,maxind1)*ax2r(0,minind2)) + (ax1r(1,maxind1)*ax2r(1,minind2)) + (ax1r(2,maxind1)*ax2r(2,minind2));

        if (std::abs(dot/(val1*val2)) > THRANMJ_DET)
        {
            finstate.at(0)=2;
            hlp=true;
        }

        if (!hlp)
            finstate.at(0) = -1;

        hlp=false;

        // med || maj ========= state 3
        val1 = std::sqrt((ax1r(0,medind1)*ax1r(0,medind1)) +
                         (ax1r(1,medind1)*ax1r(1,medind1)) +
                         (ax1r(2,medind1)*ax1r(2,medind1)));

        val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                         (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                         (ax2r(2,maxind2)*ax2r(2,maxind2)));

        dot = (ax1r(0,medind1)*ax2r(0,maxind2)) + (ax1r(1,medind1)*ax2r(1,maxind2)) + (ax1r(2,medind1)*ax2r(2,maxind2));

        if (std::abs(dot/(val1*val2)) > THRANMD_DET)
        {
            finstate.at(1)=3;
            hlp=true;
        }


        // med || med ============ state 4
        val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                         (ax2r(1,medind2)*ax2r(1,medind2)) +
                         (ax2r(2,medind2)*ax2r(2,medind2)));

        dot = (ax1r(0,medind1)*ax2r(0,medind2)) + (ax1r(1,medind1)*ax2r(1,medind2)) + (ax1r(2,medind1)*ax2r(2,medind2));
        cout << "COS_MED_MED=" << (std::abs(dot/(val1*val2))) << endl;

        if (std::abs(dot/(val1*val2)) > THRANMD_DET)
        {
            finstate.at(1)=4;
            hlp=true;
        }

        // med || min ============ state 5
        val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                         (ax2r(1,minind2)*ax2r(1,minind2)) +
                         (ax2r(2,minind2)*ax2r(2,minind2)));

        dot = (ax1r(0,medind1)*ax2r(0,minind2)) + (ax1r(1,medind1)*ax2r(1,minind2)) + (ax1r(2,medind1)*ax2r(2,minind2));

        if (std::abs(dot/(val1*val2)) > THRANMD_DET)
        {
            finstate.at(1)=5;
            hlp=true;
        }

        if (!hlp)
            finstate.at(1) = -1;

        hlp=false;

        // min || maj ========= state 6
        val1 = std::sqrt((ax1r(0,minind1)*ax1r(0,minind1)) +
                         (ax1r(1,minind1)*ax1r(1,minind1)) +
                         (ax1r(2,minind1)*ax1r(2,minind1)));

        val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                         (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                         (ax2r(2,maxind2)*ax2r(2,maxind2)));

        dot = (ax1r(0,minind1)*ax2r(0,maxind2)) + (ax1r(1,minind1)*ax2r(1,maxind2)) + (ax1r(2,minind1)*ax2r(2,maxind2));

        if (std::abs(dot/(val1*val2)) > THRANMN_DET)
        {
            finstate.at(2)= 6;
            hlp=true;
        }


        // min || med ============ state 7
        val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                         (ax2r(1,medind2)*ax2r(1,medind2)) +
                         (ax2r(2,medind2)*ax2r(2,medind2)));

        dot = (ax1r(0,minind1)*ax2r(0,medind2)) + (ax1r(1,minind1)*ax2r(1,medind2)) + (ax1r(2,minind1)*ax2r(2,medind2));

        if (std::abs(dot/(val1*val2)) > THRANMN_DET)
        {
            finstate.at(2)=7;
            hlp=true;
        }

        // min || min ============ state 8
        val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                         (ax2r(1,minind2)*ax2r(1,minind2)) +
                         (ax2r(2,minind2)*ax2r(2,minind2)));

        dot = (ax1r(0,minind1)*ax2r(0,minind2)) + (ax1r(1,minind1)*ax2r(1,minind2)) + (ax1r(2,minind1)*ax2r(2,minind2));
        cout << "COS_MIN_MIN=" << (std::abs(dot/(val1*val2))) << endl;

        if (std::abs(dot/(val1*val2)) > THRANMN_DET)
        {
            finstate.at(2)=8;
            hlp=true;
        }

        if (!hlp)
            finstate.at(2) = -1;

    }else if (mode==0)
    {

            //check major axis
            chk = finst.at(0);
            val1 = std::sqrt((ax1r(0,maxind1)*ax1r(0,maxind1)) +
                             (ax1r(1,maxind1)*ax1r(1,maxind1)) +
                             (ax1r(2,maxind1)*ax1r(2,maxind1)));

            val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                            (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                            (ax2r(2,maxind2)*ax2r(2,maxind2)));

            dot = (ax1r(0,maxind1)*ax2r(0,maxind2)) + (ax1r(1,maxind1)*ax2r(1,maxind2)) + (ax1r(2,maxind1)*ax2r(2,maxind2));
            cout << "maj degrees: "<< (180/3.14)*std::acos(std::abs(dot/(val1*val2))) <<endl;
            switch (chk)
            {
                case 0:
                    val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                                    (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                                    (ax2r(2,maxind2)*ax2r(2,maxind2)));

                    dot = (ax1r(0,maxind1)*ax2r(0,maxind2)) + (ax1r(1,maxind1)*ax2r(1,maxind2)) + (ax1r(2,maxind1)*ax2r(2,maxind2));

                    if (std::abs(dot/(val1*val2)) > THRANMJ)
                      parstate.at(0)=0;
                    break;
                case 1:
                    val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                                     (ax2r(1,medind2)*ax2r(1,medind2)) +
                                     (ax2r(2,medind2)*ax2r(2,medind2)));

                    dot = (ax1r(0,maxind1)*ax2r(0,medind2)) + (ax1r(1,maxind1)*ax2r(1,medind2)) + (ax1r(2,maxind1)*ax2r(2,medind2));

                    if (std::abs(dot/(val1*val2)) > THRANMJ)
                        parstate.at(0)=1;
                    break;
                 case 2:
                     val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                                    (ax2r(1,minind2)*ax2r(1,minind2)) +
                                    (ax2r(2,minind2)*ax2r(2,minind2)));

                     dot = (ax1r(0,maxind1)*ax2r(0,minind2)) + (ax1r(1,maxind1)*ax2r(1,minind2)) + (ax1r(2,maxind1)*ax2r(2,minind2));

                     if (std::abs(dot/(val1*val2)) > THRANMJ)
                         parstate.at(0)=2;
                     break;
                 default:
                    parstate.at(0) = -1;
                    break;

            }
            //check medium axis
            chk = finst.at(1);
            val1 = std::sqrt((ax1r(0,medind1)*ax1r(0,medind1)) +
                             (ax1r(1,medind1)*ax1r(1,medind1)) +
                             (ax1r(2,medind1)*ax1r(2,medind1)));
            val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                            (ax2r(1,medind2)*ax2r(1,medind2)) +
                            (ax2r(2,medind2)*ax2r(2,medind2)));

            dot = (ax1r(0,medind1)*ax2r(0,medind2)) + (ax1r(1,medind1)*ax2r(1,medind2)) + (ax1r(2,medind1)*ax2r(2,medind2));
           cout << "med degrees: "<< (180/3.14)*std::acos(std::abs(dot/(val1*val2))) <<endl;
            switch (chk)
            {
                case 3:
                    val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                                    (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                                    (ax2r(2,maxind2)*ax2r(2,maxind2)));

                    dot = (ax1r(0,medind1)*ax2r(0,maxind2)) + (ax1r(1,medind1)*ax2r(1,maxind2)) + (ax1r(2,medind1)*ax2r(2,maxind2));

                    if (std::abs(dot/(val1*val2)) > THRANMD)
                        parstate.at(1)=3;
                    break;
                case 4:
                    val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                                    (ax2r(1,medind2)*ax2r(1,medind2)) +
                                    (ax2r(2,medind2)*ax2r(2,medind2)));

                    dot = (ax1r(0,medind1)*ax2r(0,medind2)) + (ax1r(1,medind1)*ax2r(1,medind2)) + (ax1r(2,medind1)*ax2r(2,medind2));

                    if (std::abs(dot/(val1*val2)) > THRANMD)
                        parstate.at(1)=4;
                    break;
                case 5:
                    val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                                    (ax2r(1,minind2)*ax2r(1,minind2)) +
                                    (ax2r(2,minind2)*ax2r(2,minind2)));

                    dot = (ax1r(0,medind1)*ax2r(0,minind2)) + (ax1r(1,medind1)*ax2r(1,minind2)) + (ax1r(2,medind1)*ax2r(2,minind2));

                    if (std::abs(dot/(val1*val2)) > THRANMD)
                        parstate.at(1)=5;
                    break;
                default:
                    parstate.at(1) = -1;
                    break;
             }

            //check minor axis
            chk = finst.at(2);
            val1 = std::sqrt((ax1r(0,minind1)*ax1r(0,minind1)) +
                             (ax1r(1,minind1)*ax1r(1,minind1)) +
                             (ax1r(2,minind1)*ax1r(2,minind1)));

            val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                            (ax2r(1,minind2)*ax2r(1,minind2)) +
                            (ax2r(2,minind2)*ax2r(2,minind2)));

            dot = (ax1r(0,minind1)*ax2r(0,minind2)) + (ax1r(1,minind1)*ax2r(1,minind2)) + (ax1r(2,minind1)*ax2r(2,minind2));

            cout << "min degrees: "<< (180/3.14)*std::acos(std::abs(dot/(val1*val2))) <<endl;

            switch (chk)
            {
                case 6:
                    val2 = std::sqrt((ax2r(0,maxind2)*ax2r(0,maxind2)) +
                                    (ax2r(1,maxind2)*ax2r(1,maxind2)) +
                                    (ax2r(2,maxind2)*ax2r(2,maxind2)));

                    dot = (ax1r(0,minind1)*ax2r(0,maxind2)) + (ax1r(1,minind1)*ax2r(1,maxind2)) + (ax1r(2,minind1)*ax2r(2,maxind2));

                    if (std::abs(dot/(val1*val2)) > THRANMN)
                        parstate.at(2)=6;
                    break;
                case 7:
                    val2 = std::sqrt((ax2r(0,medind2)*ax2r(0,medind2)) +
                                    (ax2r(1,medind2)*ax2r(1,medind2)) +
                                    (ax2r(2,medind2)*ax2r(2,medind2)));

                    dot = (ax1r(0,minind1)*ax2r(0,medind2)) + (ax1r(1,minind1)*ax2r(1,medind2)) + (ax1r(2,minind1)*ax2r(2,medind2));

                    if (std::abs(dot/(val1*val2)) > THRANMN)
                        parstate.at(2)=7;
                    break;
                case 8:
                    val2 = std::sqrt((ax2r(0,minind2)*ax2r(0,minind2)) +
                                    (ax2r(1,minind2)*ax2r(1,minind2)) +
                                    (ax2r(2,minind2)*ax2r(2,minind2)));

                    dot = (ax1r(0,minind1)*ax2r(0,minind2)) + (ax1r(1,minind1)*ax2r(1,minind2)) + (ax1r(2,minind1)*ax2r(2,minind2));

                    if (std::abs(dot/(val1*val2)) > THRANMN)
                        parstate.at(2)=8;
                    break;
                default:
                    parstate.at(2) = -1;
                    break;
             }


        }
    }





void imagergb::printhints()
{

    for (i=0;i<sc;i++)
    {
        for (j=0;j<sc;j++)
        {
            cout << normct[i*sc+j] << " ";
        }
        cout << endl;
    }
    cout << endl;

    cout << "normco" <<endl;
    for (i=0;i<sc;i++)
    {
        for (j=0;j<sc;j++)
        {
            cout << normco[i*sc+j] << " ";
        }
        cout << endl;
    }


}

imagergb::~imagergb()
{


        delete[] listhorizontal;

        delete[] listvertical;

        delete[] horlines;

        delete[] vertlines;

        delete[] ct;

        delete[] co;

        delete[] normct;

        delete[] normco;
        delete[] psize;


}


