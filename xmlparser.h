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

#ifndef XMLPARSER_H
#define XMLPARSER_H
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <sstream>
#include <vector>
#include "pugixml-1.7/src/pugixml.hpp"
#include "pugixml-1.7/src/pugiconfig.hpp"

class xmlparser
{

private:
    int nkf;
    int i,j;
    pugi::xml_document doc;
    pugi::xml_node keyframe;
    pugi::xml_node semantics;
    pugi::xml_node instructorDescrNode;
    pugi::xml_node assemblyTypeNode;
    pugi::xml_node objectInterNode;
    pugi::xml_node RhandObjInterNode;
    pugi::xml_node LhandObjInterNode;
    pugi::xml_node currentAction;
    pugi::xml_node description;
    pugi::xml_node involvedObjs;
    pugi::xml_node visfeed;
    pugi::xml_node sensor;
    pugi::xml_node framerange;
    pugi::xml_node objects;
    pugi::xml_node objectt;
    pugi::xml_node meshfile;
    pugi::xml_node pose;
    pugi::xml_node pos1;
    pugi::xml_node pos2;
    pugi::xml_node quat1;
    pugi::xml_node quat2;
    pugi::xml_node deform;
    pugi::xml_node instr;
    pugi::xml_node lhand;
    pugi::xml_node rhand;
    pugi::xml_node handmeshes;
    pugi::xml_node meshfilel1;
    pugi::xml_node meshfilel2;
    pugi::xml_node meshfiler1;
    pugi::xml_node meshfiler2;
    pugi::xml_node PoseState;
    pugi::xml_node instrposl;
    pugi::xml_node instrposr;
    pugi::xml_node instrquatl;
    pugi::xml_node instrquatr;
    pugi::xml_node lhandmodel;
    pugi::xml_node rhandmodel;
    pugi::xml_node lhandparams;
    pugi::xml_node rhandparams;
    pugi::xml_node grstate;
    pugi::xml_node graspl;
    pugi::xml_node graspr;
    pugi::xml_node grobjl;
    pugi::xml_node grobjr;
    pugi::xml_node conpointsl;
    pugi::xml_node conpointsr;
    pugi::xml_node conpoint;
    pugi::xml_node conforcsl;
    pugi::xml_node conforcsr;
    pugi::xml_node conforce;
    std::vector<double> posobj1;
    std::vector<double> quatobj1;
    std::vector<double> posobj2;
    std::vector<double> quatobj2;
    std::vector<double> instposl;
    std::vector<double> instposr;
    std::vector<double> instquatl;
    std::vector<double> instquatr;
    std::vector<double> ConPointl;
    std::vector<double> ConForcel;
    std::vector<double> ConPointr;
    std::vector<double> ConForcer;
    std::vector<double> insthandlall;
    std::vector<double> insthandrall;
    //semantics
    std::string InstructorDescription;
    int AssemblyType;
    int ObjectsInteraction;
    std::vector<int> RightHandObjectInteraction;
    std::vector<int> LeftHandObjectInteraction;

    //start for xml reading
    std::string xmlname;
    pugi::xml_document indoc;
    pugi::xml_parse_result result;
    pugi::xml_node xmlkeyframe;
    pugi::xml_node xmlcurract;
    pugi::xml_node xmlsem;
public:
    std::string xmlkfid;
    int xmlkfidx;
    double xmlkft;
    std::string xmlkfxmlns;
    std::string xmlkfxsischemaloc;
    std::string xmlcaid;
    std::string xmlcadescr;
    std::string xmlobid1;
    std::string xmlobid2;
    std::string xmlcamid1;
    std::string xmlcamid2;
    int xmlcam1idxF;
    int xmlcam1idxL;
    int xmlcam2idxF;
    int xmlcam2idxL;
    std::string xmlcam1flist;
    std::string xmlcam2flist;
    std::string xmlob1name;
    std::string xmlob2name;
    std::string xmlob1meshf;
    std::string xmlob2meshf;
    std::vector<double> xmlob1pos;
    std::vector<double> xmlob1quat;
    std::vector<double> xmlob2pos;
    std::vector<double> xmlob2quat;
    std::string xmlob1deform;
    std::string xmlob2deform;
    std::string xmlinstrhandid;
    std::string xmlinstrhandname;
    std::string xmlinstrhandmeshp;
    std::string xmlinstrhandmeshd;
    std::vector<double> xmlinstrhandpos;
    std::vector<double> xmlinstrhandquat;
    std::string xmlinstrhandmodel;
    std::string xmlinstrhandparam;
    std::string xmlinstlhandid;
    std::string xmlinstlhandname;
    std::string xmlinstlhandmeshp;
    std::string xmlinstlhandmeshd;
    std::vector<double> xmlinstlhandpos;
    std::vector<double> xmlinstlhandquat;
    std::string xmlinstlhandmodel;
    std::string xmlinstlhandparam;
    std::string xmlgrasplid;
    std::string xmlgrasplobid;
    std::vector<double> xmlgrasplcp;
    std::vector<double> xmlgrasplcf;
    std::string xmlgrasprid;
    std::string xmlgrasprobid;
    std::vector<double> xmlgrasprcp;
    std::vector<double> xmlgrasprcf;
    std::vector<std::string> xmlsemrhandoi;
    std::vector<std::string> xmlsemlhandoi;
    std::string xmlsemInstrDescr;
    std::string xmlsemAssType;
    std::string xmlsemObjInter;



    xmlparser(std::string xmlinfile);
    xmlparser(int keyframenr, std::string videoname, std::string xmlfname, std::vector<double> posob1, std::vector<double> quatob1,
              std::vector<double> posob2, std::vector<double> quatob2, std::vector<double> inposl, std::vector<double> inqual,
              std::vector<double> inposr, std::vector<double> inquar, std::vector<double> CPl, std::vector<double> CFl,
              std::vector<double> CPr, std::vector<double> CFr, float *coeff_lhand, float *coeff_rhand, std::string instrdescr,
              int asstype, int objinter, std::vector<int> rhandoi, std::vector<int> lhandoi);
    void getSemantics();
    void getCurrentActionpars();
    void getObjects();
    void getInstructor();
    void getGrasping();
    void setKeyFrame(int idx, double timestamp);
    void setSemantics();
    void setCurrAction(std::string curAcid, std::string descr, int nrsensors, int idx1F, int idx1L, int idx2F, int idx2L);
    void setInvObjects(int nrobj);
    void setVisualFeedback(int nrsensors, int idx1F, int idx1L, int idx2F, int idx2L);
    void setObjValidAttributeType();
    void setCameraSensor(std::string camname, int idxF, int idxL);
    void setCamValidAttributeType();
    void setFrameRange(std::string camname, int idxF, int idxL);
    void setObjects(int nrobjects);
    void setObject(int oi);
    void setMeshFile(std::string mesh);
    void setPoseState(int il);
    void setPosition(int iil);
    void setQuaternion(int iil);
    void setDeformation();
    void setInstructor();
    void setHand(int hi);
    void setHandMeshes(int hi);
    void setHandModel(int hi);
    void setHandParameters(int hi);
    void setGraspingState();
    void setGrasp(int gi);
    void setContactPoints(int gi);
    void setContactPoint(int gi,int iii);
    void setContactForces(int gi);
    void setContactForce(int gi, int iii);
    void setCLosingData(int nrfr);
    ~xmlparser();


};

#endif // XMLPARSER_H
