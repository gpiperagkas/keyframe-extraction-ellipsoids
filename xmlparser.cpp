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

#include "xmlparser.h"

#define NUM_PARAMETERS 48 //for each hand, 48 parameters to be loaded

using namespace std;
//using namespace tinyxml;
using namespace pugi;

xmlparser::xmlparser(int keyframenr, string videoname, string xmlfname, vector<double> posob1, vector<double> quatob1,
                     vector<double> posob2, vector<double> quatob2, std::vector<double> inposl, std::vector<double> inqual,
                     std::vector<double> inposr, std::vector<double> inquar, std::vector<double> CPl, std::vector<double> CFl,
                     std::vector<double> CPr, std::vector<double> CFr, float *coeff_lhand, float *coeff_rhand, string instrdescr,
                     int asstype, int objinter, std::vector<int> rhandoi, std::vector<int> lhandoi)
{

    for (unsigned int ll=0;ll<posob1.size();ll++)
        posobj1.push_back(posob1.at(ll));

    for (unsigned int ll=0;ll<quatob1.size();ll++)
        quatobj1.push_back(quatob1.at(ll));

    for (unsigned int ll=0;ll<posob2.size();ll++)
        posobj2.push_back(posob2.at(ll));

    for (unsigned int ll=0;ll<quatob2.size();ll++)
        quatobj2.push_back(quatob2.at(ll));

    for (unsigned int ll=0;ll<inposl.size();ll++)
        instposl.push_back(inposl.at(ll));

    for (unsigned int ll=0;ll<inposr.size();ll++)
        instposr.push_back(inposr.at(ll));

    for (unsigned int ll=0;ll<inqual.size();ll++)
        instquatl.push_back(inqual.at(ll));

    for (unsigned int ll=0;ll<inquar.size();ll++)
        instquatr.push_back(inquar.at(ll));

    for (unsigned int ll=0;ll<CPl.size();ll++)
        ConPointl.push_back(CPl.at(ll));

    for (unsigned int ll=0;ll<CFl.size();ll++)
        ConForcel.push_back(CFl.at(ll));

    for (unsigned int ll=0;ll<CPr.size();ll++)
        ConPointr.push_back(CPr.at(ll));

    for (unsigned int ll=0;ll<CFr.size();ll++)
        ConForcer.push_back(CFr.at(ll));

    for (unsigned int ll=0;ll<NUM_PARAMETERS;ll++)
        insthandlall.push_back(coeff_lhand[ll]);

    for (unsigned int ll=0;ll<NUM_PARAMETERS;ll++)
        insthandrall.push_back(coeff_rhand[ll]);

    for (unsigned int ll=0;ll<rhandoi.size();ll++)
        RightHandObjectInteraction.push_back(rhandoi[ll]);

    for (unsigned int ll=0;ll<lhandoi.size();ll++)
        LeftHandObjectInteraction.push_back(lhandoi[ll]);

    InstructorDescription = instrdescr;
    AssemblyType = asstype;
    ObjectsInteraction = objinter;



    nkf = keyframenr;
    xmlname = xmlfname;

    xml_node dNode = doc.append_child(pugi::node_declaration);
    dNode.append_attribute("version")    = "1.0";
    dNode.append_attribute("encoding")   = "utf-8";
    dNode.append_attribute("standalone") = "yes";


}


xmlparser::xmlparser(std::string xmlinfile)
{

    result = indoc.load_file(xmlinfile.c_str());     //load_file(xmlinfile);


    xmlkeyframe = indoc.child("KeyFrame");


    xmlkfid = xmlkeyframe.attribute("id").value();
    xmlkfidx = xmlkeyframe.attribute("idx").as_int();
    xmlkft = xmlkeyframe.attribute("t").as_double();
    xmlkfxmlns = xmlkeyframe.attribute("xmlns").value();
    xmlkfxsischemaloc = xmlkeyframe.attribute("xsi:schemaLocation").value();

}



void xmlparser::getSemantics()
{
    xml_node descr;
    xml_node asstype;
    xml_node objinter;
    xml_node rhandoi;
    xml_node lhandoi;

    xmlsem = xmlkeyframe.child("Semantics");
    descr = xmlsem.child("InstructorDescription");
    xmlsemInstrDescr = descr.attribute("description").value();
    asstype = xmlsem.child("AssemblyType");
    xmlsemAssType = asstype.attribute("id").value();
    objinter = xmlsem.child("ObjectInteraction");
    xmlsemObjInter = objinter.attribute("id").value();
    rhandoi = xmlsem.child("RightHandObjectInteraction");
    xmlsemrhandoi.push_back(rhandoi.attribute("interaction").value());
    xmlsemrhandoi.push_back(rhandoi.attribute("AssemblyPart").value());
    lhandoi = xmlsem.child("LeftHandObjectInteraction");
    xmlsemlhandoi.push_back(lhandoi.attribute("interaction").value());
    xmlsemlhandoi.push_back(lhandoi.attribute("AssemblyPart").value());



}

void xmlparser::getCurrentActionpars()
{
       xml_node xmldescription;
       xml_node xmlinvobj;
       xml_node xmlobj1;
       xml_node xmlobj2;
       xml_node vis_feedback;
       xml_node cam_sensor;
       xml_node frame_range;

       xmlcurract = xmlkeyframe.child("CurrentAction");
       xmlcaid = xmlcurract.attribute("id").value();
       xmldescription = xmlcurract.child("Description");

       xmlcadescr = xmldescription.child_value();
       xmlinvobj = xmlcurract.child("InvolvedObjects");
       int ii=0;
       for (xmlobj1 = xmlinvobj.first_child(); xmlobj1; xmlobj1 = xmlobj1.next_sibling())
       {

           if (ii==0)
           {
            xmlobid1 = xmlobj1.attribute("id").value();
           }else
           {
             xmlobid2 = xmlobj1.attribute("id").value();
           }
           ii++;
        }

        vis_feedback = xmlcurract.child("VisualFeedback");
        ii=0;
        for (cam_sensor = vis_feedback.first_child(); cam_sensor; cam_sensor = cam_sensor.next_sibling())
        {
            if (ii==0)
            {
                xmlcamid1 = cam_sensor.attribute("id").value();
                frame_range = cam_sensor.child("FrameRange");
                xmlcam1idxF = frame_range.attribute("idxFirst").as_int();
                xmlcam1idxL = frame_range.attribute("idxLast").as_int();
                xmlcam1flist = frame_range.attribute("fileList").value();
            }else
            {
                xmlcamid2 = cam_sensor.attribute("id").value();
                frame_range = cam_sensor.child("FrameRange");
                xmlcam2idxF = frame_range.attribute("idxFirst").as_int();
                xmlcam2idxL = frame_range.attribute("idxLast").as_int();
                xmlcam2flist = frame_range.attribute("fileList").value();
            }

            ii++;
        }



}


void xmlparser::getObjects()
{
     xml_node xmlobjects;
     xml_node xmlobj;
     xml_node meshf;
     xml_node xmlpose;
     xml_node xmldeform;
     xml_node xmlposit;
     xml_node xmlquaternion;
    double temp;

    xmlobjects = xmlkeyframe.child("Objects");
    int iip=0;
    xmlobj=xmlobjects.first_child();
    xmlob1name = xmlobj.attribute("name").value();
    meshf = xmlobj.child("MeshFile");
    xmlob1meshf = meshf.child_value();
    xmlpose = xmlobj.child("PoseState");
    xmlposit = xmlpose.child("Position");
    temp = xmlposit.attribute("x").as_double();
    xmlob1pos.push_back(temp);
    temp = xmlposit.attribute("y").as_double();
    xmlob1pos.push_back(temp);
    temp = xmlposit.attribute("z").as_double();
    xmlob1pos.push_back(temp);
    xmlquaternion = xmlpose.child("YPR");
    temp = xmlquaternion.attribute("rotx").as_double();
    xmlob1quat.push_back(temp);
    temp = xmlquaternion.attribute("roty").as_double();
    xmlob1quat.push_back(temp);
    temp = xmlquaternion.attribute("rotz").as_double();
    xmlob1quat.push_back(temp);
//            xmlob1quat.push_back(xmlquaternion.attribute("z").as_double());
    xmldeform = xmlobj.child("Deformation");
    xmlob1deform = xmldeform.child_value();


    xmlobj = xmlobj.next_sibling();
    xmlob2name = xmlobj.attribute("name").value();
    meshf = xmlobj.child("MeshFile");
    xmlob2meshf = meshf.child_value();
    xmlpose = xmlobj.child("PoseState");
    xmlposit = xmlpose.child("Position");
    temp = xmlposit.attribute("x").as_double();
    xmlob2pos.push_back(temp);
    temp = xmlposit.attribute("y").as_double();
    xmlob2pos.push_back(temp);
    temp = xmlposit.attribute("z").as_double();
    xmlob2pos.push_back(temp);
    xmlquaternion = xmlpose.child("YPR");
    temp = xmlquaternion.attribute("rotx").as_double();
    xmlob2quat.push_back(temp);
    temp = xmlquaternion.attribute("roty").as_double();
    xmlob2quat.push_back(temp);
    temp = xmlquaternion.attribute("rotz").as_double();
    xmlob2quat.push_back(temp);
    //            xmlob2quat.push_back(xmlquaternion.attribute("scale").as_double());
    xmldeform = xmlobj.child("Deformation");
    xmlob2deform = xmldeform.child_value();



//    for (xmlobj=xmlobjects.first_child(); xmlobj; xmlobj = xmlobj.next_sibling())
//    {
//        if (iip==0)
//        {


//        }else
//        {
//            xmlob2name = xmlobj.attribute("name").value();
//            meshf = xmlobj.child("MeshFile");
//            xmlob2meshf = meshf.child_value();
//            xmlpose = xmlobj.child("PoseState");
//            xmlposit = xmlpose.child("Position");
//            xmlob2pos.push_back(xmlposit.attribute("x").as_double());
//            xmlob2pos.push_back(xmlposit.attribute("y").as_double());
//            xmlob2pos.push_back(xmlposit.attribute("z").as_double());
//            xmlquaternion = xmlpose.child("YPR");
//            xmlob2quat.push_back(xmlquaternion.attribute("rotx").as_double());
//            xmlob2quat.push_back(xmlquaternion.attribute("roty").as_double());
//            xmlob2quat.push_back(xmlquaternion.attribute("rotz").as_double());
////            xmlob2quat.push_back(xmlquaternion.attribute("scale").as_double());
//            xmldeform = xmlobj.child("Deformation");
//            xmlob2deform = xmldeform.child_value();
//        }

//        iip++;
//    }

}

void xmlparser::getInstructor()
{
    xml_node xmlinstructor;
    xml_node xmlhand;
    xml_node xmlhandmesh;
    xml_node xmlhandmeshfilep;
    xml_node xmlhandmeshfiled;
    xml_node xmlpose;
    xml_node xmlposit;
    xml_node xmlquaternion;
    xml_node xmlhandmodel;
    xml_node xmlhandparameters;

    xmlinstructor = xmlkeyframe.child("Instructor");
    int ii=0;
    for (xmlhand=xmlinstructor.first_child(); xmlhand; xmlhand = xmlhand.next_sibling())
    {
        if (ii==0)
        {
            xmlinstrhandid = xmlhand.attribute("id").value();
            xmlinstrhandname = xmlhand.attribute("name").value();
            xmlhandmesh = xmlhand.child("HandMeshes");
            int jj=0;
            for (xmlhandmeshfilep = xmlhandmesh.first_child(); xmlhandmeshfilep; xmlhandmeshfilep = xmlhandmeshfilep.next_sibling())
            {
                if (jj==0)
                    xmlinstrhandmeshp = xmlhandmeshfilep.child_value();
                else
                    xmlinstrhandmeshd = xmlhandmeshfilep.child_value();

                jj++;
            }
            xmlpose = xmlhand.child("PoseState");
            xmlposit = xmlpose.child("Position");
            xmlinstrhandpos.push_back(xmlposit.attribute("x").as_double());
            xmlinstrhandpos.push_back(xmlposit.attribute("y").as_double());
            xmlinstrhandpos.push_back(xmlposit.attribute("z").as_double());
            xmlquaternion = xmlpose.child("YPR");
            //xmlinstrhandquat.push_back(xmlquaternion.attribute("w").as_double());
            xmlinstrhandquat.push_back(xmlquaternion.attribute("rotx").as_double());
            xmlinstrhandquat.push_back(xmlquaternion.attribute("roty").as_double());
            xmlinstrhandquat.push_back(xmlquaternion.attribute("rotz").as_double());
            xmlhandmodel = xmlhand.child("HandModel");
            xmlinstrhandmodel = xmlhandmodel.attribute("ModelDefinition").value();
            xmlhandparameters = xmlhand.child("HandParameters");
            xmlinstrhandparam = xmlhandparameters.attribute("Params").value();

        }else
        {
            xmlinstlhandid = xmlhand.attribute("id").value();
            xmlinstlhandname = xmlhand.attribute("name").value();
            xmlhandmesh = xmlhand.child("HandMeshes");
            int jj=0;
            for (xmlhandmeshfilep = xmlhandmesh.first_child(); xmlhandmeshfilep; xmlhandmeshfilep = xmlhandmeshfilep.next_sibling())
            {
                if (jj==0)
                    xmlinstlhandmeshp = xmlhandmeshfilep.child_value();
                else
                    xmlinstlhandmeshd = xmlhandmeshfilep.child_value();

                jj++;
            }
            xmlpose = xmlhand.child("PoseState");
            xmlposit = xmlpose.child("Position");
            xmlinstlhandpos.push_back(xmlposit.attribute("x").as_double());
            xmlinstlhandpos.push_back(xmlposit.attribute("y").as_double());
            xmlinstlhandpos.push_back(xmlposit.attribute("z").as_double());
            xmlquaternion = xmlpose.child("YPR");
            //xmlinstlhandquat.push_back(xmlquaternion.attribute("w").as_double());
            xmlinstlhandquat.push_back(xmlquaternion.attribute("rotx").as_double());
            xmlinstlhandquat.push_back(xmlquaternion.attribute("roty").as_double());
            xmlinstlhandquat.push_back(xmlquaternion.attribute("rotz").as_double());
            xmlhandmodel = xmlhand.child("HandModel");
            xmlinstlhandmodel = xmlhandmodel.attribute("ModelDefinition").value();
            xmlhandparameters = xmlhand.child("HandParameters");
            xmlinstlhandparam = xmlhandparameters.attribute("Params").value();

        }

        ii++;
    }


}


void xmlparser::getGrasping()
{

    xml_node xmlgraspingstate;
    xml_node xmlgrasp;
    xml_node xmlcontactpoints;
    xml_node xmlcontactpoint;
    xml_node xmlcontactforces;
    xml_node xmlcontactforce;


    xmlgraspingstate = xmlkeyframe.child("GraspingState");
    int ii=0;

    for (xmlgrasp=xmlgraspingstate.first_child(); xmlgrasp; xmlgrasp = xmlgrasp.next_sibling())
    {
        if (ii==0)
        {
            xmlgrasplid= xmlgrasp.attribute("id").value();
            xmlcontactpoints = xmlgrasp.child("ContactPoints");
            for (xmlcontactpoint= xmlcontactpoints.first_child(); xmlcontactpoint; xmlcontactpoint=xmlcontactpoint.next_sibling())
            {
                xmlgrasplcp.push_back(xmlcontactpoint.attribute("x").as_double());
                xmlgrasplcp.push_back(xmlcontactpoint.attribute("y").as_double());
                xmlgrasplcp.push_back(xmlcontactpoint.attribute("z").as_double());
            }
            xmlcontactforces = xmlgrasp.child("ContactForces");
            for (xmlcontactforce=xmlcontactforces.first_child();xmlcontactforce; xmlcontactforce=xmlcontactforce.next_sibling())
            {
                xmlgrasplcf.push_back(xmlcontactforce.attribute("fx").as_double());
                xmlgrasplcf.push_back(xmlcontactforce.attribute("fy").as_double());
                xmlgrasplcf.push_back(xmlcontactforce.attribute("fz").as_double());

            }

        }else
        {
            xmlgrasprid= xmlgrasp.attribute("id").value();
            xmlcontactpoints = xmlgrasp.child("ContactPoints");
            for (xmlcontactpoint= xmlcontactpoints.first_child(); xmlcontactpoint; xmlcontactpoint=xmlcontactpoint.next_sibling())
            {
                xmlgrasprcp.push_back(xmlcontactpoint.attribute("x").as_double());
                xmlgrasprcp.push_back(xmlcontactpoint.attribute("y").as_double());
                xmlgrasprcp.push_back(xmlcontactpoint.attribute("z").as_double());
            }
            xmlcontactforces = xmlgrasp.child("ContactForces");
            for (xmlcontactforce=xmlcontactforces.first_child();xmlcontactforce; xmlcontactforce=xmlcontactforce.next_sibling())
            {
                xmlgrasprcf.push_back(xmlcontactforce.attribute("fx").as_double());
                xmlgrasprcf.push_back(xmlcontactforce.attribute("fy").as_double());
                xmlgrasprcf.push_back(xmlcontactforce.attribute("fz").as_double());

            }
        }
        ii++;
    }


}


void xmlparser::setKeyFrame(int idx, double timestamp)
{
    string keyfrname;
    string index;
    string tms;

    ostringstream name;
    name << nkf;
    keyfrname = name.str();

    ostringstream ind;
    ind << idx;
    index = ind.str();

    ostringstream tm;
    tm << timestamp;
    tms = tm.str();


    keyframe = doc.append_child();
    keyframe.set_name("KeyFrame");

    keyframe.append_attribute("id") = keyfrname.c_str();
    keyframe.append_attribute("idx") = index.c_str();
    keyframe.append_attribute("t") = tms.c_str();
    keyframe.append_attribute("xmlns") = "http://www.SARAFunXML.com";
    keyframe.append_attribute("xmlns:xsi") = "http://www.w3.org/2001/XMLSchema-instance";
    keyframe.append_attribute("xsi:schemaLocation") = "http://www.SARAFunXML.com SARAFun_KeyFrame_XmlSpec_v02.xsd";



}


void xmlparser::setSemantics()
{
    std::string assemblyT;
    std::string ObjInter;
    std::string rhandinter;
    std::string rhandpart;
    std::string lhandinter;
    std::string lhandpart;

    if (AssemblyType==0)
        assemblyT="folding";
    else if (AssemblyType==1)
        assemblyT="byDeformation";

    switch (ObjectsInteraction) {
    case 0:
        ObjInter="NotTouching";
        break;
    case 1:
        ObjInter="ObjAoverlappingObjB";
        break;
    case 2:
        ObjInter="touching";
        break;
    case 3:
        ObjInter="MajorParallel";
        break;
    case 4:
        ObjInter="MediumParallel";
        break;
    case 5:
        ObjInter="MinorParallel";
        break;
    case 6:
        ObjInter="AllParallel";
        break;
    case 7:
        ObjInter="TouchingParallel";
        break;
    case 8:
        ObjInter="TouchingAllParallel";
        break;
    case 9:
        ObjInter="OverlappingParallel";
        break;
    case 10:
        ObjInter="OverlappingAllParallel";
        break;
    case 11:
        ObjInter="ObjBoverlappingObjA";
        break;
    case 12:
        ObjInter="ObjAabsent";
        break;
    case 13:
        ObjInter="ObjBabsent";
        break;
    }

    if (RightHandObjectInteraction[0]==11)
        rhandinter = "absent";
    else if (RightHandObjectInteraction[0]==0)
        rhandinter = "far";//"notTouching";
    else if (RightHandObjectInteraction[0]==2)
        rhandinter = "grasping";

    if (RightHandObjectInteraction[1]==1)
        rhandpart = "ObjA";
    else if (RightHandObjectInteraction[1]==2)
        rhandpart = "ObjB";
    else
        rhandpart = "none";

    if (LeftHandObjectInteraction[0]==11)
        lhandinter = "absent";
    else if (LeftHandObjectInteraction[0]==0)
        lhandinter = "far";  //notTouching";
    else if (LeftHandObjectInteraction[0]==2)
        lhandinter = "grasping";

    if (LeftHandObjectInteraction[1]==1)
        lhandpart = "ObjA";
    else if (LeftHandObjectInteraction[1]==2)
        lhandpart = "ObjB";
    else
        lhandpart = "none";

    semantics = keyframe.append_child();
    semantics.set_name("Semantics");

    instructorDescrNode = semantics.append_child();
    instructorDescrNode.set_name("InstructorDescription");
    instructorDescrNode.append_attribute("description") = InstructorDescription.c_str();

    assemblyTypeNode = semantics.append_child();
    assemblyTypeNode.set_name("AssemblyType");
    assemblyTypeNode.append_attribute("id") = assemblyT.c_str();

    objectInterNode = semantics.append_child();
    objectInterNode.set_name("ObjectInteraction");
    objectInterNode.append_attribute("id") = ObjInter.c_str();

    RhandObjInterNode = semantics.append_child();
    RhandObjInterNode.set_name("RightHandObjectInteraction");
    RhandObjInterNode.append_attribute("interaction") = rhandinter.c_str();
    RhandObjInterNode.append_attribute("AssemblyPart") = rhandpart.c_str();

    LhandObjInterNode = semantics.append_child();
    LhandObjInterNode.set_name("LeftHandObjectInteraction");
    LhandObjInterNode.append_attribute("interaction") = lhandinter.c_str();
    LhandObjInterNode.append_attribute("AssemblyPart") = lhandpart.c_str();

}

void xmlparser::setCurrAction(string curAcid, string descr, int nrsensors, int idx1F, int idx1L, int idx2F, int idx2L)
{

    currentAction =  keyframe.append_child();
    currentAction.set_name("CurrentAction");

    currentAction.append_attribute("id") = curAcid.c_str();

    description = currentAction.append_child();
    description.set_name("Description");
    description.append_child(pugi::node_pcdata).set_value(descr.c_str());

    this->setInvObjects(2);

    this->setVisualFeedback(2, idx1F, idx1L, idx2F, idx2L);
}


void xmlparser::setInvObjects(int nrobj)
{
    involvedObjs = currentAction.append_child();
    involvedObjs.set_name("InvolvedObjects");
    if (nrobj==1)
    {
        xml_node obj1 = involvedObjs.append_child();
        obj1.set_name("Object");
        obj1.append_attribute("id") = "ObjA";
    }
    else if (nrobj==2)
    {
        xml_node obj1 = involvedObjs.append_child();
        obj1.set_name("Object");
        obj1.append_attribute("id") = "ObjA";
        xml_node obj2 = involvedObjs.append_child();
        obj2.set_name("Object");
        obj2.append_attribute("id") = "ObjB";
    }else if (nrobj==3)
    {
        xml_node obj1 = involvedObjs.append_child();
        obj1.set_name("Object");
        obj1.append_attribute("id") = "ObjA";
        xml_node obj2 = involvedObjs.append_child();
        obj2.set_name("Object");
        obj2.append_attribute("id") = "ObjB";
        xml_node obj3 = involvedObjs.append_child();
        obj3.set_name("Object");
        obj3.append_attribute("id") = "ObjC";
    }
}


void xmlparser::setVisualFeedback(int nrsensors, int idx1F, int idx1L, int idx2F, int idx2L)
{
    visfeed = currentAction.append_child();
    visfeed.set_name("VisualFeedback");
    this->setCameraSensor("RealSenseF200", idx1F, idx1L);
    if (nrsensors==2)
        this->setCameraSensor("Xtion", idx2F, idx2L );
}

void xmlparser::setObjValidAttributeType()
{

}

void xmlparser::setCameraSensor(string camname, int idxF, int idxL)
{
    sensor = visfeed.append_child();
    sensor.set_name("CameraSensor");
    sensor.append_attribute("id")= camname.c_str();

    this->setFrameRange(camname, idxF, idxL);
}

void xmlparser::setCamValidAttributeType()
{

}


void xmlparser::setFrameRange( string camname, int idxF, int idxL)
{
    string idxFF;
    string idxLL;
    string listname;
    listname = camname + "_Sequence.xml";
    ostringstream ind1;
    ind1 << idxF;
    idxFF = ind1.str();

    ostringstream ind2;
    ind2 << idxL;
    idxLL = ind2.str();

    framerange = sensor.append_child();
    framerange.set_name("FrameRange");
    framerange.append_attribute("idxFirst")= idxFF.c_str();
    framerange.append_attribute("idxLast")= idxLL.c_str();
    framerange.append_attribute("fileList")= listname.c_str();
}


void xmlparser::setObjects(int nrobjects)
{
    objects = keyframe.append_child();
    objects.set_name("Objects");

    this->setObject(1);
    if (nrobjects==2)
        this->setObject(2);
}


void xmlparser::setObject(int oi)
{
    if (oi==1)
    {
        objectt = objects.append_child();
        objectt.set_name("Object");
        objectt.append_attribute("id") = "ObjA";
        objectt.append_attribute("name") = "Mobile Phone PCB";
        this->setMeshFile("mobile_phone_pcb.obj");
        this->setPoseState(1);
        this->setDeformation();


    }else
    {
        objectt = objects.append_child();
        objectt.set_name("Object");
        objectt.append_attribute("id") = "ObjB";
        objectt.append_attribute("name") = "Mobile Phone Case";
        this->setMeshFile("mobile_phone_case.obj");
        this->setPoseState(2);
        this->setDeformation();
    }


}


void xmlparser::setMeshFile(std::string mesh)
{
    meshfile = objectt.append_child();
    meshfile.set_name("MeshFile");
    meshfile.append_child(pugi::node_pcdata).set_value(mesh.c_str());

}


void xmlparser::setPoseState(int il)
{

     pose = objectt.append_child();
     pose.set_name("PoseState");
     this->setPosition(il);
     this->setQuaternion(il);

}


void xmlparser::setPosition(int iil)
{
    string x1;
    string y1;
    string z1;
    ostringstream x11;
    ostringstream y11;
    ostringstream z11;

    if (iil==1)
    {
        x11<< posobj1.at(0);
        x1 = x11.str();

        y11<< posobj1.at(1);
        y1 = y11.str();

        z11<< posobj1.at(2);
        z1 = z11.str();


    }else
    {
        x11<< posobj2.at(0);
        x1 = x11.str();

        y11<< posobj2.at(1);
        y1 = y11.str();

        z11<< posobj2.at(2);
        z1 = z11.str();
    }
    pos1 = pose.append_child();
    pos1.set_name("Position");
    pos1.append_attribute("x")=x1.c_str();
    pos1.append_attribute("y")=y1.c_str();
    pos1.append_attribute("z")=z1.c_str();
}


void xmlparser::setQuaternion(int iil)
{
    string ww1;
    string xx1;
    string yy1;
    string zz1;

    ostringstream ww11;
    ostringstream xx11;
    ostringstream yy11;
    ostringstream zz11;
    if (iil==1)
    {

        ww11 << quatobj1.at(0);
        ww1 = ww11.str();

        xx11 << quatobj1.at(1);
        xx1 = xx11.str();

        yy11 << quatobj1.at(2);
        yy1 = yy11.str();

//        zz11 << quatobj1.at(3);
//        zz1 = zz11.str();


    }else
    {

        ww11 << quatobj2.at(0);
        ww1 = ww11.str();

        xx11 << quatobj2.at(1);
        xx1 = xx11.str();

        yy11 << quatobj2.at(2);
        yy1 = yy11.str();

//        zz11 << quatobj2.at(3);
//        zz1 = zz11.str();

     }

    quat1 = pose.append_child();
    quat1.set_name("YPR");
    quat1.append_attribute("rotx")=ww1.c_str();
    quat1.append_attribute("roty")=xx1.c_str();
    quat1.append_attribute("rotz")=yy1.c_str();
//    quat1.append_attribute("scale")=zz1.c_str();

}


void xmlparser::setDeformation()
{
    deform = objectt.append_child();
    deform.set_name("Deformation");
    deform.append_child(pugi::node_pcdata).set_value("NotYetDefined");
}


void xmlparser::setInstructor()
{

    instr = keyframe.append_child();
    instr.set_name("Instructor");
    this->setHand(1);               //left hand
    this->setHand(2);               //right hand

}


void xmlparser::setHand(int hi)
{

    string x1;
    string y1;
    string z1;
    ostringstream x11;
    ostringstream y11;
    ostringstream z11;

    string ww1;
    string xx1;
    string yy1;
    string zz1;

    ostringstream ww11;
    ostringstream xx11;
    ostringstream yy11;
    ostringstream zz11;

    if (hi==1)
    {
        lhand = instr.append_child();
        lhand.set_name("Hand");
        lhand.append_attribute("id")= "LeftHand";
        lhand.append_attribute("name")= "Instructors left Hand";

        this->setHandMeshes(hi);

        PoseState = lhand.append_child();
        PoseState.set_name("PoseState");
        instrposl = PoseState.append_child();
        instrposl.set_name("Position");

        x11<< insthandlall.at(0);
        x1 = x11.str();

        y11<< insthandlall.at(1);
        y1 = y11.str();

        z11<< insthandlall.at(2);
        z1 = z11.str();

        instrposl.append_attribute("x")= x1.c_str();
        instrposl.append_attribute("y")= y1.c_str();
        instrposl.append_attribute("z")= z1.c_str();

        instrquatl = PoseState.append_child();
        instrquatl.set_name("YPR");

        ww11 << 0;
        ww1 = ww11.str();

        xx11 << insthandlall.at(3);
        xx1 = xx11.str();

        yy11 << insthandlall.at(4);
        yy1 = yy11.str();

        zz11 << insthandlall.at(5);
        zz1 = zz11.str();

       // instrquatl.append_attribute("w")= ww1.c_str();
        instrquatl.append_attribute("rotx")= xx1.c_str();
        instrquatl.append_attribute("roty")= yy1.c_str();
        instrquatl.append_attribute("rotz")= zz1.c_str();





    }else
    {
        rhand = instr.append_child();
        rhand.set_name("Hand");
        rhand.append_attribute("id")= "RightHand";
        rhand.append_attribute("name")= "Instructors right Hand";

        this->setHandMeshes(hi);

        PoseState = rhand.append_child();
        PoseState.set_name("PoseState");
        instrposr = PoseState.append_child();
        instrposr.set_name("Position");

        x11<< insthandrall.at(0);
        x1 = x11.str();

        y11<< insthandrall.at(1);
        y1 = y11.str();

        z11<< insthandrall.at(2);
        z1 = z11.str();

        instrposr.append_attribute("x")= x1.c_str();
        instrposr.append_attribute("y")= y1.c_str();
        instrposr.append_attribute("z")= z1.c_str();

        instrquatr = PoseState.append_child();
        instrquatr.set_name("YPR");

        ww11 << 0;
        ww1 = ww11.str();

        xx11 << insthandrall.at(3);
        xx1 = xx11.str();

        yy11 << insthandrall.at(4);
        yy1 = yy11.str();

        zz11 << insthandrall.at(5);
        zz1 = zz11.str();

       // instrquatr.append_attribute("w")= ww1.c_str();
        instrquatr.append_attribute("rotx")= xx1.c_str();
        instrquatr.append_attribute("roty")= yy1.c_str();
        instrquatr.append_attribute("rotz")= zz1.c_str();

    }

    this->setHandModel(hi);
    this->setHandParameters(hi);
}



void xmlparser::setHandMeshes(int hi)
{
    if (hi==1)
    {
        handmeshes = lhand.append_child();
        handmeshes.set_name("HandMeshes");
        meshfilel1 = handmeshes.append_child();
        meshfilel1.set_name("MeshFile");
        meshfilel1.append_child(pugi::node_pcdata).set_value("LeftHandPrototype.obj");
        meshfilel2 = handmeshes.append_child();
        meshfilel2.set_name("MeshFile");
        meshfilel2.append_child(pugi::node_pcdata).set_value("LeftHand.obj");

    }else
    {
        handmeshes = rhand.append_child();
        handmeshes.set_name("HandMeshes");
        meshfilel1 = handmeshes.append_child();
        meshfilel1.set_name("MeshFile");
        meshfilel1.append_child(pugi::node_pcdata).set_value("RightHandPrototype.obj");
        meshfilel2 = handmeshes.append_child();
        meshfilel2.set_name("MeshFile");
        meshfilel2.append_child(pugi::node_pcdata).set_value("RightHand.obj");
    }
}


void xmlparser::setHandModel(int hi)
{
    if (hi==1)
    {
        lhandmodel = lhand.append_child();
        lhandmodel.set_name("HandModel");
        lhandmodel.append_attribute("ModelDefinition")= "LeftHandModel.xml";
    }else
    {
        rhandmodel = rhand.append_child();
        rhandmodel.set_name("HandModel");
        rhandmodel.append_attribute("ModelDefinition")= "RightHandModel.xml";
    }
}


void xmlparser::setHandParameters(int hi)
{
    if (hi==1)
    {
        lhandparams = lhand.append_child();
        lhandparams.set_name("HandParameters");
        lhandparams.append_attribute("Params")= "LeftHandParams_02.xml";
    }else
    {
        rhandparams = rhand.append_child();
        rhandparams.set_name("HandParameters");
        rhandparams.append_attribute("Params")= "RightHandParams_02.xml";
    }
}


void xmlparser::setGraspingState()
{
    grstate = keyframe.append_child();
    grstate.set_name("GraspingState");
    this->setGrasp(1);   //left grasp
    this->setGrasp(2);   //right grasp
}


void xmlparser::setGrasp(int gi)
{


    if (gi==1)
    {
        graspl = grstate.append_child();
        graspl.set_name("Grasp");
        graspl.append_attribute("id")="Left";
        grobjl = graspl.append_child();
        grobjl.set_name("Object");
        grobjl.append_attribute("id")="ObjA";

        this->setContactPoints(1);
        this->setContactForces(1);

    }else
    {
        graspr = grstate.append_child();
        graspr.set_name("Grasp");
        graspr.append_attribute("id")="Right";
        grobjr = graspr.append_child();
        grobjr.set_name("Object");
        grobjr.append_attribute("id")= "ObjB";

        this->setContactPoints(2);
        this->setContactForces(2);

    }
}


void xmlparser::setContactPoints(int gi)
{

    if (gi==1)
    {
        conpointsl = graspl.append_child();
        conpointsl.set_name("ContactPoints");
        div_t divresult;
        divresult = div (ConPointl.size(), 3);
        for (int iii=0;iii<divresult.quot;iii++)
            this->setContactPoint(1,iii);
    }else
    {
        conpointsr = graspr.append_child();
        conpointsr.set_name("ContactPoints");

        div_t divresult;
        divresult = div (ConPointr.size(), 3);
        for (int iii=0;iii<divresult.quot;iii++)
            this->setContactPoint(2,iii);


    }
}


void xmlparser::setContactPoint(int gi, int iii)
{
    string x1;
    string y1;
    string z1;
    ostringstream x11;
    ostringstream y11;
    ostringstream z11;

    if (gi==1)
    {
        x11<< ConPointl.at(iii*3+0);
        x1 = x11.str();

        y11<< ConPointl.at(iii*3+1);
        y1 = y11.str();

        z11<< ConPointl.at(iii*3+2);
        z1 = z11.str();

        conpoint = conpointsl.append_child();

    }
    else
    {

        x11<< ConPointr.at(iii*3+0);
        x1 = x11.str();

        y11<< ConPointr.at(iii*3+1);
        y1 = y11.str();

        z11<< ConPointr.at(iii*3+2);
        z1 = z11.str();

        conpoint = conpointsr.append_child();
    }
    conpoint.set_name("ContactPoint");
    conpoint.append_attribute("x")= x1.c_str();
    conpoint.append_attribute("y")= y1.c_str();
    conpoint.append_attribute("z")= z1.c_str();
}


void xmlparser::setContactForces(int gi)
{
    if (gi==1)
    {
        conforcsl= graspl.append_child();
        conforcsl.set_name("ContactForces");

        div_t divresult;
        divresult = div(ConForcel.size(),3);

        for (int iii=0;iii<divresult.quot;iii++)
            this->setContactForce(1,iii);
    }else
    {
        conforcsr = graspr.append_child();
        conforcsr.set_name("ContactForces");

        div_t divresult;
        divresult = div(ConForcer.size(),3);
        for (int iii=0;iii<divresult.quot;iii++)
            this->setContactForce(2,iii);
    }
}


void xmlparser::setContactForce(int gi, int iii)
{
    string x1;
    string y1;
    string z1;
    ostringstream x11;
    ostringstream y11;
    ostringstream z11;

    if (gi==1)
    {
        x11<< ConForcel.at(iii*3+0);
        x1 = x11.str();

        y11<< ConForcel.at(iii*3+1);
        y1 = y11.str();

        z11<< ConForcel.at(iii*3+2);
        z1 = z11.str();

        conforce = conforcsl.append_child();

    }
    else
    {

        x11<< ConForcer.at(iii*3+0);
        x1 = x11.str();

        y11<< ConForcer.at(iii*3+1);
        y1 = y11.str();

        z11<< ConForcer.at(iii*3+2);
        z1 = z11.str();

        conforce = conforcsr.append_child();
    }
    conforce.set_name("ContactForce");
    conforce.append_attribute("fx")= x1.c_str();
    conforce.append_attribute("fy")= y1.c_str();
    conforce.append_attribute("fz")= z1.c_str();
}


void xmlparser::setCLosingData(int nrfr)
{

    //doc.print(std::cout);
    std::string outfn;
    std::string otfn;
    ostringstream outf;
    outf<< nrfr;
    outfn = outf.str();
    otfn = "/home/gpiperagkas/SARAFun_kf/kf"+ outfn + ".xml";
    doc.save_file(otfn.c_str());


}


xmlparser::~xmlparser()
{

    posobj1.clear();

    quatobj1.clear();

    posobj2.clear();

    quatobj2.clear();

    instposl.clear();

    instposr.clear();

    instquatl.clear();

    instquatr.clear();

    ConPointl.clear();

    ConForcel.clear();

    ConPointr.clear();

    ConForcer.clear();

    insthandlall.clear();

    insthandrall.clear();

    RightHandObjectInteraction.clear();

    LeftHandObjectInteraction.clear();


}
