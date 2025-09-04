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

#include "xmlhandparser.h"
#define NUM_PARAMETERS 48 //for each hand, 48 parameters to be loaded

using namespace std;
using namespace pugi;

xmlhandparser::xmlhandparser(int keyframenr, float *coeff_hand, int lr)
{
        //left hand lr=0
        //right hand lr=1


    for (unsigned int ll=0;ll<NUM_PARAMETERS;ll++)
        insthandall.push_back(coeff_hand[ll]);


    nkf = keyframenr;
    //xmlname = xmlfname;
    side=lr;

    xml_node dNode = doc.append_child(pugi::node_declaration);
    dNode.append_attribute("version")    = "1.0";
    dNode.append_attribute("encoding")   = "utf-8";
    dNode.append_attribute("standalone") = "yes";



}

xmlhandparser::xmlhandparser(std::string xmlinfile)
{

    result = indoc.load_file(xmlinfile.c_str());


    xmlkeyframe = indoc.child("KeyFrame");


        xmlkfid_h = xmlkeyframe.attribute("id").value();
        xmlkfidx_h = xmlkeyframe.attribute("idx").as_int();
        xmlkft_h = xmlkeyframe.attribute("t").as_double();
        xmlkfxmlns_h = xmlkeyframe.attribute("xmlns").value();
        xmlkfxsischemaloc_h = xmlkeyframe.attribute("xsi:schemaLocation").value();


}


void xmlhandparser::setKeyFrame(int idx, double timestamp)
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
    keyframe.append_attribute("xsi:schemaLocation") = "http://www.SARAFunXML.com SARAFun_KeyFrame_XmlSpec_v02.xsd";



}


void xmlhandparser::setHandpars()
{

    string x1;
    string y1;
    string z1;
    string w1;
    string c1;
    string t1;
    string u1;
    string p1;
    ostringstream x11;
    ostringstream y11;
    ostringstream z11;
    ostringstream w11;
    ostringstream c11;
    ostringstream t11;
    ostringstream u11;
    ostringstream p11;



    hand = keyframe.append_child();
    hand.set_name("Hand");
    if (side==0)
    {
        hand.append_attribute("id")= "LeftHand";
        hand.append_attribute("name")= "Instructors Left Hand";
    }else
    {
        hand.append_attribute("id")= "RightHand";
        hand.append_attribute("name")= "Instructors Right Hand";
    }

    pos= hand.append_child();
    pos.set_name("Position");

    x11<< insthandall.at(0);
    x1 = x11.str();

    y11<< insthandall.at(1);
    y1 = y11.str();

    z11<< insthandall.at(2);
    z1 = z11.str();

    pos.append_attribute("x")= x1.c_str();
    pos.append_attribute("y")= y1.c_str();
    pos.append_attribute("z")= z1.c_str();


    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();



    orient = hand.append_child();
    orient.set_name("Orientation");

    x11<< insthandall.at(3);
    x1 = x11.str();

    y11<< insthandall.at(4);
    y1 = y11.str();

    z11<< insthandall.at(5);
    z1 = z11.str();

    orient.append_attribute("x")=x1.c_str();
    orient.append_attribute("y")=y1.c_str();
    orient.append_attribute("z")=z1.c_str();

    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();

    wrist = hand.append_child();
    wrist.set_name("Wrist");

    x11<< insthandall.at(6);
    x1 = x11.str();

    y11<< insthandall.at(7);
    y1 = y11.str();

    wrist.append_attribute("theta")=x1.c_str();
    wrist.append_attribute("phi")= y1.c_str();

    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();


    thumb = hand.append_child();
    thumb.set_name("Thumb");

    x11<< insthandall.at(8);
    x1 = x11.str();

    y11<< insthandall.at(9);
    y1 = y11.str();

    z11<< insthandall.at(10);
    z1 = z11.str();

    w11<< insthandall.at(11);
    w1 = w11.str();

    c11<< insthandall.at(12);
    c1 = c11.str();

    t11<< insthandall.at(41);
    t1 = t11.str();

    u11<< insthandall.at(46);
    u1 = u11.str();

    thumb.append_attribute("theta")= x1.c_str();
    thumb.append_attribute("phi")= y1.c_str();
    thumb.append_attribute("k1_theta")= z1.c_str();
    thumb.append_attribute("k1_phi")= w1.c_str();
    thumb.append_attribute("k2_phi")= c1.c_str();
    thumb.append_attribute("twist")=t1.c_str();
    thumb.append_attribute("length")=u1.c_str();


    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();

    w11.str("");
    w11.clear();

    c11.str("");
    c11.clear();

    t11.str("");
    t11.clear();

    u11.str("");
    u11.clear();

    f0=hand.append_child();
    f0.set_name("F0");

    x11<< insthandall.at(13);
    x1 = x11.str();

    y11<< insthandall.at(14);
    y1 = y11.str();

    z11<< insthandall.at(15);
    z1 = z11.str();

    w11<< insthandall.at(16);
    w1 = w11.str();

    c11<< insthandall.at(17);
    c1 = c11.str();

    t11<< insthandall.at(18);
    t1 = t11.str();

    u11<< insthandall.at(37);
    u1 = u11.str();

    p11<< insthandall.at(42);
    p1 = p11.str();

    f0.append_attribute("root_theta")=x1.c_str();
    f0.append_attribute("root_phi")=y1.c_str();
    f0.append_attribute("theta")=z1.c_str();
    f0.append_attribute("phi")=w1.c_str();
    f0.append_attribute("knuckle_mid")=c1.c_str();
    f0.append_attribute("knuckle_end")=t1.c_str();
    f0.append_attribute("twist")=u1.c_str();
    f0.append_attribute("length")=p1.c_str();


    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();

    w11.str("");
    w11.clear();

    c11.str("");
    c11.clear();

    t11.str("");
    t11.clear();

    u11.str("");
    u11.clear();

    p11.str("");
    p11.clear();

    f1=hand.append_child();
    f1.set_name("F1");

    x11<< insthandall.at(19);
    x1 = x11.str();

    y11<< insthandall.at(20);
    y1 = y11.str();

    z11<< insthandall.at(21);
    z1 = z11.str();

    w11<< insthandall.at(22);
    w1 = w11.str();

    c11<< insthandall.at(23);
    c1 = c11.str();

    t11<< insthandall.at(24);
    t1 = t11.str();

    u11<< insthandall.at(38);
    u1 = u11.str();

    p11<< insthandall.at(43);
    p1 = p11.str();

    f1.append_attribute("root_theta")=x1.c_str();
    f1.append_attribute("root_phi")=y1.c_str();
    f1.append_attribute("theta")=z1.c_str();
    f1.append_attribute("phi")=w1.c_str();
    f1.append_attribute("knuckle_mid")=c1.c_str();
    f1.append_attribute("knuckle_end")=t1.c_str();
    f1.append_attribute("twist")=u1.c_str();
    f1.append_attribute("length")=p1.c_str();




    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();

    w11.str("");
    w11.clear();

    c11.str("");
    c11.clear();

    t11.str("");
    t11.clear();

    u11.str("");
    u11.clear();

    p11.str("");
    p11.clear();

    f2=hand.append_child();
    f2.set_name("F2");

    x11<< insthandall.at(25);
    x1 = x11.str();

    y11<< insthandall.at(26);
    y1 = y11.str();

    z11<< insthandall.at(27);
    z1 = z11.str();

    w11<< insthandall.at(28);
    w1 = w11.str();

    c11<< insthandall.at(29);
    c1 = c11.str();

    t11<< insthandall.at(30);
    t1 = t11.str();

    u11<< insthandall.at(39);
    u1 = u11.str();

    p11<< insthandall.at(44);
    p1 = p11.str();

    f2.append_attribute("root_theta")=x1.c_str();
    f2.append_attribute("root_phi")=y1.c_str();
    f2.append_attribute("theta")=z1.c_str();
    f2.append_attribute("phi")=w1.c_str();
    f2.append_attribute("knuckle_mid")=c1.c_str();
    f2.append_attribute("knuckle_end")=t1.c_str();
    f2.append_attribute("twist")=u1.c_str();
    f2.append_attribute("length")=p1.c_str();



    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();

    w11.str("");
    w11.clear();

    c11.str("");
    c11.clear();

    t11.str("");
    t11.clear();

    u11.str("");
    u11.clear();

    p11.str("");
    p11.clear();

    f3=hand.append_child();
    f3.set_name("F3");

    x11<< insthandall.at(31);
    x1 = x11.str();

    y11<< insthandall.at(32);
    y1 = y11.str();

    z11<< insthandall.at(33);
    z1 = z11.str();

    w11<< insthandall.at(34);
    w1 = w11.str();

    c11<< insthandall.at(35);
    c1 = c11.str();

    t11<< insthandall.at(36);
    t1 = t11.str();

    u11<< insthandall.at(40);
    u1 = u11.str();

    p11<< insthandall.at(45);
    p1 = p11.str();

    f3.append_attribute("root_theta")=x1.c_str();
    f3.append_attribute("root_phi")=y1.c_str();
    f3.append_attribute("theta")=z1.c_str();
    f3.append_attribute("phi")=w1.c_str();
    f3.append_attribute("knuckle_mid")=c1.c_str();
    f3.append_attribute("knuckle_end")=t1.c_str();
    f3.append_attribute("twist")=u1.c_str();
    f3.append_attribute("length")=p1.c_str();



    x11.str("");
    x11.clear();

    y11.str("");
    y11.clear();

    z11.str("");
    z11.clear();

    w11.str("");
    w11.clear();

    c11.str("");
    c11.clear();

    t11.str("");
    t11.clear();

    u11.str("");
    u11.clear();

    p11.str("");
    p11.clear();

    scale = hand.append_child();
    scale.set_name("Scale");

    x11<< insthandall.at(47);
    x1 = x11.str();

    scale.append_attribute("scale")=x1.c_str();

    x11.str("");
    x11.clear();
}

void xmlhandparser::setClosingData(int nrfr)
{
    std::string outfn;
    std::string otfn;
    ostringstream outf;
    outf<< nrfr;
    outfn = outf.str();
    if (side==0) //left hand
    {
        otfn = "/home/gpiperagkas/SARAFun_kf/kf_l_hand"+ outfn + ".xml";
    }else //right hand
    {
        otfn = "/home/gpiperagkas/SARAFun_kf/kf_r_hand"+ outfn + ".xml";
    }
    doc.save_file(otfn.c_str());
}


void xmlhandparser::getHandPars()
{
    xml_node xmlhand;
    xml_node xmlpos;
    xml_node xmlorient;
    xml_node xmlthumb;
    xml_node xmlwrist;
    xml_node xmlf0;
    xml_node xmlf1;
    xml_node xmlf2;
    xml_node xmlf3;
    xml_node xmlscale;


    xmlhand = xmlkeyframe.child("Hand");
    xmlhandid_h = xmlhand.attribute("id").value();
    xmlhandname_h = xmlhand.attribute("name").value();

    xmlpos = xmlhand.child("Position");
    xmlhandpos_h.push_back(xmlpos.attribute("x").as_double());
    xmlhandpos_h.push_back(xmlpos.attribute("y").as_double());
    xmlhandpos_h.push_back(xmlpos.attribute("z").as_double());


    xmlorient = xmlhand.child("Orientation");
    xmlhandorient_h.push_back(xmlorient.attribute("x").as_double());
    xmlhandorient_h.push_back(xmlorient.attribute("y").as_double());
    xmlhandorient_h.push_back(xmlorient.attribute("z").as_double());

    xmlwrist = xmlhand.child("Wrist");
    xmlhandwrist_h.push_back(xmlwrist.attribute("theta").as_double());
    xmlhandwrist_h.push_back(xmlwrist.attribute("phi").as_double());

    xmlthumb = xmlhand.child("Thumb");
    xmlhandthumb_h.push_back(xmlthumb.attribute("theta").as_double());
    xmlhandthumb_h.push_back(xmlthumb.attribute("phi").as_double());
    xmlhandthumb_h.push_back(xmlthumb.attribute("k1_theta").as_double());
    xmlhandthumb_h.push_back(xmlthumb.attribute("k1_phi").as_double());
    xmlhandthumb_h.push_back(xmlthumb.attribute("k2_phi").as_double());
    xmlhandthumb_h.push_back(xmlthumb.attribute("twist").as_double());
    xmlhandthumb_h.push_back(xmlthumb.attribute("length").as_double());

    xmlf0 = xmlhand.child("F0");
    xmlhandf0_h.push_back(xmlf0.attribute("root_theta").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("root_phi").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("theta").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("phi").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("knuckle_mid").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("knuckle_end").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("twist").as_double());
    xmlhandf0_h.push_back(xmlf0.attribute("length").as_double());

    xmlf1 = xmlhand.child("F1");
    xmlhandf1_h.push_back(xmlf1.attribute("root_theta").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("root_phi").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("theta").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("phi").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("knuckle_mid").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("knuckle_end").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("twist").as_double());
    xmlhandf1_h.push_back(xmlf1.attribute("length").as_double());

    xmlf2 = xmlhand.child("F2");
    xmlhandf2_h.push_back(xmlf2.attribute("root_theta").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("root_phi").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("theta").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("phi").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("knuckle_mid").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("knuckle_end").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("twist").as_double());
    xmlhandf2_h.push_back(xmlf2.attribute("length").as_double());

    xmlf3 = xmlhand.child("F3");
    xmlhandf3_h.push_back(xmlf3.attribute("root_theta").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("root_phi").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("theta").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("phi").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("knuckle_mid").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("knuckle_end").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("twist").as_double());
    xmlhandf3_h.push_back(xmlf3.attribute("length").as_double());

    xmlscale = xmlhand.child("Scale");
    xmlhandscale_h = xmlscale.attribute("Scale").as_double();

}

xmlhandparser::~xmlhandparser()
 {
      insthandall.clear();
 }
