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

#ifndef XMLHANDPARSER_H
#define XMLHANDPARSER_H
#include <stdio.h>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <exception>
#include <sstream>
#include <vector>
#include "pugixml-1.7/src/pugixml.hpp"
#include "pugixml-1.7/src/pugiconfig.hpp"


class xmlhandparser
{
private:
    int i,j, side, nkf;
    pugi::xml_document doc;
    pugi::xml_node keyframe;
    std::vector<double> insthandall;
    std::string xmlname;
    pugi::xml_document indoc;
    pugi::xml_parse_result result;
    pugi::xml_node hand;
    pugi::xml_node pos;
    pugi::xml_node orient;
    pugi::xml_node wrist;
    pugi::xml_node thumb;
    pugi::xml_node f0;
    pugi::xml_node f1;
    pugi::xml_node f2;
    pugi::xml_node f3;
    pugi::xml_node scale;
    pugi::xml_node xmlkeyframe;

public:
    std::string xmlkfid_h;
    int xmlkfidx_h;
    double xmlkft_h;
    std::string xmlkfxmlns_h;
    std::string xmlkfxsischemaloc_h;
    std::string xmlhandid_h;
    std::string xmlhandname_h;
    std::vector<double> xmlhandpos_h;
    std::vector<double> xmlhandorient_h;
    std::vector<double> xmlhandwrist_h;
    std::vector<double> xmlhandthumb_h;
    std::vector<double> xmlhandf0_h;
    std::vector<double> xmlhandf1_h;
    std::vector<double> xmlhandf2_h;
    std::vector<double> xmlhandf3_h;
    double xmlhandscale_h;
    xmlhandparser(int keyframenr, float *coeff_hand, int lr);
    xmlhandparser(std::string xmlinfile);
    void setKeyFrame(int idx, double timestamp);
    void setHandpars();
    void setClosingData(int nrfr);
    void getHandPars();
    ~xmlhandparser();

};

#endif // XMLHANDPARSER_H
