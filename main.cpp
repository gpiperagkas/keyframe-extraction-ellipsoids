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


#include <iostream>
#include <vector>
#include <exception>
#include <fstream>
#include "imagergb.h"
#include "tracker.h"
#include "eventchain.h"



using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    int ik;
    cout << "Hello World!" << endl;

    eventchain *sec= new eventchain();

    sec->buildInitChain();

    for (ik=0;ik<(sec->nini);ik++){
        sec->calcEigenValues(ik);
    }

    sec->extractKeyframes();


    sec->derivativeSEC();

    sec->constructCSEC();


   cout << "Visualize Keyframes"<< endl;
   sec->visualizeKeyframes(); //xml is written here

   cout << "Extract Video"<< endl;
   sec->extractVideo();


   //sec->printOSEC();




    return 0;
}

