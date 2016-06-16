/*
 * Copyright (C) 2016 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

#include "ipopt/eyesCalibration.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;


/*******************************************************/
CalibrationData &EyesCalibration::addData()
{
    CalibrationData d;
    data.push_back(d);
    return data.back();
}


/*******************************************************/
bool EyesCalibration::runCalibration(Matrix &extrinsics_left,
                                     Matrix &extrinsics_right)
{
    return true;
}


