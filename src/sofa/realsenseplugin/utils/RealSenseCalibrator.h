/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

//#include <opencv2/ccalib/randpattern.hpp>

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/OptionsGroup.h>

#include <sofa/opencvplugin/OpenCVWidget.h>
#include <sofa/opencvplugin/BaseOpenCVStreamer.h>

#include <librealsense2/rs.hpp>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/realsenseplugin/cv-helpers.hpp>

#include <exception>

namespace sofa
{

namespace rgbdtracking
{

static void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
    corners.resize(0);

    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(cv::Point3f(float(j*squareSize),
                                      float(i*squareSize), 0));
}

class RealSenseCalibrator : public core::objectmodel::BaseObject
{
public :

    typedef core::objectmodel::BaseObject Inherited;
    SOFA_CLASS( RealSenseCalibrator, Inherited);

    RealSenseCalibrator()
        : Inherited()
    {
//        cv::Mat view, viewGray;
        cv::Mat cameraMatrix[2], distCoeffs[2], R[2], P[2], R12, T12;
        for(int  k = 0; k < 2; k++ ) {
            cameraMatrix[k] = cv::Mat_<double>::eye(3,3);
            cameraMatrix[k].at<double>(0,0) = 3./4.;
            cameraMatrix[k].at<double>(1,1) = 1.;
            distCoeffs[k] = cv::Mat_<double>::zeros(5,1);
        }
    }

//    init 3 cam matrices

// get shessboard corners
//{
//    int k1 = k == 0 ? 2 : k == 1 ? 0 : 1;
//    printf("%s\n", imageList[i*3+k].c_str());
//    cv::Mat view = imread(imageList[i*3+k], 1);

//    cv::Size boardSize ;
//    if(!view.empty())
//    {
//        vector<Point2f> ptvec;
//        cv::Size imageSize = view.size();
//        cvtColor(view, viewGray, COLOR_BGR2GRAY);
//        bool found = cv::findChessboardCorners( view, boardSize, ptvec, CALIB_CB_ADAPTIVE_THRESH );

//        cv::drawChessboardCorners( view, boardSize, Mat(ptvec), found );
//        if( found )
//        {
//            imgpt[k1][i].resize(ptvec.size());
//            std::copy(ptvec.begin(), ptvec.end(), imgpt[k1][i].begin());
//        }
//        //imshow("view", view);
//        //int c = waitKey(0) & 255;
//        //if( c == 27 || c == 'q' || c == 'Q' )
//        //    return -1;
//    }
//}



};

}

}

