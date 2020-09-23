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
#include <sofa/helper/system/FileSystem.h>

#include <sofa/opencvplugin/OpenCVWidget.h>
#include <sofa/realsenseplugin/utils/MultiCamCalibrator.h>
#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/core/types_c.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

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

class MultiCamLiveCalibrator : public MultiCamCalibrator
{
public :

    typedef MultiCamCalibrator Inherited;
    SOFA_CLASS( MultiCamLiveCalibrator, Inherited);

    Data<opencvplugin::ImageData> d_img1 ;
    Data<opencvplugin::ImageData> d_img2 ;
    DataCallback callback_img ;
    Data<opencvplugin::ImageData> d_img1out ;
    Data<opencvplugin::ImageData> d_img2out ;

    MultiCamLiveCalibrator()
        : Inherited()
        // input images
        , d_img1(initData(&d_img1, "imgmaster", "images from camera 1"))
        , d_img2(initData(&d_img2, "imgslave", "images from camera 2"))
        // highligthed output images
        , d_img1out(initData(&d_img1out, "imgcorners1", "image from camera 1 with highlighted corners"))
        , d_img2out(initData(&d_img2out, "imgcorners2", "image from camera 2 with highlighted corners"))
    {
        callback_img.addInputs({&d_img1}) ;
        callback_img.addCallback(std::bind(&MultiCamLiveCalibrator::checkForCorners, this)) ;
    }

    void checkForCorners() {
        defaulttype::Vector2 bsize = d_chessboardsize.getValue() ;
        cv::Size boardsize = cv::Size(bsize[0],bsize[1]) ;

        std::vector<cv::Point2f> corners1, corners2 ;
        cv::Mat img1 = d_img1.getValue().getImage(),
                img2 = d_img2.getValue().getImage(),
                grey1, grey2 ;
        if (img1.rows*img1.cols == 0 || img2.rows*img2.cols == 0 ) {
            return ;
        }
        cv::cvtColor(img1, grey1, cv::COLOR_BGR2GRAY) ;
        cv::cvtColor(img2, grey2, cv::COLOR_BGR2GRAY) ;
        bool gotCorners = getCorners(boardsize, grey1, corners1, grey2, corners2);
        if (gotCorners) {
            std::cout << "3a" << corners1.size() << " " << corners2.size() << std::endl ;
            calibimage1.push_back(grey1);
            calibimage2.push_back(grey2);
            cv::drawChessboardCorners(grey1,boardsize,corners1,true) ;
            cv::drawChessboardCorners(grey2,boardsize,corners2,true) ;
            d_img1out.setValue(grey1);
            d_img2out.setValue(grey2);
        }
    }

    void handleEvent(sofa::core::objectmodel::Event* event) {
        if (core::objectmodel::KeypressedEvent* ev = dynamic_cast<core::objectmodel::KeypressedEvent*>(event)) {
            if (ev->getKey() == 'c'||ev->getKey() == 'C') {
                process();
            }
        }
    }

};

}

}

