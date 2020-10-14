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

#include <sofa/realsenseplugin/calib/MultiCamCalibrator.h>

#include <sofa/realsenseplugin/RSData.h>

namespace sofa
{

namespace realsenseplugin
{

/*!
 * \brief The MultiCamLiveCalibrator class
 * Doesn't need saved images to calibrate, just plug in stream of images from camera to calibrate
 */
class MultiCamLiveCalibrator : public MultiCamCalibrator
{
public :

    typedef MultiCamCalibrator Inherited;
    SOFA_CLASS( MultiCamLiveCalibrator, Inherited);

    /// \brief image from camera 1
    Data<RealSenseDataFrame> d_img1 ;
    /// \brief image from camera 2
    Data<RealSenseDataFrame> d_img2 ;
    core::objectmodel::DataCallback callback_img ;
    /// \brief image from camera 1 with corners
    Data<RealSenseDataFrame> d_img1out ;
    /// \brief image from camera 2 with corners
    Data<RealSenseDataFrame> d_img2out ;

    MultiCamLiveCalibrator()
        : Inherited()
        // input images
        , d_img1(initData(&d_img1, "imgmaster", "images from camera 1"))
        , d_img2(initData(&d_img2, "imgslave", "images from camera 2"))
        // highligthed output images
        , d_img1out(initData(&d_img1out, "imgcorners1", "image from camera 1 with highlighted corners"))
        , d_img2out(initData(&d_img2out, "imgcorners2", "image from camera 2 with highlighted corners"))
        , activated(true)
    {
        callback_img.addInputs({&d_img1}) ;
        callback_img.addCallback(std::bind(&MultiCamLiveCalibrator::checkForCorners, this)) ;
        calibimage1.clear();
        calibimage2.clear();
    }

    /*!
     * \brief checkForCorners
     * check if streamed images can be coupled for stereo calibration
     * and affect them to appropriate lists for later processing
     */
    void checkForCorners() {
        if (calibimage1.size() == 30 || !activated) {
            activated = false ;
            return ; // 30 should be enough, can be changed
        }
        defaulttype::Vector2 bsize = d_chessboardsize.getValue() ;
        cv::Size boardsize = cv::Size(bsize[0],bsize[1]) ;

        std::vector<cv::Point2f> corners1, corners2 ;
        cv::Mat img1 = d_img1.getValue().getcvColor(),
                img2 = d_img2.getValue().getcvColor(),
                grey1, grey2 ;
        if (img1.rows*img1.cols == 0 || img2.rows*img2.cols == 0 ) {
            return ;
        }
        cv::cvtColor(img1, grey1, cv::COLOR_BGR2GRAY) ;
        cv::cvtColor(img2, grey2, cv::COLOR_BGR2GRAY) ;
        bool gotCorners = getCorners(boardsize, grey1, corners1, grey2, corners2);
        if (gotCorners) {
            calibimage1.push_back(grey1);
            calibimage2.push_back(grey2);
            cv::drawChessboardCorners(grey1,boardsize,corners1,true) ;
            cv::drawChessboardCorners(grey2,boardsize,corners2,true) ;
            d_img1out.setValue(RealSenseDataFrame(grey1, grey1));
            d_img2out.setValue(RealSenseDataFrame(grey2, grey2));
        }
    }

    /*!
     * \brief handleEvent Press C to do calibration
     * \param event
     */
    void handleEvent(sofa::core::objectmodel::Event* event) {
        if (core::objectmodel::KeypressedEvent* ev = dynamic_cast<core::objectmodel::KeypressedEvent*>(event)) {
            if (ev->getKey() == 'c'||ev->getKey() == 'C') {
                process();
            }
        }
    }

private:
    bool activated ;
};

}

}

