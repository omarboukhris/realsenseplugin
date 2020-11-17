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

#include <sofa/realsenseplugin/calib/mono/MonoCalibrator.h>

namespace sofa
{

namespace realsenseplugin
{

/*!
 * \brief The MultiCamLiveCalibrator class
 * Doesn't need saved images to calibrate, just plug in stream of images from camera to calibrate
 */
class MonoCamLiveCalibrator : public MonoCalibrator
{
public :

    typedef MonoCalibrator Inherited;
    SOFA_CLASS( MonoCamLiveCalibrator, Inherited);

    /// \brief image from camera 1
    Data<RealSenseDataFrame> d_img1 ;
    core::objectmodel::DataCallback callback_img ;
    /// \brief image from camera 1 with corners
    Data<RealSenseDataFrame> d_img1out ;

    MonoCamLiveCalibrator()
        : Inherited()
        // input images
        , d_img1(initData(&d_img1, "imgmaster", "images from camera 1"))
        // highligthed output images
        , d_img1out(initData(&d_img1out, "imgcorners1", "image from camera 1 with highlighted corners"))
        , activated(true)
    {
        callback_img.addInputs({&d_img1}) ;
        callback_img.addCallback(std::bind(&MonoCamLiveCalibrator::checkForCorners, this)) ;
        calibimage1.clear();
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

        std::vector<cv::Point2f> corners1 ;
        cv::Mat img1 = d_img1.getValue().getcvColor(),
                grey1 ;
        if (img1.rows*img1.cols == 0) {
            return ;
        }
        cv::cvtColor(img1, grey1, cv::COLOR_BGR2GRAY) ;
        bool gotCorners = getCorners(boardsize, grey1, corners1);
        if (gotCorners) {
            calibimage1.push_back(grey1);
            cv::drawChessboardCorners(grey1,boardsize,corners1,true) ;
            d_img1out.setValue(RealSenseDataFrame(grey1, grey1));
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

