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

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/OptionsGroup.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <librealsense2/rs.hpp>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/realsenseplugin/streamer/RealSenseCam.h>
#include <sofa/realsenseplugin/cv-helpers.hpp>

namespace sofa
{

namespace rgbdtracking
{
/// /!\ See https://github.com/IntelRealSense/librealsense/blob/master/wrappers/opencv/grabcuts/rs-grabcuts.cpp
/// for implementation details

/*!
 * \brief The RealSenseGrabCut class
 * Applies grabcut on RGBD frames
 * ROI's BBOX is defined by contour
 */
class RealSenseGrabCut : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( RealSenseGrabCut, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    /// \brief input color image
    Data<opencvplugin::ImageData> d_image_in ;
    /// \brief output color image
    Data<opencvplugin::ImageData> d_image_out ;

    /// \brief input depth image
    Data<opencvplugin::ImageData> d_depth_in ;
    /// \brief output depth image
    Data<opencvplugin::ImageData> d_depth_out ;

    /// \brief near threshold
    Data<int> d_near_thr ;
    /// \brief far threshold
    Data<int> d_far_thr ;

    /// \brief contour from which to extract ROI BBox
    Data<helper::vector<defaulttype::Vector2> > d_contour;

    DataCallback c_image_in ;
    DataCallback c_contour ;

    // bounding box rectangle
    cv::Rect rect ;
    std::vector<cv::Point2f> m_contour ;

    RealSenseGrabCut()
        : d_image_in(initData(&d_image_in, "in", "input data image"))
        , d_image_out(initData(&d_image_out, "out", "output data image"))
        , d_depth_in(initData(&d_depth_in, "din", "input data image"))
        , d_depth_out(initData(&d_depth_out, "dout", "output data image"))
        , d_near_thr(initData(&d_near_thr, 200, "nearthr", "threshold value for near mask"))
        , d_far_thr(initData(&d_far_thr, 80, "farthr", "threshold value for far mask"))
        , d_contour( initData(&d_contour, "contour", "contour to compute ROI rectangle"))
        , m_contour()
    {
        c_image_in.addInputs({&d_image_in, &d_far_thr, &d_near_thr});
        c_image_in.addCallback(std::bind(&RealSenseGrabCut::realsense_grabcut, this));
        c_contour.addInputs({&d_contour});
        c_contour.addCallback(std::bind(&RealSenseGrabCut::updateContour, this));
    }

    /*!
     * \brief updateContour updates contour to appropriate data format
     */
    void updateContour () {
        const helper::vector<defaulttype::Vector2> contourdata = d_contour.getValue() ;
        m_contour.clear();
        for (defaulttype::Vector2 point : contourdata) {
            m_contour.push_back(cv::Point2f(point[0], point[1]));
        }
    }

    /*!
     * \brief realsense_grabcut
     * creates thresholding masks, computes bounding box from contour,
     * applies grabcut then applies mask to input color and depth frames
     */
    void realsense_grabcut () {
        if (d_depth_in.getValue().getImage().empty()) {
            std::cerr << "(RSGrabCut) depth frame is not setup" << std::endl ;
            return ;
        }
        if (d_contour.getValue().size() < 4) {
            std::cerr << "(RSGrabCut) check contour size" << std::endl ;
            return ;
        }
        cv::Mat near = d_depth_in.getValue().getImage() ;
        cv::cvtColor(near, near, cv::COLOR_BGR2GRAY);
        create_mask_from_depth(near, d_near_thr.getValue(), cv::THRESH_BINARY);

        // get far image mask
        cv::Mat far = d_depth_in.getValue().getImage() ;
        cv::cvtColor(far, far, cv::COLOR_BGR2GRAY) ;
        far.setTo(255, far==0) ;
        create_mask_from_depth(far, d_far_thr.getValue(), cv::THRESH_BINARY_INV);

        // create mask
        cv::Mat mask;
        mask.create(near.size(), CV_8UC1);
        mask.setTo(cv::Scalar::all(cv::GC_BGD)) ; // Set "background" as default guess
        mask.setTo(cv::GC_PR_BGD, far==0) ; // Relax this to "probably background" for pixels outside "far" region
        mask.setTo(cv::GC_PR_FGD, near==255) ; // Set pixels within the "near" region to "foreground"

        // Run Grab-Cut algorithm:
        cv::Mat bgModel, fgModel,
            color_mat = d_image_in.getValue(),
            depth_mat = d_depth_in.getValue() ;

        // get ROI
        rect = cv::boundingRect(m_contour) ;

        cv::grabCut(
            color_mat,
            mask,
            rect,
            bgModel,
            fgModel,
            1,
            cv::GC_INIT_WITH_MASK
        );


        // Extract foreground pixels based on refined mask from the algorithm
        cv::Mat maskimg , & imageDest = *d_image_out.beginEdit(), imgtmp, imgtmp2, & depthDest = *d_depth_out.beginEdit() ;
        cv::compare(mask,cv::GC_PR_FGD,maskimg,cv::CMP_EQ);

        if (depth_mat.size() == maskimg.size()) {
            depth_mat.copyTo(imgtmp2, maskimg);
            depthDest = maskimg.clone() ;
        }

        color_mat.copyTo(imgtmp, maskimg); // imageDest is the output
        imageDest = imgtmp.clone() ;
//        cv::rectangle(imageDest, rect, cv::Scalar(255,0,0,1)) ;

        d_image_out.endEdit();
        d_depth_out.endEdit();
    }

protected :

    /*!
     * \brief create_mask_from_depth applies thresholding then dilation and erosion
     * \param depth depth cv::mat
     * \param thresh
     * \param type
     */
    void create_mask_from_depth (cv::Mat& depth, int thresh, cv::ThresholdTypes type) {
        #define EROSION_KERNEL_SIZE cv::Size(3, 3)
        cv::Mat
            erode_less = cv::getStructuringElement(cv::MORPH_RECT, EROSION_KERNEL_SIZE),
            erode_more = cv::getStructuringElement(cv::MORPH_RECT, EROSION_KERNEL_SIZE*2) ;
        cv::threshold(depth, depth, thresh, 255, type);
        cv::dilate(depth, depth, erode_less);
        cv::erode(depth, depth, erode_more);
    }
} ;

}

}

