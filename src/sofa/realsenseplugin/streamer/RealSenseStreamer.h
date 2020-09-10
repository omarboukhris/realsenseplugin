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

/*!
 * \brief The RealSenseStreamer class
 * Abstract streamer used as a superclass for realsensecam and realsense virtual cam (used with multicam)
 */
class RealSenseStreamer : public opencvplugin::streamer::BaseOpenCVStreamer //core::objectmodel::BaseObject
{
public :
    SOFA_CLASS( RealSenseStreamer , opencvplugin::streamer::BaseOpenCVStreamer );
    typedef opencvplugin::streamer::BaseOpenCVStreamer Inherited;

    // RGBD image data
    Data<opencvplugin::ImageData> d_color ;
    Data<opencvplugin::ImageData> d_depth ;

    // path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;

    Data<defaulttype::Vec4f > d_intrinsicParameters ;
    // decimation filter
    Data<int> d_decimation ;
    // temporal filter
    Data<float> d_tmp_alpha ;
    Data<float> d_tmp_delta ;
    DataCallback c_filters ;

    Data<std::string> d_calibpath ;
    std::vector<cv::Mat> calib_imagelist ;

    Data<int> depthScale;

    rs2::video_frame *color ;
    rs2::depth_frame *depth ;

    rs2_intrinsics cam_intrinsics ;

    RealSenseStreamer ()
        : Inherited()
        , d_color(initData(&d_color, "color", "RGB data image"))
        , d_depth(initData(&d_depth, "depth", "depth data image"))
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsics", "path to file to write realsense intrinsics into"))
        , d_intrinsicParameters(initData(&d_intrinsicParameters, "intrinsicParameters", "vector output with camera intrinsic parameters"))
        , d_decimation(initData(&d_decimation, "decimation", "decimation magnitude"))
        , d_tmp_alpha(initData(&d_tmp_alpha, "alpha", "temporal filter alpha"))
        , d_tmp_delta(initData(&d_tmp_delta, "delta", "temporal filter delta"))
        , d_calibpath(initData(&d_calibpath, std::string("./"), "calibpath", "path to folder with calibration images"))
        , depthScale(initData(&depthScale,10,"depthScale","scale for the depth values, 1 for SR300, 10 for 435"))
        , color(nullptr), depth(nullptr)
        , calib_imagelist()
    {
        d_decimation.setValue(4200);
        d_tmp_alpha.setValue(0.);
        d_tmp_delta.setValue(0.);
        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseStreamer::writeIntrinsicsToFile, this));
        c_filters.addInputs({&d_decimation, &d_tmp_alpha, &d_tmp_delta});
        c_filters.addCallback(std::bind(&RealSenseStreamer::checkFiltersParams, this));
        this->f_listening.setValue(true) ;
    }

    void checkFiltersParams () {
        int & decimation = *d_decimation.beginEdit() ;
        float &
            alpha = *d_tmp_alpha.beginEdit(),
            delta = *d_tmp_delta.beginEdit() ;
        if (decimation >= 8000) decimation = 8000 ;
        if (decimation <= 8000) decimation = 0 ;
        if ((int)(alpha*1e5) >= 10000) alpha = 1. ;
        if ((int)(alpha*1e5) <= 0) alpha = 0. ;
        if ((int)delta >= 100) delta = 100. ;
        if ((int)delta <= 0) delta = 0. ;
        d_decimation.endEdit();
        d_tmp_alpha.endEdit();
        d_tmp_delta.endEdit();
     }

    /*!
     * \brief handleEvent : Press i to push image to calibration image list, z to cancel, s to save
     * \param event
     */
    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == 'I' || ev->getKey() == 'i') {
                calib_imagelist.push_back(d_color.getValue().getImage());
            }
            else if (ev->getKey() == 'z' || ev->getKey() == 'Z') {
                if (calib_imagelist.size() > 0) calib_imagelist.pop_back();
            }
            else if (ev->getKey() == 's' || ev->getKey() == 'S') {
                // save
                const std::string path = d_calibpath.getValue() ;
                int i = 0 ;
                for (const auto & image : calib_imagelist) {
                    std::string imagepath = path + "/" + std::to_string(i) + ".png" ;
                    cv::imwrite(imagepath, image) ;
                }
            }
        }
    }

protected :

    /*!
     * \brief writeIntrinsicsToFile : intrinsics path file in sofa data needs to be set
     */
    void writeIntrinsicsToFile() {
        if (depth == nullptr) return ;
        cam_intrinsics = depth->get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;
        this->writeIntrinsics(d_intrinsics.getValue(), cam_intrinsics);
    }

    void writeIntrinsics (std::string filename, const rs2_intrinsics &cam_intrinsics) {
        // for exporting intrinsics as sofa data
        defaulttype::Vec4f intrinsics (
            cam_intrinsics.fx,
            cam_intrinsics.fy,
            cam_intrinsics.ppx, // x0
            cam_intrinsics.ppy // y0
        ) ;
        d_intrinsicParameters.setValue(intrinsics);
        std::cout << "intrinsics : " << d_intrinsicParameters.getValue() << std::endl ;

        // export intrinsics for later use (offline reproj)
        std::FILE* filestream = std::fopen(filename.c_str(), "wb") ;
        if (filestream == NULL) {
            std::cerr << "Check rights on intrins.log file" << std::endl ;
            return ;
        }
        std::fwrite(&cam_intrinsics.width, sizeof(int), 1, filestream) ;
        std::fwrite(&cam_intrinsics.height, sizeof(int), 1, filestream) ;
        std::fwrite(&cam_intrinsics.ppx, sizeof(float), 1, filestream) ;
        std::fwrite(&cam_intrinsics.ppy, sizeof(float), 1, filestream) ;
        std::fwrite(&cam_intrinsics.fx, sizeof(float), 1, filestream) ;
        std::fwrite(&cam_intrinsics.fy, sizeof(float), 1, filestream) ;
        std::fwrite(&cam_intrinsics.model, sizeof(rs2_distortion), 1, filestream) ;
        std::fwrite(cam_intrinsics.coeffs, sizeof(float), 5, filestream) ;
        std::fclose(filestream) ;
    }

    /*!
     * \brief wait_for_frame : waits for rgbd frame from realsense
     * \param pipe
     * \return realsense current frameset
     */
    rs2::frameset wait_for_frame(rs2::pipeline & pipe) {
        rs2::align align(RS2_STREAM_COLOR);
        rs2::frameset frameset;
        while (
            (!frameset.first_or_default(RS2_STREAM_DEPTH) ||
             !frameset.first_or_default(RS2_STREAM_COLOR))
        ) {
            frameset = pipe.wait_for_frames();
        }
        rs2::frameset processed = align.process(frameset);

        return processed;
    }

    /*!
     * @brief convert RGB & D rs2::frames to cv::Mat and stores them in data container
     */
    void frame_to_cvmat(rs2::video_frame color, rs2::depth_frame depth,
                        cv::Mat& bgr_image, cv::Mat& depth8) {
        int widthc = color.get_width();
        int heightc = color.get_height();

//        cv::Mat rgb0(heightc,widthc, CV_8UC3, (void*) color.get_data()) ;
//        cv::cvtColor (rgb0, bgr_image, cv::COLOR_RGB2BGR); // bgr_image is output

        bgr_image = cv::Mat(heightc,widthc, CV_8UC3, (void*) color.get_data()) ;

        int widthd = depth.get_width();
        int heightd = depth.get_height();
        cv::Mat depth16 = cv::Mat(heightd, widthd, CV_16U, (void*)depth.get_data()) ;
        depth16.convertTo(depth8, CV_8U, 1.f/64*depthScale.getValue()); //depth32 is output
    }
} ;

}

}

