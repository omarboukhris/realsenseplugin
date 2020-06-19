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

    Data<helper::fixed_array<double, 5> > d_intrinsicParameters ;

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
        , depthScale(initData(&depthScale,10,"depthScale","scale for the depth values, 1 for SR300, 10 for 435"))
        , color(nullptr), depth(nullptr)
    {
        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseStreamer::writeIntrinsicsToFile, this));
    }

protected :

    void writeIntrinsicsToFile() {
        if (depth == nullptr) return ;
        cam_intrinsics = depth->get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;
        this->writeIntrinsics(d_intrinsics.getValue(), cam_intrinsics);
    }

    void writeIntrinsics (std::string filename, const rs2_intrinsics &cam_intrinsics) {
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

        // for printing intrinsics
        helper::fixed_array<double, 5> intrinsics (
            cam_intrinsics.fx,
            cam_intrinsics.fy,
            0.0, // s ?
            cam_intrinsics.ppx, // x0
            cam_intrinsics.ppy // y0
        ) ;
        d_intrinsicParameters.setValue(intrinsics);
        //std::cout << "intrinsics : " << intrinsics << std::endl ;
    }

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

