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

#include "RealSenseStreamer.h"

namespace sofa
{

namespace rgbdtracking
{

using namespace cimg_library;
using defaulttype::Vec;
using defaulttype::Vector3;


using namespace std;
using namespace cv;
using namespace boost;
using namespace rs2;

class RealSenseCam : public RealSenseStreamer //core::objectmodel::BaseObject
{
public:
    SOFA_CLASS( RealSenseCam , RealSenseStreamer );
    typedef RealSenseStreamer Inherited;

    Data<int> depthMode;

    // RGBD image data
//    Data<opencvplugin::ImageData> d_color ;
//    Data<opencvplugin::ImageData> d_depth ;

    // path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;

    rs2_intrinsics cam_intrinsics ;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // for pointcloud extraction
    rs2::pointcloud pc ;
    rs2::points points ;

//    rs2::video_frame *color ;
//    rs2::depth_frame *depth ;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Start streaming with default recommended configuration

    RealSenseCam()
        : Inherited()
        , depthMode ( initData ( &depthMode,1,"depthMode","depth mode" ))
//        , d_color(initData(&d_color, "color", "RGB data image"))
//        , d_depth(initData(&d_depth, "depth", "depth data image"))
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsics", "path to file to write realsense intrinsics into"))
    {
        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseCam::writeIntrinsicsToFile, this));

        this->f_listening.setValue(true) ;
        //color = nullptr ;
        //depth = nullptr ;
    }

    ~RealSenseCam () {
    }

    void init() {
        initAlign();
    }

    void writeIntrinsicsToFile() {
        this->writeIntrinsics(d_intrinsics.getValue(), cam_intrinsics);
    }

    void decodeImage(cv::Mat & /*img*/) {
        acquireAligned();
    }

protected:

    void initAlign() {
        //rs2::pipeline_profile selection =
        pipe.start();

        rs2::frameset frameset = wait_for_frame(pipe);

        // Trying to get both color and aligned depth frames
        color = new rs2::video_frame(frameset.get_color_frame()) ;
        depth = new rs2::depth_frame(frameset.get_depth_frame()) ;

        // fetch and save intrinsics to specified file
        cam_intrinsics = depth->get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;
        writeIntrinsics(d_intrinsics.getValue().c_str(), cam_intrinsics);

        // extract pointcloud
        //getpointcloud(*color, *depth) ;

        // Create depth and color image
        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
        d_color.endEdit(); d_depth.endEdit();
    }

    void acquireAligned() {
        rs2::frameset frameset = wait_for_frame(pipe) ;

        // Trying to get both color and aligned depth frames
        if (color) delete color ;
        if (depth) delete depth ;
        color = new rs2::video_frame(frameset.get_color_frame()) ;
        depth = new rs2::depth_frame(frameset.get_depth_frame()) ;

        // extract pointcloud
        //getpointcloud(*color, *depth) ;

        // Create depth and color image
        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
        d_color.endEdit(); d_depth.endEdit();
    }

protected :
    void getpointcloud (rs2::frame color, rs2::frame depth) {
        if (color) {
            pc.map_to (color) ;
        }
        points = pc.calculate(depth) ;
    }
};

}

}

