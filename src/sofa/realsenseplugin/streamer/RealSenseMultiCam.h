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

#include <sofa/realsenseplugin/streamer/RealSenseStreamer.h>

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

class RealSenseVirtualCam : public RealSenseStreamer {
public:
    typedef RealSenseStreamer Inherited;
    SOFA_CLASS( RealSenseVirtualCam , Inherited);

//    core::objectmodel::SingleLink<
//        RealSenseVirtualCam,
//        RealSenseMultiCam,
//        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
//    > l_rs_cam ; //for intrinsics

    rs2::video_frame *color ;
    rs2::depth_frame *depth ;

    // RGBD image data
//    Data<uint> d_cam_id ;
//    DataCallback c_cam_id ;

    // path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;

    RealSenseVirtualCam() {
    }

    RealSenseVirtualCam(rs2::video_frame * col, rs2::depth_frame * dep, std::string & name)
        : Inherited()
        , color(col), depth(dep)
//        , d_cam_id(initData(&d_cam_id, "cam_id", "rs camera id for frame processing"))
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsics", "path to file to write realsense intrinsics into"))
//        , l_rs_cam(initLink("rs_multicam", "link to rs multicam sofa component"))
    {
        this->setName(name) ;
//        c_cam_id.addInputs({&d_cam_id});
//        c_cam_id.addCallback(std::bind(&RealSenseVirtualCam::updateCamId, this));

        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseVirtualCam::writeIntrinsicsToFile, this));
    }

    void updateCamId () {
//        if (!l_rs_cam) {
//            std::cerr << "(RealSenseCam) Link to multicam not valid"
//                      << std::endl ;
//            return ;
//        }
//        uint cam_id = d_cam_id.getValue() ;
//        if (cam_id >= l_rs_cam->pipelines.size()) {
//            std::cerr << "(RealSenseCam) cam id not supported"
//                      << std::endl ;
//            return ;
//        }
//        color = new rs2::video_frame(l_rs_cam->colors[cam_id]) ;
//        depth = new rs2::depth_frame(l_rs_cam->depths[cam_id]) ;
    }

    void decodeImage(cv::Mat & /*img*/) {
//        uint cam_id = d_cam_id.getValue() ;
//        if (color) delete color ;
//        if (depth) delete depth ;
//        color = new rs2::video_frame(l_rs_cam->colors[cam_id]) ;
//        depth = new rs2::depth_frame(l_rs_cam->depths[cam_id]) ;
        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
        d_color.endEdit(); d_depth.endEdit();
    }

    void writeIntrinsicsToFile() {
//        rs2_intrinsics cam_intrinsics = depth->get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;
//        writeIntrinsics(d_intrinsics.getValue(), cam_intrinsics);
    }

} ;


class RealSenseMultiCam : public RealSenseStreamer
{
public:
    typedef RealSenseStreamer Inherited;
    SOFA_CLASS( RealSenseMultiCam , Inherited);

    std::vector<rs2::pipeline> pipelines ;

    std::vector<rs2::video_frame*> colors ;
    std::vector<rs2::depth_frame*> depths ;

    std::vector<RealSenseVirtualCam::SPtr> realsense_cams ;

    RealSenseMultiCam()
        : Inherited()
        , pipelines()
    {
        preparePipes();
    }

    void init () {
    }

protected:
    void preparePipes () {
        static bool once = false ;
        if (once) {
            return ;
        }
        once = true ;
        pipelines.clear();
        rs2::context ctx ;
        for (auto && device : ctx.query_devices()) {
            rs2::pipeline pipe (ctx) ;
            rs2::config cfg ;
            cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            pipe.start(cfg) ;
            pipelines.push_back(pipe) ;
        }
    }

    void addRealsenseSlave(int & i, rs2::video_frame & color, rs2::depth_frame & depth)
    {
        std::string name = std::string("realsense") + std::to_string(i) ;
        RealSenseVirtualCam::SPtr rsvc (new
            RealSenseVirtualCam(
                &color,
                &depth,
                name)) ;
        rsvc->init();
        this->addSlave(rsvc);
        realsense_cams.push_back(rsvc);
    }

    void decodeImage(cv::Mat & /*img*/) {
        colors.clear(); depths.clear(); int i = 0 ;
        static bool once = false ;
        for (rs2::pipeline & pipe : pipelines) {
            rs2::frameset frameset = wait_for_frame(pipe) ;
            rs2::video_frame color (frameset.get_color_frame()) ;
            rs2::depth_frame depth (frameset.get_depth_frame()) ;

            if (!once) {
                addRealsenseSlave(i, color, depth);
            } else {
                realsense_cams[i]->depth = new rs2::depth_frame(depth) ;
                realsense_cams[i]->color = new rs2::video_frame(color) ;
            }
            i++ ;
        }
        once = true ;
    }

} ;

}

}

