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
#include <sofa/opencvplugin/scheduler/OpenCVScheduler.h>

#include <SofaSimulationCommon/common.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#include <SofaSimulationTree/init.h>
#include <SofaSimulationTree/TreeSimulation.h>

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

    rs2::pipeline * pipe ;

    // path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;

    RealSenseVirtualCam()
        : Inherited(), pipe(nullptr)
    {}

    RealSenseVirtualCam(rs2::pipeline * pipe_param, std::string name)
        : Inherited(), pipe (pipe_param)
    {
        this->setName(name);
        this->f_listening.setValue(true);
        init() ;
    }

    void init() {
        cv::Mat placeholder ;
        decodeImage(placeholder);
        writeIntrinsicsToFile();
    }

    void decodeImage(cv::Mat & /*img*/) {
        rs2::frameset frameset = wait_for_frame(*pipe) ;
        if (color) delete color ;
        if (depth) delete depth ;
        color = new rs2::video_frame (frameset.get_color_frame()) ;
        depth = new rs2::depth_frame (frameset.get_depth_frame()) ;
        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
        d_color.endEdit(); d_depth.endEdit();
    }

} ;


class RealSenseMultiCam : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherited;
    SOFA_CLASS( RealSenseMultiCam , Inherited);

    std::vector<rs2::pipeline> pipelines ;

    std::vector<RealSenseVirtualCam::SPtr> realsense_cams ;

    RealSenseMultiCam()
        : Inherited()
        , pipelines()
    {
        preparePipes();
    }

protected:
    void preparePipes () {
        // this method should be called onl once at most
//        static bool once = false ;
//        if (once) {
//            return ;
//        }
//        once = true ;
        pipelines.clear();
        rs2::context ctx ;
        for (auto && device : ctx.query_devices()) {
            rs2::pipeline pipe (ctx) ;
            rs2::config cfg ;
            cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            pipe.start(cfg) ;
            pipelines.push_back(pipe) ;
            addRealsenseObject(pipe, device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
    }

    void addRealsenseObject(rs2::pipeline & pipe, const char * serialnum) {
        std::string name = std::string("realsense-") + std::string(serialnum) ;
        std::cout << name << std::endl ;
        RealSenseVirtualCam* realsense_cam = new RealSenseVirtualCam(&pipe, name) ;
        opencvplugin::scheduler::OpenCVScheduler* sched = new opencvplugin::scheduler::OpenCVScheduler() ;
        sched->l_streamer.add(realsense_cam) ;
        // get ptr on parent node and create opencvScheduler
        // then link streamer on created realsense virtual cam
//        simulation::Node* parentNode = static_cast<simulation::Node*>(this->getContext()) ;
//        parentNode->addObject(realsense_cam) ;
//        parentNode->addObject(sched) ;
        // push to list (if ever needed)
        realsense_cams.push_back(realsense_cam);
    }

} ;

}

}

