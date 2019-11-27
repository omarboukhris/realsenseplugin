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

class RealSenseMultiCam : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherited;
    SOFA_CLASS( RealSenseMultiCam , Inherited);

    std::vector<rs2::pipeline*> pipelines ;

    RealSenseMultiCam()
        : Inherited()
        , pipelines()
    {
        preparePipes();
    }

    void preparePipes () {
        pipelines.clear();
        rs2::context ctx ;
        for (auto && device : ctx.query_devices()) {
            rs2::pipeline pipe (ctx) ;
            rs2::config cfg ;
            cfg.enable_device(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
            pipe.start(cfg) ;
            pipelines.push_back(&pipe) ;
        }
    }

} ;


class RealSenseVirtualCam : public RealSenseStreamer {
public:
    typedef RealSenseStreamer Inherited;
    SOFA_CLASS( RealSenseVirtualCam , Inherited);

    rs2::pipeline * pipe ;

    Data<int> d_camindex ;

    core::objectmodel::SingleLink<
        RealSenseVirtualCam,
        RealSenseMultiCam,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_rs_cam ; //for intrinsics

    RealSenseVirtualCam()
        : Inherited(), pipe(nullptr)
        , d_camindex(initData(&d_camindex, 0, "camid", "camera id"))
        , l_rs_cam(initLink("rsmulticam", "link to RealSenseMulticam"))
    {
        setupPipe();
    }

    void setupPipe() {
        if (!l_rs_cam) {
            std::cerr << "(RealSenseVirtualCam) link to multicam invalid" << std::endl ;
            return ;
        }
        std::cout << "(RealSenseVirtualCam) link to multicam is valid" << std::endl ;
        l_rs_cam->preparePipes() ;
        std::vector<rs2::pipeline*> & pipelines = l_rs_cam->pipelines ;
        int camid = d_camindex.getValue() ;
        if (camid >= pipelines.size()) {
            std::cerr << "(RealSenseVirtualCam) invalid cam id : "
                      << camid << " not in [0.."
                      << pipelines.size() << "]"
                      << std::endl ;
            return ;
        }
        std::cout << "(RealSenseVirtualCam) cam id is valid" << std::endl ;
        pipe = pipelines.at(camid) ;
        std::cout << "(RealSenseVirtualCam) pipe is set" << std::endl ;
    }

    void decodeImage(cv::Mat & /*img*/) {
        if (pipe == nullptr) {
            setupPipe();
            return ;
        }
        rs2::frameset frameset = wait_for_frame(*pipe) ;
        if (color) delete color ;
        if (depth) delete depth ;
        color = new rs2::video_frame (frameset.get_color_frame()) ;
        depth = new rs2::depth_frame (frameset.get_depth_frame()) ;
        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
        d_color.endEdit(); d_depth.endEdit();
    }

} ;

}

}

