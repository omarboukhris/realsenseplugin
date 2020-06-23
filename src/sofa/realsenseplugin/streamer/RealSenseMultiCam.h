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

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

namespace sofa
{

namespace rgbdtracking
{


class RealSenseVirtualCam : public RealSenseStreamer {
public:
    typedef RealSenseStreamer Inherited;
    SOFA_CLASS( RealSenseVirtualCam , Inherited);

    rs2::pipeline pipe ;

    std::string serial ;

    RealSenseVirtualCam()
        : Inherited()
    {}

    RealSenseVirtualCam(std::string serialnum)
        : Inherited()
        , serial(serialnum)
    {
    }

    void init() {
        if (serial.empty()) {
            std::cout << "(RealSenseVirtualCam) cam id is invalid" << std::endl ;
            return ;
        }
        rs2::config cfg ;
        cfg.enable_device(serial);
        pipe.start(cfg) ;
    }

    void decodeImage(cv::Mat & /*img*/) {
        rs2::frameset frameset = wait_for_frame(pipe) ;

        if (color) delete color ;
        if (depth) delete depth ;
        color = new rs2::video_frame (frameset.get_color_frame()) ;
        depth = new rs2::depth_frame (frameset.get_depth_frame()) ;

        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
        d_color.endEdit(); d_depth.endEdit();

    }

    std::vector<cv::Mat> calib_imagelist ;
    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == 'I' || ev->getKey() == 'i') {
                calib_imagelist.push_back(d_color.getValue().getImage());
            }
            if (ev->getKey() == 'z' || ev->getKey() == 'Z') {
                calib_imagelist.pop_back();
            }
        }
    }

} ;

class RealSenseMultiCam : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherited;
    SOFA_CLASS( RealSenseMultiCam , Inherited);

    RealSenseMultiCam()
        : Inherited()
    {
        this->f_listening.setValue(true);
    }

    std::vector<std::string> listSerialNum()
    {
        std::vector<std::string> serial_list ;
        rs2::context ctx ;
        for (auto && device : ctx.query_devices()) {
            serial_list.push_back(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
        return serial_list ;
    }

    void init () {
        auto serial_list = listSerialNum();

        core::objectmodel::BaseContext::SPtr context = this->getContext() ;

        size_t i = 0 ;
        for (std::string serial : serial_list) {
            add_realsenseCam(context, serial, ++i);
        }
    }

    void add_realsenseCam (core::objectmodel::BaseContext::SPtr node, std::string serial, size_t i) {
        RealSenseVirtualCam* rs_vcam = new RealSenseVirtualCam(serial) ;
        rs_vcam->setName(std::string("RSCam_") + std::to_string(i));
        node->addObject(rs_vcam) ;

        using namespace opencvplugin::scheduler ;
        static OpenCVScheduler::SPtr scheduler = new OpenCVScheduler ;
        scheduler->setName(std::string("scheduler_rs") + std::to_string(i));
        scheduler->addStreamer (rs_vcam) ;
        node->addObject(scheduler) ;
    }

} ;

}

}

