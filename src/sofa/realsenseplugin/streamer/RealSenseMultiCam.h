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
#include <sofa/sofascheduler/scheduler/GenericScheduler.h>

#include <SofaSimulationCommon/common.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/config.h> // #defines SOFA_HAVE_DAG (or not)
#include <SofaSimulationTree/init.h>
#include <SofaSimulationTree/TreeSimulation.h>
#include <sofa/helper/system/FileSystem.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

namespace sofa
{

namespace realsenseplugin
{


/*!
 * \brief The RealSenseVirtualCam class
 * virtual cam instanciated from multicam component
 */
class RealSenseVirtualCam : public RealSenseStreamer {
public:
    typedef RealSenseStreamer Inherited;
    SOFA_CLASS( RealSenseVirtualCam , Inherited);

    rs2::pipeline pipe ;

    int serial ;

    RealSenseVirtualCam()
        : Inherited()
        , serial (0)
    {}

    RealSenseVirtualCam(int serialnum)
        : Inherited()
        , serial(serialnum)
    {
    }

    void init() {
        std::vector<std::string> seriallist = listSerialNum() ;
        if (serial < 0 || serial >= seriallist.size()) {
            std::cout << "(RealSenseCam) serial number index invalid <" << serial << "> "
                      << "should be between [0.." << seriallist.size() << "]" << std::endl ;
        }
        rs2::config cfg ;
        cfg.enable_device(seriallist[serial]);
        pipe.start(cfg) ;
    }

    void decodeImage() {
        rs2::frameset frameset = wait_for_frame(pipe) ;

        RealSenseDataFrame::RealSenseFrame _frame ;
        _frame.color = new rs2::video_frame(frameset.get_color_frame()) ;
        _frame.depth = new rs2::depth_frame(frameset.get_depth_frame()) ;
        this->applyfilters(*(_frame.depth));
        d_rsframe.setValue(RealSenseDataFrame(_frame));

//        frame_to_cvmat(*color, *depth, img, *d_depth.beginEdit());
////        frame_to_cvmat(*color, *depth, *d_color.beginEdit(), *d_depth.beginEdit());
////        d_color.endEdit();
//        d_depth.endEdit();
    }

} ;

/*!
 * \brief The RealSenseMultiCam class
 * tries listing all realsense cameras connected and instanciates VirtualCamera components accordingly
 */
class RealSenseMultiCam : public core::objectmodel::BaseObject {
public:
    typedef core::objectmodel::BaseObject Inherited;
    SOFA_CLASS( RealSenseMultiCam , Inherited);

    Data<std::string> d_calibpath ;

    RealSenseMultiCam()
        : Inherited()
        , d_calibpath(initData(&d_calibpath, std::string("./"), "calibpath", "path to folder with calibration images"))
    {
        this->f_listening.setValue(true);
    }

    /*!
     * \brief listSerialNum
     * \return a list of the connected realsense camera's serial numbers
     */
    std::vector<std::string> listSerialNum()
    {
        std::vector<std::string> serial_list ;
        rs2::context ctx ;
        for (auto && device : ctx.query_devices()) {
            serial_list.push_back(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
        }
        return serial_list ;
    }

    /*!
     * \brief init
     * at the initialization of the component a virtual realsensecamera component will be added to the scene
     */
    void init () {
        auto serial_list = listSerialNum();

        core::objectmodel::BaseContext::SPtr context = this->getContext() ;

        size_t i = 0 ;
//        for (std::string serial : serial_list) {
//            add_realsenseCam(context, serial, i);
        for (int i = 0 ; i < serial_list.size() ; i++) {
            add_realsenseCam(context, i);
        }
    }

    /*!
     * \brief add_realsenseCam create the component and add it to the corresponding node
     * \param node : usually actual context
     * \param serial : realsense serial number
     * \param i : index interator
     */
    void add_realsenseCam (core::objectmodel::BaseContext::SPtr node, int serialid) {
        RealSenseVirtualCam* rs_vcam = new RealSenseVirtualCam(serialid) ;
        rs_vcam->setName(std::string("RSCam_") + std::to_string(serialid));
        std::string calibpath = d_calibpath.getValue()+std::to_string(serialid) ;
        rs_vcam->d_calibpath.setValue(calibpath);
        rs_vcam->d_serialnum.setValue(serialid);
        if (!helper::system::FileSystem::exists(calibpath)) {
            helper::system::FileSystem::createDirectory(calibpath) ;
        }
        node->addObject(rs_vcam) ;
    }

} ;

}

}

