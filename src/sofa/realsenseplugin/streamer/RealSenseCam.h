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

namespace realsenseplugin
{

using defaulttype::Vec;
using defaulttype::Vector3;


using namespace std;
using namespace cv;
using namespace boost;
using namespace rs2;

/*!
 * \brief The RealSenseCam class
 * Streamer to use when only one realsense is connected
 */
class RealSenseCam : public RealSenseStreamer
{
public:
    SOFA_CLASS( RealSenseCam , RealSenseStreamer );
    typedef RealSenseStreamer Inherited;

    Data<int> depthMode;

    Data<size_t> d_exposure;
//    DataCallback c_exposure ;

    rs2_intrinsics cam_intrinsics ;
    rs2::pipeline_profile selection ;

    /// \brief for pointcloud extraction, deprecated
    rs2::pointcloud pc ;
    rs2::points points ;

    bool pause ;

    RealSenseCam()
        : Inherited()
        , depthMode ( initData ( &depthMode,1,"depthMode","depth mode" ))
        , d_exposure(initData(&d_exposure, "exposure", "exposure"))
        , pause (false)
    {
        d_exposure.setValue(100) ;
//        c_exposure.addInputs({&d_exposure});
//        c_exposure.addCallback(std::bind(&RealSenseCam::setExposure, this));
    }

    ~RealSenseCam () {
    }

    void decodeImage(RealSenseDataFrame & rs_frame) {
        if (pause) return ;
        acquireAligned(rs_frame);
    }

    void handleEvent(sofa::core::objectmodel::Event* event) {
        Inherited::handleEvent(event) ;
        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == ' ') {
                pause = ! pause ;
            }
        }
    }

protected:

    ///\brief set exposure of camera from "exposure" data in sofa
    inline void setExposure()
    {
        pipe.get_active_profile()
                .get_device()
                .first<color_sensor>()
                .set_option(RS2_OPTION_EXPOSURE, d_exposure.getValue());
    }

    ///\brief setup realsense data acquisition pipeline
    inline void configPipe()
    {
        auto resolution = d_resolution_rs.getValue() ;
        rs2::config cfg ;
        int serialid = d_serialnum.getValue() ;
        if (serialid < 0 || serialid >= seriallist.size()) {
            std::cout << "(RealSenseCam) serial number index invalid <" << serialid << "> "
                      << "should be between [0.." << seriallist.size() << "]" << std::endl ;
        } else {
            std::cout << "(RealSenseCam) serial number index valid" << std::endl
                      << "using serial <" << seriallist[serialid] << "> " << std::endl ;
            cfg.enable_device(seriallist[serialid]);
        }
        cfg.enable_stream(
            RS2_STREAM_COLOR,
            resolution.at(0), resolution.at(1),
            RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(
            RS2_STREAM_DEPTH,
            resolution.at(0), resolution.at(1),
            RS2_FORMAT_Z16, 30);
        selection = pipe.start(cfg);
        if (d_exposure.getValue() > 0) {
//            setExposure();
        }
    }

    ///\brief wait for a second in order for auto exposure to settle in at initialization
    inline void stabilizeAutoExp() {
        //drop first frames to let auto exposure settle in
        for (int i = 60 ; i-- ;) wait_for_frame(pipe) ;
    }

    void init() {
        configPipe();
        stabilizeAutoExp();
        acquireAligned(*d_rsframe.beginEdit());
        d_rsframe.endEdit();
        writeIntrinsicsToFile();
    }

    void acquireAligned(RealSenseDataFrame & rs) {
        rs2::frameset frameset = wait_for_frame(pipe) ;

        // Trying to get both color and aligned depth frames
        if (color) delete color ;
        if (depth) delete depth ;
        color = new rs2::video_frame(frameset.get_color_frame()) ;
        depth = new rs2::depth_frame(frameset.get_depth_frame()) ;
//        this->applyfilters(*depth);

        // extract pointcloud
        //getpointcloud(*color, *depth) ;

        // Create depth and color image
        rgbd_frame & frame = rs.getRGBD() ;
        frame_to_cvmat(*color, *depth, frame.color, frame.depth);
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

