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
#include <sofa/opencvplugin/OpenCVWidget.h>

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

    Data<defaulttype::Vector2> d_resolution_rs ;
    Data<opencvplugin::TrackBar1> d_exposure;
    DataCallback c_exposure ;

    Data<std::string> d_calibpath ;
    std::vector<cv::Mat> calib_imagelist ;

    rs2_intrinsics cam_intrinsics ;
    rs2::pipeline_profile selection ;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;

    // for pointcloud extraction
    rs2::pointcloud pc ;
    rs2::points points ;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;

    RealSenseCam()
        : Inherited()
        , depthMode ( initData ( &depthMode,1,"depthMode","depth mode" ))
        , d_resolution_rs(initData(&d_resolution_rs, defaulttype::Vector2(640, 480), "resolution_rs", "realsense camera resolution"))
        , d_exposure(initData(&d_exposure, opencvplugin::TrackBar1(100,2000), "exposure", "exposure"))
        , d_calibpath(initData(&d_calibpath, std::string("./"), "calibpath", "path to folder with calibration images"))
    {
        c_exposure.addInputs({&d_exposure});
        c_exposure.addCallback(std::bind(&RealSenseCam::setExposure, this));
        this->f_listening.setValue(true) ;
        initAlign();
    }

    ~RealSenseCam () {
    }

    void decodeImage(cv::Mat & img) {
        acquireAligned(img);
        writeIntrinsicsToFile();
    }

    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == 'I' || ev->getKey() == 'i') {
                calib_imagelist.push_back(d_color.getValue().getImage());
            }
            else if (ev->getKey() == 'z' || ev->getKey() == 'Z') {
                calib_imagelist.pop_back();
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

protected:

    void setExposure()
    {
        rs2::device selected_device = selection.get_device();
        std::vector<rs2::sensor> sensors = selected_device.query_sensors();
        for (auto && sensor : sensors) {
            if (sensor.get_stream_profiles()[0].stream_type() == RS2_STREAM_COLOR) {
                sensor.set_option(RS2_OPTION_EXPOSURE, d_exposure.getValue());
            }
        }
    }

    inline void configPipe()
    {
        auto resolution = d_resolution_rs.getValue() ;
        rs2::config cfg ;
        cfg.enable_stream(
            RS2_STREAM_COLOR,
            resolution.at(0), resolution.at(1),
            RS2_FORMAT_BGR8, 30);
        cfg.enable_stream(
            RS2_STREAM_DEPTH,
            resolution.at(0), resolution.at(1),
            RS2_FORMAT_Z16, 30);
//        pipe.start(cfg);
        selection = pipe.start(cfg);
        if (d_exposure.getValue() > 0) {
            setExposure();
        }
    }

    inline void stabilizeAutoExp() {
        //drop first frames to let auto exposure settle in
        for (int i = 60 ; i-- ;) wait_for_frame(pipe) ;
    }

    void initAlign() {
        // set config for resolution/fps ...
        configPipe();
        stabilizeAutoExp();
        acquireAligned(*d_image.beginEdit());
        d_image.endEdit();
    }

    void acquireAligned(cv::Mat & img) {
        rs2::frameset frameset = wait_for_frame(pipe) ;

        // Trying to get both color and aligned depth frames
        if (color) delete color ;
        if (depth) delete depth ;
        color = new rs2::video_frame(frameset.get_color_frame()) ;
        depth = new rs2::depth_frame(frameset.get_depth_frame()) ;

        // extract pointcloud
        //getpointcloud(*color, *depth) ;

        // Create depth and color image
        frame_to_cvmat(*color, *depth, img, *d_depth.beginEdit());
        d_depth.endEdit();
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

