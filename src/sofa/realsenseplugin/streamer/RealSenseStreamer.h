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
class RealSenseStreamer : public core::objectmodel::BaseObject
{
public :
    SOFA_CLASS( RealSenseStreamer , core::objectmodel::BaseObject );
    typedef core::objectmodel::BaseObject Inherited;

    /// \brief realsense camera resolution
    Data<defaulttype::Vector2> d_resolution_rs ;

    /// \brief RGB image data
    Data<opencvplugin::ImageData> d_color ;
    /// \brief depth image data
    Data<opencvplugin::ImageData> d_depth ;

    /// \brief path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;

    /// \brief camera's serial number index
    Data <int> d_serialnum ;
    /// \brief output camera's inrinsic parameters
    Data<defaulttype::Vector4 > d_intrinsicParameters ;
    /// \brief magnitude for decimation filter
    Data<int> d_decimation ;
    /// \brief smooth alpha for temporal filter
    Data<float> d_tmp_alpha ;
    /// \brief smooth delta for temporal filter
    Data<int> d_tmp_delta ;
    DataCallback c_filters ;
    /// \brief decimation filter
    rs2::decimation_filter decimation ;
    /// \brief temporal filter
    rs2::temporal_filter temporal ;

    /// \brief path to save snapshots for potentiel calibration
    Data<std::string> d_calibpath ;
    std::vector<cv::Mat> calib_imagelist ;

    /// \brief depth scale for converting depth to meters
    Data<int> depthScale;
    /// \brief for colorizing depth frame
    Data<bool> d_colorize;

    /// \brief color frame pointer
    rs2::video_frame *color ;
    /// \brief depth frame pointer
    rs2::depth_frame *depth ;

    /// \brief sensor's intrinsics
    rs2_intrinsics cam_intrinsics ;
    /// \brief colorizer used for colorizing depth frame
    rs2::colorizer rizer ;

    RealSenseStreamer ()
        : Inherited()
        // frame attributes
        , d_resolution_rs(initData(&d_resolution_rs, defaulttype::Vector2(640, 480), "resolution", "realsense camera resolution"))
        , d_color(initData(&d_color, "color", "RGB data image"))
        , d_depth(initData(&d_depth, "depth", "depth data image"))
        // sensor's attributes
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsics", "path to file to write realsense intrinsics into"))
        , d_serialnum(initData(&d_serialnum, 0, "serialid", "camera's serial number id (between 0 and number of connected cams-1)"))
        , d_intrinsicParameters(initData(&d_intrinsicParameters, "intrinsicParameters", "vector output with camera intrinsic parameters"))
        // filters parameters
        , d_decimation(initData(&d_decimation, 4, "decimation", "decimation magnitude"))
        , d_tmp_alpha(initData(&d_tmp_alpha, 0.420f, "alpha", "temporal filter alpha [0, 1]"))
        , d_tmp_delta(initData(&d_tmp_delta, 20, "delta", "temporal filter delta [1, 100]"))
        // path to store calib images
        , d_calibpath(initData(&d_calibpath, std::string("./"), "calibpath", "path to folder with calibration images"))
        // depth scale and colorizer
        , depthScale(initData(&depthScale,10,"depthScale","scale for the depth values, 1 for SR300, 10 for 435"))
        , d_colorize(initData(&d_colorize,false,"colorize","colorize depth frame"))
        // class members init
        , color(nullptr), depth(nullptr)
        , calib_imagelist()
    {
        checkFiltersParams();
        rizer.set_option(RS2_OPTION_COLOR_SCHEME, 2);
        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseStreamer::writeIntrinsicsToFile, this));
        c_filters.addInputs({&d_decimation, &d_tmp_alpha, &d_tmp_delta});
        c_filters.addCallback(std::bind(&RealSenseStreamer::checkFiltersParams, this));
        this->f_listening.setValue(true) ;
        seriallist = listSerialNum() ;
    }

    /*!
     * \brief checkFiltersParams check that parameters are valid
     */
    void checkFiltersParams () {
        int & decimation = *d_decimation.beginEdit() ;
        float &
            alpha = *d_tmp_alpha.beginEdit(),
            delta = *d_tmp_delta.beginEdit() ;
        if (decimation >= 8) decimation = 8 ;
        if (decimation <= 2) decimation = 2 ;
        if ((int)(alpha*1e2) > 100) alpha = 1. ;
        if ((int)(alpha*1e2) < 1) alpha = .01 ;
        if ((int)delta > 100) delta = 100 ;
        if ((int)delta < 1) delta = 1 ;
        d_decimation.endEdit();
        d_tmp_alpha.endEdit();
        d_tmp_delta.endEdit();
        this->decimation.set_option(RS2_OPTION_FILTER_MAGNITUDE, d_decimation.getValue());
        this->temporal.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, d_tmp_alpha.getValue());
        this->temporal.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, d_tmp_delta.getValue());
     }

    virtual void decodeImage(cv::Mat & img) = 0 ;

    /*!
     * \brief handleEvent : Press i to push image to calibration image list, z to cancel, s to save
     * \param event
     */
    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if(sofa::simulation::AnimateBeginEvent::checkEventType(event)) {
            decodeImage(*d_color.beginEdit());
            d_color.endEdit();
        }
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
                    std::string imagepath = path + "/" + std::to_string(i++) + ".png" ;
                    cv::imwrite(imagepath, image) ;
                }
            }
        }
    }

protected :

    /// \brief list of connected realsense sensors serial numbers
    std::vector<std::string> seriallist ;
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
     * \brief writeIntrinsicsToFile : intrinsics path file in sofa data needs to be set
     */
    void writeIntrinsicsToFile() {
        if (depth == nullptr) return ;
        cam_intrinsics = depth->get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;
        this->writeIntrinsics(d_intrinsics.getValue(), cam_intrinsics);
    }

    /*!
     * \brief writeIntrinsics write realsense cam intrinsics into file (for offline reconstruction)
     * \param filename
     * \param cam_intrinsics
     */
    void writeIntrinsics (std::string filename, const rs2_intrinsics &cam_intrinsics) {
        // for exporting intrinsics as sofa data
        defaulttype::Vector4 & intrinsics = *d_intrinsicParameters.beginEdit() ;
        intrinsics = defaulttype::Vector4 (
            cam_intrinsics.fx,
            cam_intrinsics.fy,
            cam_intrinsics.ppx, // x0
            cam_intrinsics.ppy // y0
        ) ;
        d_intrinsicParameters.endEdit();
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
     * \brief applyfilters :: applies decimation and temporal filters to depth frame
     * \param depth : rs2::depth_frame to process
     */
    void applyfilters(rs2::depth_frame & depth) {
//        depth = decimation.process(depth) ;
        depth = temporal.process(depth) ;
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
//        int widthc = color.get_width();
//        int heightc = color.get_height();

////        cv::Mat rgb0(heightc,widthc, CV_8UC3, (void*) color.get_data()) ;
////        cv::cvtColor (rgb0, bgr_image, cv::COLOR_RGB2BGR); // bgr_image is output

//        bgr_image = cv::Mat(heightc,widthc, CV_8UC3, (void*) color.get_data()) ;

        bgr_image = frame_to_mat(color) ;
        if (d_colorize.getValue()) {
            rs2::frame bw_depth = depth.apply_filter(rizer) ;
            depth8 = frame_to_mat(bw_depth) ;
        } else {
            int widthd = depth.get_width();
            int heightd = depth.get_height();
            cv::Mat depth16 = cv::Mat(heightd, widthd, CV_16U, (void*)depth.get_data()) ;
            depth16.convertTo(depth8, CV_8U, 1.f/64*depthScale.getValue()); //depth32 is output
        }
    }
} ;

}

}

