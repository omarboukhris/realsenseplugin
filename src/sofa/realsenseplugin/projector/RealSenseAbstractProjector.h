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

// sofa imports
#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/OptionsGroup.h>

// lib realsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

// c++ stl
#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

// external plugins
#include <sofa/opencvplugin/OpenCVWidget.h>
#include <sofa/PCLPlugin/PointCloudData.h>

#include <sofa/realsenseplugin/streamer/RealSenseCam.h>
#include <sofa/realsenseplugin/projector/RealSenseDistFrame.h>

namespace sofa {

namespace rgbdtracking {

class RealSenseAbstractDeprojector : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( RealSenseAbstractDeprojector , core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<opencvplugin::ImageData> d_depth ;
    Data<defaulttype::Vector3> d_tr_offset ;
    Data<helper::vector<defaulttype::Vector3> > d_output ;
    Data <pointcloud::PointCloudData> d_outpcl ;

    Data<opencvplugin::TrackBar1> d_scale ;
    DataCallback c_scale ;

    // distance frame for offline reco
    Data<RealSenseDistFrame> d_distframe ;
    // path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;

    // downsampling and visualization
    Data<opencvplugin::TrackBar2> d_minmax ;
    Data<int> d_downsampler ;
    Data<bool> d_drawpcl ;

    core::objectmodel::SingleLink<
        RealSenseAbstractDeprojector,
        RealSenseCam,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_rs_cam ; //for intrinsics

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointcloud ;
    rs2_intrinsics cam_intrinsics ;

    RealSenseAbstractDeprojector()
        : Inherited()
        , d_depth(initData(&d_depth, "depth", "segmented depth data image"))
        , d_tr_offset(initData(&d_tr_offset, defaulttype::Vector3(0.1, -0.1, 0.5), "offset", "translation offset"))
        , d_output(initData(&d_output, "output", "output 3D position"))
        , d_outpcl(initData(&d_outpcl, "outpcl", "output pcl PointCloud"))
        , d_scale(initData(&d_scale, opencvplugin::TrackBar1(200, 2550, 1), "scale", "point cloud scaling factor"))
        // offline reco
        , d_distframe(initData(&d_distframe, "distframe", "frame encoding pixel's distance from camera. used for offline deprojection"))
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsics", "path to realsense intrinsics file to read from"))
        // visualization
        , d_minmax (initData(&d_minmax, opencvplugin::TrackBar2(helper::fixed_array<double,2>(0, 255)),"minmax", "depth value filter"))
        , d_downsampler(initData(&d_downsampler, 5, "downsample", "point cloud downsampling"))
        , d_drawpcl(initData(&d_drawpcl, false, "drawpcl", "true if you want to draw the point cloud"))
        // link to realsense for online reco
        , l_rs_cam(initLink("rscam", "link to realsense camera component - used for getting camera intrinsics"))
        , m_pointcloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseAbstractDeprojector::readIntrinsics, this));
        c_scale.addInputs({&d_scale, &d_tr_offset}) ;
        c_scale.addCallback(std::bind(&RealSenseAbstractDeprojector::deproject_image, this));
        this->f_listening.setValue(true) ;
    }

    void init () {
        readIntrinsics();
    }

    /*!
     * \brief readIntrinsics reads from a specified file (data "intrinsics")
     * to rs2_intrinsics cam_intrinsics
     */
    void readIntrinsics () {
        if (l_rs_cam) {
            std::cout << "(RealSenseProjectors) link to realsense cam is valid, " <<
                         "intrinsics won't be read"
                      << std::endl ;
            return ;
        }

        std::FILE* filestream = std::fopen(d_intrinsics.getValue().c_str(), "rb") ;
        if (filestream == NULL) {
            std::cout << "Check rights on " << d_intrinsics.getValue()
                      << " file" << std::endl ;
            return ;
        }
        std::fread(&cam_intrinsics.width, sizeof(int), 1, filestream) ;
        std::fread(&cam_intrinsics.height, sizeof(int), 1, filestream) ;
        std::fread(&cam_intrinsics.ppx, sizeof(float), 1, filestream) ;
        std::fread(&cam_intrinsics.ppy, sizeof(float), 1, filestream) ;
        std::fread(&cam_intrinsics.fx, sizeof(float), 1, filestream) ;
        std::fread(&cam_intrinsics.fy, sizeof(float), 1, filestream) ;
        std::fread(&cam_intrinsics.model, sizeof(rs2_distortion), 1, filestream) ;
        std::fread(cam_intrinsics.coeffs, sizeof(float), 5, filestream) ;
        std::fclose(filestream) ;
        std::cout << "(RealSenseProjectors) successfully read intrinsics from file " <<
                     d_intrinsics.getValue() << std::endl ;

//      // for printing intrinsics
//        std::cout <<
//            cam_intrinsics.width << std::endl <<
//            cam_intrinsics.height<< std::endl <<
//            cam_intrinsics.ppx<< std::endl <<
//            cam_intrinsics.ppy<< std::endl <<
//            cam_intrinsics.fx<< std::endl <<
//            cam_intrinsics.fy<< std::endl <<
//            cam_intrinsics.coeffs[0]<< "," << cam_intrinsics.coeffs[1]<< "," << cam_intrinsics.coeffs[2]<< "," << cam_intrinsics.coeffs[3]<< "," << cam_intrinsics.coeffs[4]
//        << std::endl ;
    }

    /*!
     * \brief deproject_image
     * dispatches data processing depending on data (link to realsense cam or not)
     */
    void deproject_image () {
        if (!l_rs_cam) {
        // we need a valid link to realsense cam sofa component
            deproject_image_offline () ;
        } else {
            deproject_image_online();
        }
        // set pointcloud to output
        d_outpcl.setValue(m_pointcloud);
    }

    /*!
     * \brief deproject_image_offline
     * Load depth/distframe from saved files
     */
    void deproject_image_offline () {
        cv::Mat depth_im = d_depth.getValue().getImage() ;
        RealSenseDistFrame distframe = d_distframe.getValue() ;
        RealSenseDistFrame::RealSenseDistStruct diststruct = distframe.getFrame() ;
        int downSample = d_downsampler.getValue() ;

        if (diststruct._width != (size_t)(depth_im.cols/downSample) ||
            diststruct._height != (size_t)(depth_im.rows/downSample)) {
            //std::cerr << "(deprojector) check sizes" << std::endl ;
            return ;
        }
        m_pointcloud->clear();
        writeOfflineToOutput(diststruct, depth_im, downSample);
    }

    /*!
     * \brief deproject_image_online
     * get depth/distframe/intrinsics.. from realsense cam
     */
    void deproject_image_online () {
        // get intrinsics from link to rs-cam component
        rs2::depth_frame depth = *l_rs_cam->depth ;
        cam_intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;

        // get depth image, and downsampling value
        int downSample = d_downsampler.getValue() ;
        cv::Mat depth_im = d_depth.getValue().getImage() ;
        RealSenseDistFrame::RealSenseDistStruct & diststruct = *d_distframe.beginEdit();
        diststruct._width = depth_im.cols/downSample ;
        diststruct._height = depth_im.rows/downSample ;
        diststruct.frame = new float[
            depth_im.cols/downSample *
            depth_im.rows/downSample
        ] ;
        m_pointcloud->clear();
        writeOnlineToOutput(depth, diststruct, depth_im, downSample);
        d_distframe.endEdit() ;
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_drawpcl.getValue()) {
        // don't draw point cloud
            return ;
        }

        helper::vector<defaulttype::Vector3> output = d_output.getValue() ;
        for (unsigned int i=0; i< output.size(); i++) {
            vparams->drawTool()->drawSphere(output[i], 0.0032);
//            vparams->drawTool()->drawPoint(output[i], sofa::defaulttype::Vector4 (0, 0, 255, 0)) ;
        }
    }
    void push_to_pointcloud(helper::vector<defaulttype::Vector3> & outpoints, size_t i, size_t j, int index, RealSenseDistFrame::RealSenseDistStruct& diststruct, float dist)
    {
        float
            point3d[3] = {0.f, 0.f, 0.f},
            point2d[2] = {static_cast<float>(i), static_cast<float>(j)};
        rs2_deproject_pixel_to_point(
            point3d,
            &cam_intrinsics,
            point2d,
            dist
        );
        // set dist frame for exportation if needed
        diststruct.frame[index] = dist ;

        // set units // switch comments for alignment
        pcl::PointXYZ pt = scalePoint(point3d) ;
        m_pointcloud->push_back(pt);

        defaulttype::Vector3 point = defaulttype::Vector3(pt.x, pt.y, pt.z) ;
        outpoints.push_back(point) ;
    }

    inline pcl::PointXYZ scalePoint (float * point3d) {
        float scale = (float)d_scale.getValue()/100.f ;
        defaulttype::Vector3 tr = d_tr_offset.getValue() ;
        return pcl::PointXYZ(
            scale*point3d[0]+tr[0],
            scale*point3d[1]+tr[1],
            -scale*point3d[2]+tr[2]
        ) ;
    }
    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == 'b' || ev->getKey() == 'B') {
                f_listening.setValue(false);
            }
        }
        if (sofa::core::objectmodel::MouseEvent * ev = dynamic_cast<sofa::core::objectmodel::MouseEvent*>(event)){
            if (ev->getState() == sofa::core::objectmodel::MouseEvent::LeftPressed) {
                std::cout << ev->getPosX() << " " << ev->getPosY() << std::endl ;
            } else if (ev->getState() == sofa::core::objectmodel::MouseEvent::Wheel) {
                std::cout << ev->getWheelDelta() << std::endl ;
            }
        }
    }

private :
    /// \brief write to output sofa data
    virtual void writeOfflineToOutput (
        RealSenseDistFrame::RealSenseDistStruct & diststruct,
        const cv::Mat & depth_im,
        int downSample) = 0 ;
    virtual void writeOnlineToOutput (
        rs2::depth_frame & depth,
        RealSenseDistFrame::RealSenseDistStruct & diststruct,
        const cv::Mat & depth_im,
        int downSample) = 0 ;


} ;

}

}

