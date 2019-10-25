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

// pcl // deactivated
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
#include "RealSenseCam.h"
#include "RealSenseDistFrame.h"

namespace sofa {

namespace rgbdtracking {

/*!
 * \brief The RealSenseDeprojector class
 * Online / Offline 2D-3D deprojection
 */
class RealSenseDeprojector : public core::objectmodel::BaseObject
{

public:
    SOFA_CLASS( RealSenseDeprojector , core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<opencvplugin::ImageData> d_depth ;
    Data<opencvplugin::ImageData> d_color ;
    Data<int> d_downsampler ;
    Data<bool> d_drawpcl ;
    Data<helper::vector<defaulttype::Vector3> > d_output ;

    // distance frame for offline reco
    Data<RealSenseDistFrame> d_distframe ;
    // path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;
    // path to save snapshots to
    Data<std::string> d_snap_path ;


    core::objectmodel::SingleLink<
        RealSenseDeprojector,
        RealSenseCam,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_rs_cam ; //for intrinsics
    DataCallback c_image ;

    rs2_intrinsics cam_intrinsics ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr m_pointcloud ;
    pcl::PointCloud<pcl::Normal>::Ptr m_cloud_normals ;

    helper::vector<defaulttype::Vector3> m_colors ;

    RealSenseDeprojector()
        : Inherited()
        , d_depth(initData(&d_depth, "depth", "segmented depth data image"))
        , d_color(initData(&d_color, "color", "segmented color data image"))
        , d_downsampler(initData(&d_downsampler, 5, "downsample", "point cloud downsampling"))
        , d_output(initData(&d_output, "output", "output 3D position"))
        , d_drawpcl(initData(&d_drawpcl, false, "drawpcl", "true if you want to draw the point cloud"))

        //offline reco
        , d_distframe(initData(&d_distframe, "distframe", "frame encoding pixel's distance from camera. used for offline deprojection"))
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsics", "path to realsense intrinsics file to read from"))
        , d_snap_path(initData(&d_snap_path, std::string("."), "snap_path", "path to snap shots folder"))

        // needed for online reco
        , l_rs_cam(initLink("rscam", "link to realsense camera component - used for getting camera intrinsics"))
        , m_pointcloud(new pcl::PointCloud<pcl::PointXYZ>)
        , m_cloud_normals(new pcl::PointCloud<pcl::Normal>)
    {
        c_image.addInputs({&d_color, &d_depth});
        c_image.addCallback(std::bind(&RealSenseDeprojector::deproject_image, this));

        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseDeprojector::readIntrinsics, this));
        readIntrinsics();
        this->f_listening.setValue(true) ;
    }

    virtual ~RealSenseDeprojector () {
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_drawpcl.getValue()) {
        // don't draw point cloud
            return ;
        }

        if (m_colors.size() == m_pointcloud->size()) {
            size_t i = 0 ;
            for (const auto & pt : *m_pointcloud) {
                auto color = m_colors[i++] ;
                vparams->drawTool()->drawPoint(
                    defaulttype::Vector3(pt.x, pt.y, pt.z),
                    sofa::defaulttype::Vector4 (color[0],color[1],color[2],0)
                );
            }
        } else {
            for (const auto & pt : *m_pointcloud) {
                vparams->drawTool()->drawPoint(
                    defaulttype::Vector3(pt.x, pt.y, pt.z),
                    sofa::defaulttype::Vector4 (0, 0, 255, 0)
                );
            }
        }
    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)) {
            //std::cout << ev->getKey() << std::endl ;

            // screen shot : ctrl+shift+P
            if ((int)ev->getKey() == 'P') {
                exportSnapShot () ;
            }
        }
    }
private :
    void erode_mask (const cv::Mat & src, cv::Mat & erosion_dst, int erosion_size) {
        cv::Mat element = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
            cv::Point( erosion_size, erosion_size )
        );

        /// Apply the erosion operation
        cv::dilate( src, erosion_dst , element );
    }

    void exportSnapShot () {
        static size_t i = 0 ; //< i is a snap shot 'id'
        // set filenames
        std::string
            snapshot_color_filename =
                d_snap_path.getValue() +
                "/rgb_snap_" + std::to_string(i) + ".png",
            snapshot_depth_filename =
                d_snap_path.getValue() +
                "/depth_snap_" + std::to_string(i) + ".png",
            snapshot_dist_filename =
                d_snap_path.getValue() +
                "/dist_snap_" + std::to_string(i) + ".dist" ;
        i++ ;

        // get images to export
        cv::Mat colormat = d_color.getValue().getImage(),
                depthmat = d_depth.getValue().getImage() ;
        // export pngs
        cv::imwrite(snapshot_color_filename.c_str(), colormat) ;
        std::cout << "(RealSenseCam) exported " << snapshot_color_filename << std::endl ;
        cv::imwrite(snapshot_depth_filename.c_str(), depthmat) ;
        std::cout << "(RealSenseCam) exported " << snapshot_depth_filename << std::endl ;

        // export dist frames
        this->_write_distFrame(snapshot_dist_filename);
        std::cout << "(RealSenseCam) exported " << snapshot_dist_filename << std::endl ;
    }

    void _write_distFrame (std::string snapshot_dist_filename) {
        RealSenseDistFrame distframe = d_distframe.getValue() ;
        RealSenseDistFrame::RealSenseDistStruct diststruct = distframe.getFrame();

        std::FILE* filestream = std::fopen(snapshot_dist_filename.c_str(), "wb") ;
        if (filestream == nullptr) {
            std::cerr << "(RealSenseDeprojector) check writing rights on file : "
                     << snapshot_dist_filename
                     << std::endl ;
            return ;
        }
        // write width and height
        std::fwrite(&diststruct._width, sizeof(size_t), 1, filestream) ;
        std::fwrite(&diststruct._height, sizeof(size_t), 1, filestream) ;

        // write frame data
        std::fwrite (
            diststruct.frame,
            sizeof(float),
            diststruct._width * diststruct._height,
            filestream
        ) ;
        std::fclose(filestream) ;
    }

    void readIntrinsics () {
        std::FILE* filestream = std::fopen(d_intrinsics.getValue().c_str(), "rb") ;
        if (filestream == NULL) {
            std::cout << "Check rights on intrins.log file" << std::endl ;
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
    }

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
        for (size_t i = 0 ; i < diststruct._height ; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > 0) {
                    // deprojection
                    float dist = diststruct.frame[i*diststruct._width+j] ;
                    push_to_pointcloud(j, diststruct, downSample, i, dist);
                }
            }
        }

    }

    void push_to_pointcloud(size_t j, RealSenseDistFrame::RealSenseDistStruct& diststruct, int downSample, size_t i, float dist)
    {
        float
            point3d[3] = {0.f, 0.f, 0.f},
            point2d[2] = {downSample*i, downSample*j};
        rs2_deproject_pixel_to_point(
            point3d,
            &cam_intrinsics,
            point2d,
            dist
        );
        // set
        diststruct.frame[i*diststruct._width+j] = dist ;
        // set units // switch comments for alignment
        //pcl::PointXYZ pclpoint = pcl::PointXYZ(-point3d[1], -point3d[0], -point3d[2]) ;
        pcl::PointXYZ pclpoint = pcl::PointXYZ(point3d[1], -point3d[0], -point3d[2]) ;
        // add units to result
        m_pointcloud->push_back(pclpoint);
    }

    void deproject_image_online()
    {
        // get intrinsics from link to rs-cam component
        rs2::depth_frame depth = *l_rs_cam->depth ;
        cam_intrinsics = //l_rs_cam->cam_intrinsics ;
            depth.get_profile()
                .as<rs2::video_stream_profile>()
                .get_intrinsics();

        // get depth
        cv::Mat depth_im = d_depth.getValue().getImage() ;

        // setup output
        m_pointcloud->clear();

        int downSample = d_downsampler.getValue() ;
        RealSenseDistFrame::RealSenseDistStruct & diststruct = *d_distframe.beginEdit();
        diststruct._width = depth_im.cols/downSample ;
        diststruct._height = depth_im.rows/downSample ;
        diststruct.frame = new float[
            depth_im.cols/downSample *
            depth_im.rows/downSample
        ] ;
        for (size_t i = 0 ; i < diststruct._height; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > 0) {
                    // deprojection
                    float dist = depth.get_distance(downSample*j, downSample*i) ;
                    push_to_pointcloud(j, diststruct, downSample, i, dist);
                }
            }
        }
        d_distframe.endEdit();
    }

    void deproject_image () {
        if (!l_rs_cam) {
        // we need a valid link to realsense cam sofa component
            deproject_image_offline () ;
        } else {
            deproject_image_online();
        }
        // once pcl extracted compute normals
        //compute_pcl_normals() ;
    }

    void compute_pcl_normals () {
        // compute normals for remeshing ?
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne ;
        ne.setInputCloud(m_pointcloud);
        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        m_cloud_normals->clear();
        ne.compute (*m_cloud_normals);
    }


};

}

}

