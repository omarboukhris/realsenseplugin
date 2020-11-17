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
#include <sofa/helper/AdvancedTimer.h>

// lib realsense
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

// c++ stl
#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/realsenseplugin/streamer/RealSenseCam.h>

namespace sofa {

namespace realsenseplugin {

class RealSenseAbstractDeprojector : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( RealSenseAbstractDeprojector , core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    /// \brief rgbd frame
    Data<RealSenseDataFrame> d_rsframe ;
    /// \brief rgbd frame
    Data<RealSenseDataFrame> d_rsframe_out ;
    /// \brief translation offset
    Data<defaulttype::Vector3> d_tr_offset ;
    /// \brief projection matrix for pointcloud rotation/translation
    Data<defaulttype::Mat3x3> d_rotation ;
    DataCallback c_projmat ;
    /// \brief output pointcloud sofa
    Data<helper::vector<defaulttype::Vector3> > d_output ;
    /// \brief synthetic volume pointcloud
    Data<helper::vector<defaulttype::Vector3> > d_synthvolume ;

    Data<int> d_scale ;
    DataCallback c_scale ;

    /// \brief path to intrinsics file
    Data<std::string> d_intrinsics ;
    DataCallback c_intrinsics ;
    /// \brief intput camera's inrinsic parameters
    Data<defaulttype::Vector4 > d_intrinsicParameters ;
    DataCallback c_intrinsicsParam ;

    Data<defaulttype::Vector2> d_minmax ;
    ///\brief true to flip pointcloud over z-axis
    Data<bool> d_flip ;
    ///\brief downsampling
    Data<int> d_downsampler ;
    /// \brief true to render pointcloud in viewer
    Data<bool> d_drawpcl ;
    /// \brief true to add synthetic volume
    Data<int> d_densify ;

    /// \brief link for realsense camera
    core::objectmodel::SingleLink<
        RealSenseAbstractDeprojector,
        RealSenseCam,
        BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK
    > l_rs_cam ; //for intrinsics

    /// \brief pcl pointcloud as internal data
    rgbd_frame::sofaPointCloud m_pointcloud ;
    /// \brief camera intrinsics read from file or realsensecam component
    rs2_intrinsics cam_intrinsics ;
    /// \brief for rotating the pointcloud
    helper::Quater<double> q ;

    double inv_fx, inv_fy, ppx, ppy ;

    RealSenseAbstractDeprojector()
        : Inherited()
        , d_rsframe(initData(&d_rsframe, "rsframe", "RGBD data image input"))
        , d_rsframe_out(initData(&d_rsframe_out, "rsframe_out", "RGBD data image output"))
        , d_tr_offset(initData(&d_tr_offset, defaulttype::Vector3(0,0,0), "offset", "translation offset"))
        , d_rotation(
            initData(
                &d_rotation,
                defaulttype::Mat3x3(
                    defaulttype::Vector3(1,0,0),
                    defaulttype::Vector3(0,1,0),
                    defaulttype::Vector3(0,0,1)),
                "rotation",
                "rotation matrix for rotating pointcloud"
            )
        ), d_output(initData(&d_output, "output", "output 3D position"))
        , d_synthvolume(initData(&d_synthvolume, "synthvol", "synthetic volume for ICP optimization"))
        , d_scale(initData(&d_scale, 100, "scale", "point cloud scaling factor"))
        // offline reco
        , d_intrinsics(initData(&d_intrinsics, std::string("intrinsics.log"), "intrinsicsPath", "path to realsense intrinsics file to read from"))
        , d_intrinsicParameters(initData(&d_intrinsicParameters, "intrinsicParameters", "vector output with camera intrinsic parameters"))
        // visualization
        , d_minmax (initData(&d_minmax, defaulttype::Vector2(0, 255),"minmax", "depth value filter"))
        , d_flip(initData(&d_flip, false, "flip", "flip z axis"))
        , d_downsampler(initData(&d_downsampler, 1, "downsample", "point cloud downsampling"))
        , d_drawpcl(initData(&d_drawpcl, false, "drawpcl", "true if you want to draw the point cloud"))
        , d_densify(initData(&d_densify, 0, "densify", "densify pointcloud to approximate volume (naive method)"))
        // link to realsense for online reco
        , l_rs_cam(initLink("rscam", "link to realsense camera component - used for getting camera intrinsics"))
        , m_pointcloud(rgbd_frame::sofaPointCloud())
    {
        c_intrinsics.addInput({&d_intrinsics});
        c_intrinsics.addCallback(std::bind(&RealSenseAbstractDeprojector::readIntrinsics, this));
        c_intrinsicsParam.addInput({&d_intrinsicParameters});
        c_intrinsicsParam.addCallback(std::bind(&RealSenseAbstractDeprojector::updateIntrinsics, this));
        c_scale.addInputs({&d_scale, &d_tr_offset}) ;
        c_scale.addCallback(std::bind(&RealSenseAbstractDeprojector::deproject_image, this));
        c_projmat.addInputs({&d_rotation}) ;
        c_projmat.addCallback(std::bind(&RealSenseAbstractDeprojector::updateRotation, this)) ;
    }

    void updateIntrinsics () {
        inv_fx = 1./(d_intrinsicParameters.getValue()[0]) ;
        inv_fy = 1./(d_intrinsicParameters.getValue()[1]) ;
        ppx = d_intrinsicParameters.getValue()[2] ;
        ppy = d_intrinsicParameters.getValue()[3] ;
    }

    void init () {
        readIntrinsics();
        updateRotation();
    }

    void handleEvent(sofa::core::objectmodel::Event* event) {
//        if(sofa::simulation::AnimateEndEvent::checkEventType(event)) {
//            deproject_image();
//        }
    }

    /*!
     * \brief updateRotation updates quaternion for rotating pointcloud
     */
    void updateRotation () {
//        std::cout << "(RealSenseReprojector) updated rotation matrix" << std::endl ;
        defaulttype::Mat3x3 pmat = d_rotation.getValue() ;
        q.fromMatrix(pmat);
    }

    /*!
     * \brief readIntrinsics reads from a specified file (data "intrinsics")
     * to rs2_intrinsics cam_intrinsics
     */
    void readIntrinsics () {
        if (l_rs_cam) {
//            std::cout << "(RealSenseProjectors) link to realsense cam is valid, " <<
//                         "intrinsics won't be read" << std::endl ;
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
        d_intrinsicParameters.setValue(
            defaulttype::Vector4(cam_intrinsics.fx, cam_intrinsics.fy, cam_intrinsics.ppx, cam_intrinsics.ppy)
        );

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
     * get depth/distframe/intrinsics.. from realsense cam
     * outputs 3D pointcloud
     */
    inline void deproject_image () {
        helper::AdvancedTimer::stepBegin("RS Deprojection") ;
        if (l_rs_cam) {
            // get intrinsics from link to rs-cam component
            rs2::depth_frame depth = *l_rs_cam->depth ;
            cam_intrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics() ;
        } else {
            readIntrinsics(); // from file
        }

        // get depth image, and downsampling value
        int downSample = d_downsampler.getValue() ;
        cv::Mat depth_im = d_rsframe.getValue().getcvDepth() ;
        if (depth_im.empty()) {
        // depth image is empty
            return ;
        }
        m_pointcloud.clear();
        writeToOutput(depth_im, downSample);
        RealSenseDataFrame current_frame = d_rsframe.getValue() ;
        RealSenseDataFrame frame_out (
            current_frame.getcvColor(),
            current_frame.getcvDepth(),
            d_output.getValue());
        d_rsframe_out.setValue(frame_out) ;

        //make synthetic volume
        this->makeSyntheticVolume();
        // set pointcloud to output
        helper::AdvancedTimer::stepEnd("RS Deprojection") ;
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_drawpcl.getValue()) {
        // don't draw point cloud
            return ;
        }
        helper::AdvancedTimer::stepBegin("RS Deprojection draw") ;
        helper::vector<defaulttype::Vector3> output = d_output.getValue() ;
        for (unsigned int i=0; i< output.size(); i++) {
            vparams->drawTool()->drawSphere(output[i], 0.002);
//            vparams->drawTool()->drawPoint(output[i], sofa::defaulttype::Vector4 (0, 0, 255, 0)) ;
        }
        helper::AdvancedTimer::stepEnd("RS Deprojection draw") ;
    }

    /*!
     * \brief applyRotationTranslation apply rotation then translation (offset) to point
     * \param point
     */
    void applyRotationTranslation(defaulttype::Vector3 & point) {
        point = q.rotate(point) + d_tr_offset.getValue() ;
    }

    /*!
     * \brief push_to_pointcloud
     * push a single point to pointcloud set
     * \param outpoints : output points set
     * \param i : pixel coordinate to deproject in x-axis
     * \param j : pixel coordinate to deproject in y-axis
     * \param index : index of dist frame to use
     * \param diststruct : dist structure containing distance information
     * \param dist : distance value to use
     */
    void push_to_pointcloud(helper::vector<defaulttype::Vector3> & outpoints, size_t i, size_t j, float dist)
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

        // check for outliers
        if (std::abs(point3d[0]) < 1e-4 ||
            std::abs(point3d[1]) < 1e-4 ||
            std::abs(point3d[2]) < 1e-4 ||
            std::abs(point3d[0]) > 5 ||
            std::abs(point3d[1]) > 5 /*||
            std::abs(point3d[2]) > 0.45*/) {
        //invalid point
            return ;
        }

        defaulttype::Vector3 point = scalePoint(point3d) ;
        //m_pointcloud.push_back(pt);

        if (d_flip.getValue()) {
            point = defaulttype::Vector3(point[1], point[0], - point[2]) ;
        }
        applyRotationTranslation(point);
        outpoints.push_back(point) ;
    }

    /*!
     * \brief push_to_pointcloud_2
     * push a single point to pointcloud set
     * \param outpoints : output points set
     * \param depth_im : depth image
     * \param i : pixel coordinate to deproject in x-axis
     * \param j : pixel coordinate to deproject in y-axis
     */
    void push_to_pointcloud(helper::vector<defaulttype::Vector3> & outpoints, const cv::Mat & depth_im, size_t i, size_t j)
    {
        float depthvalue = depth_im.at<float>(i,j)  ;
        float point3d[3] = {
            (j - ppx) * depthvalue * inv_fx * .1,
            (i - ppy) * depthvalue * inv_fy * .1,
            depthvalue
        } ;
//        // check for outliers
//        if (std::abs(point3d[0]) < 1e-3 ||
//            std::abs(point3d[1]) < 1e-3 ||
//            std::abs(point3d[2]) < 1e-3 ||
//            std::abs(point3d[0]) > 5 ||
//            std::abs(point3d[1]) > 5 /*||
//            std::abs(point3d[2]) > 0.45*/) {
//        //invalid point
//            return ;
//        }

        defaulttype::Vector3 point = scalePoint(point3d) ;
        //m_pointcloud->push_back(pt);

        if (d_flip.getValue()) {
            point = defaulttype::Vector3(point[1], point[0], - point[2]) ;
        }
        applyRotationTranslation(point);
        outpoints.push_back(point) ;
    }


    /*!
     * \brief makeSyntheticVolume
     * creates synthetic volume from surface (used for rigid registration)
     */
    void makeSyntheticVolume () {
        int dense = d_densify.getValue() ;
        if (dense <= 0) {
        // do not densify
            return ;
        }
        helper::vector<defaulttype::Vector3> & synth = *d_synthvolume.beginEdit();
        synth.clear() ;
        for (const auto & ptmp : m_pointcloud) {
            for (int i = 1 ; i<dense ; i++) {
                defaulttype::Vector3 point = defaulttype::Vector3(ptmp[0], ptmp[1], ptmp[2] - i*1e-2) ;
                if (d_flip.getValue()) {
                    point = defaulttype::Vector3(ptmp[1], ptmp[0], - ptmp[2] + i*1e-2) ;
                }
                applyRotationTranslation(point);
                synth.push_back(point) ;
            }
        }
        d_synthvolume.endEdit();
    }

    /*!
     * \brief scalePoint : self explanatory
     * \param point3d
     * \return scaled point as pcl::PointXYZ
     */
    inline defaulttype::Vector3 scalePoint (float * point3d) {
        float scale = (float)d_scale.getValue()/100.f ;
        return defaulttype::Vector3 (
            scale*point3d[0],
            scale*point3d[1],
            -scale*point3d[2]
        ) ;
    }

private :
    /// \brief write to output sofa data
    virtual void writeToOutput (
        const cv::Mat & depth_im,
        int downSample) = 0 ;

} ;

}

}
