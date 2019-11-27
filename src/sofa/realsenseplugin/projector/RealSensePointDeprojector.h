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

#include <sofa/realsenseplugin/projector/RealSenseAbstractProjector.h>

namespace sofa {

namespace rgbdtracking {

class RealSensePointDeprojector : public RealSenseAbstractDeprojector
{
public:
    typedef RealSenseAbstractDeprojector Inherited;
    SOFA_CLASS( RealSensePointDeprojector , Inherited);

    Data<helper::vector<defaulttype::Vector2> > d_input ;
    DataCallback c_image ;

    RealSensePointDeprojector()
        : Inherited()
        , d_input(initData(&d_input, "input", "input 2D position to de-project"))
    {
        c_image.addInputs({&d_input});
        c_image.addCallback(std::bind(&RealSensePointDeprojector::deproject_image, this));
    }

    virtual ~RealSensePointDeprojector () {
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

private :
    virtual void writeOfflineToOutput (RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        // setup output
        const helper::vector<defaulttype::Vector2> input = d_input.getValue() ;
        helper::vector<defaulttype::Vector3> & output = *d_output.beginEdit() ;
        output.clear () ;
        for (defaulttype::Vec2i vec : input){
            size_t i = vec[0], j = vec[1] ;
            if (depth_im.at<const uchar>(i, j) > d_minmax.getValue()[0] &&
                depth_im.at<const uchar>(i, j) < d_minmax.getValue()[1]) {
            // deprojection
                int index = static_cast<int>(i/downSample) * diststruct._width +
                    static_cast<int>(j/downSample) ;
                float dist = diststruct.frame[index] ;
                push_to_pointcloud (i, j, index, dist, diststruct, output) ;
            }
        }
        // the end
        d_output.endEdit();
    }

    virtual void writeOnlineToOutput (rs2::depth_frame & depth, RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        // setup output
        const helper::vector<defaulttype::Vector2> input = d_input.getValue() ;
        helper::vector<defaulttype::Vector3> & output = *d_output.beginEdit() ;
        output.clear () ;
        for (defaulttype::Vector2 vec : input) {
            size_t i = vec[1], j = vec[0] ;
            if (depth_im.at<const uchar>(i, j) > d_minmax.getValue()[0] &&
                depth_im.at<const uchar>(i, j) < d_minmax.getValue()[1]) {
                // deprojection
                float dist = depth.get_distance(j, i) ;
                int index = static_cast<int>(i/downSample) * diststruct._width +
                    static_cast<int>(j/downSample) ;
                push_to_pointcloud (i, j, index, dist, diststruct, output) ;
            }
        }
        // the end
        d_output.endEdit();

        //export pos to txt
        static int i = 0 ; // frame num
        std::string ss = std::to_string(i) + "\t" ;
        for (const auto & point : output) {
            ss += std::to_string(point[0]) + " " + std::to_string(point[1]) + " " + std::to_string(point[2]) + "\t" ;
        }
        i ++ ;
        ss += "\n" ;
        if (i == 1) {
            std::ofstream ff ("/home/omar/Data/SergeiExp/node_x.txt", std::ofstream::out) ;
            ff << ss ;
            ff.close();
        } else {
            std::ofstream ff ("/home/omar/Data/SergeiExp/node_x.txt", std::ofstream::app) ;
            ff << ss ;
            ff.close();
        }
    }

    void push_to_pointcloud (int i, int j, int index, float dist, RealSenseDistFrame::RealSenseDistStruct & diststruct, helper::vector<defaulttype::Vector3> & output) {
        float
            point3d[3] = {0.f, 0.f, 0.f},
            point2d[2] = {i, j};
        rs2_deproject_pixel_to_point(
            point3d,
            &cam_intrinsics,
            point2d,
            dist
        );
        diststruct.frame[index] = dist ;

        pcl::PointXYZ pt = pcl::PointXYZ(point3d[1], -point3d[0], -point3d[2]) ;
        m_pointcloud->push_back(pt);

        defaulttype::Vector3 deprojected_point = defaulttype::Vector3(pt.x, pt.y, pt.z) ;
        output.push_back(deprojected_point) ;
    }

};

}

}

