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

//#include <CImgPlugin/CImgData.h>
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

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/opencvplugin/OpenCVWidget.h>
#include <sofa/opencvplugin/utils/OpenCVMouseEvents.h>

#include <sofa/realsenseplugin/streamer/RealSenseCam.h>
#include <sofa/realsenseplugin/projector/RealSenseDistFrame.h>
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
            vparams->drawTool()->drawSphere(output[i], 0.008);
        }
    }

private :
    virtual void writeOfflineToOutput (RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) {
        // setup output
        const helper::vector<defaulttype::Vector2> input = d_input.getValue() ;
        helper::vector<defaulttype::Vector3> & output = *d_output.beginEdit() ;
        output.clear () ;
        for (defaulttype::Vec2i vec : input){
            float dist = diststruct.frame[vec[0]*diststruct._width+vec[1]] ;
            push_to_pointcloud(vec[0], vec[1], dist, diststruct, output) ;
        }
        // the end
        d_output.endEdit();
    }

    virtual void writeOnlineToOutput (rs2::depth_frame & depth, RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) {
        // setup output
        const helper::vector<defaulttype::Vector2> input = d_input.getValue() ;
        helper::vector<defaulttype::Vector3> & output = *d_output.beginEdit() ;
        output.clear () ;
        for (defaulttype::Vector2 vec : input) {
            float dist = depth.get_distance(vec[1], vec[0]) ;
            push_to_pointcloud (vec[0], vec[1], dist, diststruct, output) ;
        }
        // the end
        d_output.endEdit();
    }

    void push_to_pointcloud (int i, int j, float dist, RealSenseDistFrame::RealSenseDistStruct & diststruct, helper::vector<defaulttype::Vector3> & output) {
        float
            point3d[3] = {0.f, 0.f, 0.f},
            point2d[2] = {i, j};
        rs2_deproject_pixel_to_point(
            point3d,
            &cam_intrinsics,
            point2d,
            dist
        );
        diststruct.frame[i*diststruct._width+j] = dist ;
        defaulttype::Vector3 deprojected_point = defaulttype::Vector3(point3d[1], -point3d[2], -point3d[0]) ;
        output.push_back(deprojected_point) ;
    }

};

}

}

