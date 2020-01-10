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

class RealSenseMaskDeprojector : public RealSenseAbstractDeprojector
{
public:
    typedef RealSenseAbstractDeprojector Inherited;
    SOFA_CLASS( RealSenseMaskDeprojector , Inherited);

    Data<opencvplugin::ImageData> d_input ;
    DataCallback c_image ;

    RealSenseMaskDeprojector()
        : Inherited()
        , d_input(initData(&d_input, "input", "input mask to de-project"))
    {
        c_image.addInputs({&d_input});
        c_image.addCallback(std::bind(&RealSenseMaskDeprojector::deproject_image, this));
    }

    virtual ~RealSenseMaskDeprojector () {
    }

private :
    virtual void writeOfflineToOutput (RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        // setup output
        // const cv::Mat & input =  d_input.getValue().getImage() ;
        cv::Mat input ;
        d_input.getValue().getImage().copyTo(input);
        cv::cvtColor(d_input.getValue().getImage(), input, cv::COLOR_BGR2GRAY); ;
        helper::vector<defaulttype::Vector3> & output = *d_output.beginEdit() ;
        output.clear () ;
        for (size_t i = 0 ; i < diststruct._height; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > d_minmax.getValue()[0] &&
                    depth_im.at<const uchar>(downSample*i,downSample*j) < d_minmax.getValue()[1] &&
                    input.at<const uchar>(downSample*i,downSample*j, 0) > 1
                ) {
                // deprojection
                    float dist = diststruct.frame[i*diststruct._width+j] ;
                    int index = i*diststruct._width+j ;
                    push_to_pointcloud(output, downSample*i, downSample*j, index, diststruct, dist);
                }
            }
        }
        // the end
        d_output.endEdit();
    }

    virtual void writeOnlineToOutput (rs2::depth_frame & depth, RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        // setup output
        // const cv::Mat & input =  d_input.getValue().getImage() ;
        const cv::Mat & input = d_input.getValue().getImage() ; //.copyTo(input);
        //cv::cvtColor(d_input.getValue().getImage(), input, CV_BGR2GRAY); ;
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear () ;
        for (size_t i = 0 ; i < diststruct._height; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > d_minmax.getValue()[0] &&
                    depth_im.at<const uchar>(downSample*i,downSample*j) < d_minmax.getValue()[1] &&
                    input.at<const uchar>(downSample*i,downSample*j, 0) > 1
                ) {
                    // deprojection
                    float dist = depth.get_distance(downSample*j, downSample*i) ;
                    int index = i*diststruct._width+j ;
                    push_to_pointcloud(outpoints, downSample*i, downSample*j, index, diststruct, dist);
                }
            }
        }
        // the end
        d_output.endEdit();
    }
};

}

}

