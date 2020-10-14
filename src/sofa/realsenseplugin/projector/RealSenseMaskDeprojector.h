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

namespace realsenseplugin {

/*!
 * \brief The RealSenseMaskDeprojector class
 * deprojects a set of points depending on a 2D binary mask defined as an image in d_input (label input in sofa)
 */
class RealSenseMaskDeprojector : public RealSenseAbstractDeprojector
{
public:
    typedef RealSenseAbstractDeprojector Inherited;
    SOFA_CLASS( RealSenseMaskDeprojector , Inherited);

    /// \brief input binary mask for filtering reprojections
    Data<RealSenseDataFrame> d_input ;
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

    virtual void writeToOutput (const cv::Mat & depth_im, int downSample) override {
        // setup output
        const cv::Mat & input = d_input.getValue().getcvColor() ; //.copyTo(input);
        //cv::cvtColor(d_input.getValue().getImage(), input, CV_BGR2GRAY); ;
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear () ;
        for (size_t i = 0 ; i < depth_im.rows/downSample ; ++i) {
            for (size_t j = 0 ; j < depth_im.cols/downSample ; ++j) {
                auto depth_ij = depth_im.at<const uchar>(downSample*i,downSample*j) ;
                if (depth_ij > d_minmax.getValue()[0] &&
                    depth_ij < d_minmax.getValue()[1] &&
                    input.at<const uchar>(downSample*i,downSample*j, 0) > 1
                ) {
                    // deprojection
                    push_to_pointcloud(outpoints, downSample*i, downSample*j,
                                       l_rs_cam->depth->get_distance(downSample*j,downSample*i));
                }
            }
        }
        // the end
        d_output.endEdit();
    }
};

}

}
