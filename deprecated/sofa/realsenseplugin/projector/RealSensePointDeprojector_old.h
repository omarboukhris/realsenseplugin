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

#include <sofa/realsenseplugin/projector/RealSenseAbstractProjector_old.h>

namespace sofa {

namespace rgbdtracking {

/*!
 * \brief The RealSensePointDeprojector_old class
 * deprojects only a defined  set of 2D points defined in d_input (label input in sofa) from depth frame
 */
class RealSensePointDeprojector_old : public RealSenseAbstractDeprojector_old
{
public:
    typedef RealSenseAbstractDeprojector_old Inherited;
    SOFA_CLASS( RealSensePointDeprojector_old , Inherited);

    /// \brief input list of 2d points to project in 3d
    Data<helper::vector<defaulttype::Vector2> > d_input ;
    DataCallback c_image ;

    RealSensePointDeprojector_old()
        : Inherited()
        , d_input(initData(&d_input, "input", "input 2D position to de-project"))
    {
        c_image.addInputs({&d_input});
        c_image.addCallback(std::bind(&RealSensePointDeprojector_old::deproject_image, this));
    }

    virtual ~RealSensePointDeprojector_old () {
    }

private :
    virtual void writeOfflineToOutput (RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        // setup output
        const helper::vector<defaulttype::Vector2> input = d_input.getValue() ;
        helper::vector<defaulttype::Vector3> & output = *d_output.beginEdit() ;
        output.clear () ;
        for (defaulttype::Vec2i vec : input){
            size_t i = vec[1], j = vec[0] ;
            if (depth_im.at<const uchar>(i, j) > d_minmax.getValue()[0] &&
                depth_im.at<const uchar>(i, j) < d_minmax.getValue()[1]) {
            // deprojection
                int index = static_cast<int>(i/downSample) * diststruct._width +
                    static_cast<int>(j/downSample) ;
                float dist = diststruct.frame[index] ;
                push_to_pointcloud (output, i, j, index, diststruct, dist) ;
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
            // works for point selector
            size_t i = vec[1], j = vec[0] ;
            if (depth_im.at<const uchar>(i, j) > d_minmax.getValue()[0] &&
                depth_im.at<const uchar>(i, j) < d_minmax.getValue()[1]) {
                // deprojection
                float dist = depth.get_distance(j, i) ;
                int index = static_cast<int>(i/downSample) * diststruct._width +
                    static_cast<int>(j/downSample) ;
                push_to_pointcloud (output, i, j, index, diststruct, dist) ;
            }
        }
        // the end
        d_output.endEdit();
    }

};

}

}

