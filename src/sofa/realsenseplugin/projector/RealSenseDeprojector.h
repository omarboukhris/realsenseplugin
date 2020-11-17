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
 * \brief The RealSenseDeprojector class
 * Online / Offline 2D-3D deprojection of whole rgb-d scene
 */
class RealSenseDeprojector : public RealSenseAbstractDeprojector
{

public:
    typedef RealSenseAbstractDeprojector Inherited;
    SOFA_CLASS( RealSenseDeprojector , Inherited);

    /// \brief path to save snapshots to
    Data<std::string> d_snap_path ;

    DataCallback c_image ;

    RealSenseDeprojector()
        : Inherited()
        , d_snap_path(initData(&d_snap_path, std::string("."), "snap_path", "path to snap shots folder"))
    {
        f_listening.setValue(true);
        c_image.addInputs({&d_rsframe});
        c_image.addCallback(std::bind(&RealSenseDeprojector::deproject_image, this));
    }

    virtual ~RealSenseDeprojector () {
    }

    /*!
     * \brief handleEvent : Press P to export 3D snapshot of current frame
     * \param event
     */
    void handleEvent(sofa::core::objectmodel::Event *event) {
        Inherited::handleEvent(event) ;
        if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)) {
            // screen shot : ctrl+shift+P
            if ((int)ev->getKey() == 'P') {
                exportSnapShot () ;
            }
        }
    }
private :

    /*!
     * \brief exportSnapShot exports color and depth frames
     */
    inline void exportSnapShot () {
        static size_t i = 0 ; //< i is a snap shot 'id'
        // set filenames
        std::string
            snapshot_color_filename =
                d_snap_path.getValue() +
                "/rgb_snap_" + std::to_string(i) + ".png",
            snapshot_depth_filename =
                d_snap_path.getValue() +
                "/depth_snap_" + std::to_string(i) + ".png";
        i++ ;

        // get images to export
        cv::Mat colormat = d_rsframe.getValue().getcvColor(),
                depthmat = d_rsframe.getValue().getcvDepth() ;
        // export pngs
        cv::imwrite(snapshot_color_filename.c_str(), colormat) ;
        std::cout << "(RealSenseCam) exported " << snapshot_color_filename << std::endl ;
        cv::imwrite(snapshot_depth_filename.c_str(), depthmat) ;
        std::cout << "(RealSenseCam) exported " << snapshot_depth_filename << std::endl ;
    }

    /*!
     * \brief writeOnlineToOutput : implementation to reproject the whole scene online
     * \param depth
     * \param diststruct
     * \param depth_im
     * \param downSample
     */
    virtual void writeToOutput (const cv::Mat & depth_im, int downSample) override {
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear() ;
        for (size_t i = 0 ; i < depth_im.rows/downSample; ++i) {
            for (size_t j = 0 ; j < depth_im.cols/downSample ; ++j) {
                push_to_pointcloud(outpoints, downSample*i, downSample*j,
                                   l_rs_cam->depth->get_distance(downSample*j,downSample*i));
//                push_to_pointcloud(outpoints, depth_im, downSample*i, downSample*j);
            }
        }
        d_output.endEdit();
    }

};

}

}
