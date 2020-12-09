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
 * \brief The RealSenseDeprojector_old class
 * Online / Offline 2D-3D deprojection of whole rgb-d scene
 */
class RealSenseDeprojector_old : public RealSenseAbstractDeprojector_old
{

public:
    typedef RealSenseAbstractDeprojector_old Inherited;
    SOFA_CLASS( RealSenseDeprojector_old , Inherited);

    /// \brief color frame for snapshot exportation
    Data<opencvplugin::ImageData> d_color ;
    /// \brief path to save snapshots to
    Data<std::string> d_snap_path ;

    DataCallback c_image ;

    RealSenseDeprojector_old()
        : Inherited()
        , d_color(initData(&d_color, "color", "segmented color data image"))
        //offline reco
        , d_snap_path(initData(&d_snap_path, std::string("."), "snap_path", "path to snap shots folder"))
    {
        c_image.addInputs({&this->d_depth});
        c_image.addCallback(std::bind(&RealSenseDeprojector_old::deproject_image, this));
    }

    virtual ~RealSenseDeprojector_old () {
    }

    /*!
     * \brief handleEvent : Press P to export 3D snapshot of current frame
     * \param event
     */
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

    /*!
     * \brief _write_distFrame
     * \param snapshot_dist_filename
     */
    inline void _write_distFrame (std::string snapshot_dist_filename) {
        RealSenseDistFrame distframe = d_distframe.getValue() ;
        RealSenseDistFrame::RealSenseDistStruct diststruct = distframe.getFrame();

        std::FILE* filestream = std::fopen(snapshot_dist_filename.c_str(), "wb") ;
        if (filestream == nullptr) {
            std::cerr << "(RealSenseDeprojector_old) check writing rights on file : "
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

    /*!
     * \brief writeOfflineToOutput : implementation to reproject the whole scene/frame offline
     * \param diststruct
     * \param depth_im
     * \param downSample
     */
    virtual void writeOfflineToOutput (RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear() ;
        m_pointcloud->clear();
        for (size_t i = 0 ; i < diststruct._height ; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > d_minmax.getValue()[0] &&
                    depth_im.at<const uchar>(downSample*i,downSample*j) < d_minmax.getValue()[1]
                ) {
                    // deprojection
                    float dist = diststruct.frame[i*diststruct._width+j] ;
                    int index = i*diststruct._width+j ;
                    push_to_pointcloud(outpoints, downSample*i, downSample*j, index, diststruct, dist);
                }
            }
        }
//        std::cout << outpoints.size() << std::endl ;
        d_output.endEdit();
    }

    /*!
     * \brief writeOnlineToOutput : implementation to reproject the whole scene online
     * \param depth
     * \param diststruct
     * \param depth_im
     * \param downSample
     */
    virtual void writeOnlineToOutput (rs2::depth_frame & depth, RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear() ;
        for (size_t i = 0 ; i < diststruct._height; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > d_minmax.getValue()[0] &&
                    depth_im.at<const uchar>(downSample*i,downSample*j) < d_minmax.getValue()[1]
                ) {
                    // deprojection
                    float dist = depth.get_distance(downSample*j, downSample*i) ;
                    int index = i*diststruct._width+j ;
                    push_to_pointcloud(outpoints, downSample*i, downSample*j, index, diststruct, dist);
                }
            }
        }
        d_output.endEdit();
    }

};

}

}

