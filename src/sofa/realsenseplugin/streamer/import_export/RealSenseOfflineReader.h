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

#include <sofa/realsenseplugin/RSData.h>

#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

namespace sofa
{

namespace realsenseplugin
{

/*!
 * \brief The RealSenseOfflineReader class
 * Streamer to use when reprocessing a pointcloud
 */
class RealSenseOfflineReader : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS( RealSenseOfflineReader , core::objectmodel::BaseObject );
	typedef core::objectmodel::BaseObject Inherited;

    /// \brief path to color video
    sofa::core::objectmodel::DataFileName d_path_color ;
    /// \brief path to depth video
    sofa::core::objectmodel::DataFileName d_path_depth ;
    /// \brief path to collection of pointcloud frames
    sofa::core::objectmodel::DataFileName d_path_pcl ;

    /// \brief output data frame
    Data<RealSenseDataFrame> d_rsframe ;
    /// \brief output pointcloud
    Data<rgbd_frame::sofaPointCloud> d_output ;
    /// \brief synthetic volume pointcloud
    Data<rgbd_frame::sofaPointCloud> d_synthvolume ;

    /// \brief true to add synthetic volume
    Data<int> d_densify ;
    /// \brief true to render pointcloud in viewer
    Data<bool> d_drawpcl ;
    /// \brief true to render synthetic pointcloud
    Data<bool> d_drawsynth ;

    /// \brief resp. opencv color and depth streams
    cv::VideoCapture _reader_color, _reader_depth ;
    /// \brief file stream for saving pointcloud frames
    std::ifstream ptcloud_fstream ;
    /// \brief pause
    bool paused ;

    RealSenseOfflineReader()
        : Inherited()
        , d_path_color(initData(&d_path_color, "pathcolor", "path to 3D video to write"))
        , d_path_depth(initData(&d_path_depth, "pathdepth", "depth path to 3D video to write"))
        , d_path_pcl(initData(&d_path_pcl, "pathpcl", "path to pointcloud video to write"))
        , d_rsframe(initData(&d_rsframe, "rsframe", "input rgbd data frame"))
        , d_output(initData(&d_output, "output", "output pointcloud"))
        , d_synthvolume(initData(&d_synthvolume, "synthvol", "synthetic volume for ICP optimization"))
        , d_densify(initData(&d_densify, 0, "densify", "densify pointcloud to approximate volume (naive method)"))
        , d_drawpcl(initData(&d_drawpcl, true, "drawpcl", "true if you want to draw the point cloud"))
        , d_drawsynth(initData(&d_drawsynth, false, "drawsynth", "true if you want to draw the synthetic point cloud"))
        , paused (false)
    {
        this->f_listening.setValue(true);
    }

    /*!
     * \brief init
     * open color, depth and pointcloud video/file streams
     */
    void init () {
        opencolor () ;
        opendepth () ;
        openpcl();
	}

    ~RealSenseOfflineReader () {
        if (_reader_color.isOpened()) _reader_color.release();
        if (_reader_depth.isOpened()) _reader_depth.release();
        if (ptcloud_fstream.is_open()) ptcloud_fstream.close();
    }

    /*!
     * \brief draws the pointcloud read from file
     * \param vparams
     */
    void draw(const core::visual::VisualParams* vparams) {
        if (!d_drawpcl.getValue()) {
        // don't draw point cloud
            return ;
        }
        rgbd_frame::sofaPointCloud output = d_rsframe.getValue().getsofapointcloud() ;
        for (unsigned int i=0; i< output.size(); i++) {
            vparams->drawTool()->drawSphere(output[i], 0.002);
        }
        if (!d_drawsynth.getValue()) {
        // don't draw point cloud
            return ;
        }
        output = d_synthvolume.getValue() ;
        for (unsigned int i=0; i< output.size(); i++) {
            vparams->drawTool()->drawSphere(output[i], 0.0015);
        }

    }

    /*!
     * \brief properly open color stream
     */
    inline void opencolor () {
        if (_reader_color.isOpened()) _reader_color.release();
        if (d_path_color.getFullPath().empty()) return ;
        _reader_color = cv::VideoCapture(d_path_color.getFullPath()) ;
    }
    /*!
     * \brief properly open depth stream
     */
    inline void opendepth () {
        if (_reader_depth.isOpened()) _reader_depth.release();
        if (d_path_depth.getFullPath().empty()) return ;
        _reader_depth = cv::VideoCapture(d_path_depth.getFullPath()) ;
    }
    /*!
     * \brief properly open pointcloud stream
     */
    inline void openpcl () {
        if (ptcloud_fstream.is_open()) ptcloud_fstream.close();
        ptcloud_fstream.open(d_path_pcl.getFullPath(), std::ios::binary);
    }
    void handleEvent(sofa::core::objectmodel::Event* event) {
        if(sofa::simulation::AnimateBeginEvent::checkEventType(event)) {
            if (paused) {
            // don't read frame
                return ;
            }
            d_rsframe.setValue(
                RealSenseDataFrame(
                    readcolor (),
                    readdepth (),
                    readpcl()
            )) ;
            makeSyntheticVolume();
        }
        if (sofa::core::objectmodel::KeypressedEvent * ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)){
            if (ev->getKey() == ' ') {
                paused = ! paused ;
            }
        }
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
        rgbd_frame::sofaPointCloud & synth = *d_synthvolume.beginEdit();
        rgbd_frame::sofaPointCloud surface = d_output.getValue() ;
        synth.clear() ;
        for (const auto & pt : surface) {
            for (int i = 1 ; i<dense ; i++) {
                defaulttype::Vector3 point = defaulttype::Vector3(pt[0], pt[1], pt[2] - i*1e-2) ;
                synth.push_back(point) ;
            }
        }
        d_synthvolume.endEdit();
    }

    /*!
     * \brief readcolor
     * \return read color frame from opened stream
     */
    cv::Mat readcolor () {
        cv::Mat out ;
        if (!_reader_color.isOpened()) return out ;
        _reader_color.retrieve(out) ;
        return out ;
    }
    /*!
     * \brief readdepth
     * \return read depth frame from opened stream
     */
    cv::Mat readdepth () {
        cv::Mat out ;
        if (!_reader_depth.isOpened()) return out ;
        _reader_depth.retrieve(out) ;
        return out ;
    }
    /*!
     * \brief readpcl
     * \return pointcloud read from opened stream
     */
    rgbd_frame::sofaPointCloud readpcl () {
        if (!ptcloud_fstream.is_open()) {
        // stream is not opened
            return rgbd_frame::sofaPointCloud () ;
        }
        if (ptcloud_fstream.peek()==EOF) {
        // end of file reached
            return rgbd_frame::sofaPointCloud () ;
        }
        //first read frame size
        int size = 0 ;
        ptcloud_fstream.read(reinterpret_cast<char*>(&size), sizeof(size));
        rgbd_frame::sofaPointCloud & out = *d_output.beginEdit() ;
        out = rgbd_frame::sofaPointCloud(size);
        // then read positions
        for (int i = 0 ; i < size ; i++) {
            defaulttype::Vector3 pt (0,0,0) ;
            ptcloud_fstream.read(reinterpret_cast<char*>(&pt[0]), sizeof(pt[0])) ;
            ptcloud_fstream.read(reinterpret_cast<char*>(&pt[1]), sizeof(pt[1])) ;
            ptcloud_fstream.read(reinterpret_cast<char*>(&pt[2]), sizeof(pt[2])) ;
            out[i] = pt ;
        }
        d_output.endEdit();
        return out ;
    }

};

}

}

