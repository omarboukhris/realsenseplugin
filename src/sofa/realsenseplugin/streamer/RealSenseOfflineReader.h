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

#include <sofa/realsenseplugin/streamer/RealSenseStreamer.h>
#include <sofa/realsenseplugin/projector/RealSenseDistFrame.h>
#include <sofa/opencvplugin/OpenCVWidget.h>

namespace sofa
{

namespace rgbdtracking
{

using namespace cimg_library;
using defaulttype::Vec;
using defaulttype::Vector3;


using namespace std;
using namespace cv;
using namespace boost;
using namespace rs2;

/*!
 * \brief The RealSenseOfflineReader class
 * Streamer to use when only one realsense is connected
 */
class RealSenseOfflineReader : public core::objectmodel::BaseObject
{
public:
    SOFA_CLASS( RealSenseOfflineReader , core::objectmodel::BaseObject );
	typedef core::objectmodel::BaseObject Inherited;

    sofa::core::objectmodel::DataFileName d_path_c ;
    sofa::core::objectmodel::DataFileName d_path_d ;
    sofa::core::objectmodel::DataFileName d_path_dist ;

	Data<opencvplugin::ImageData> d_color ;
	Data<opencvplugin::ImageData> d_depth ;
	Data<RealSenseDistFrame> d_dist ;

    cv::VideoCapture _reader_color, _reader_depth ;
    std::FILE* _dist_fstream ;

    RealSenseOfflineReader()
        : Inherited()
        , d_path_c(initData(&d_path_c, "pathc", "path to 3D video to read"))
        , d_path_d(initData(&d_path_d, "pathd", "path to 3D video to read"))
        , d_path_dist(initData(&d_path_dist, "pathdist", "path to 3D video to read"))
        , d_color (initData(&d_color , "color", "read color frame"))
        , d_depth (initData(&d_depth , "depth", "read depth frame"))
        , d_dist (initData(&d_dist , "dist", "read dist frame"))
    {}

    void init () {
        this->f_listening.setValue(true);
        opencolor () ;
        opendepth () ;
        opendist  () ;
	}

    ~RealSenseOfflineReader () {
        if (_reader_color.isOpened()) _reader_color.release();
        if (_reader_depth.isOpened()) _reader_depth.release();
        if (_dist_fstream != NULL) std::fclose(_dist_fstream) ;
    }

    inline void opencolor () {
        if (_reader_color.isOpened()) _reader_color.release();
        _reader_color = cv::VideoCapture(d_path_c.getFullPath()) ;
    }
    inline void opendepth () {
        if (_reader_depth.isOpened()) _reader_depth.release();
        _reader_depth = cv::VideoCapture(d_path_d.getFullPath()) ;
    }
    inline void opendist () {
        if (_dist_fstream != NULL) std::fclose(_dist_fstream) ;
        _dist_fstream = std::fopen(d_path_dist.getFullPath().c_str(), "rb") ;
    }

    void handleEvent(sofa::core::objectmodel::Event* event) {
        if(sofa::simulation::AnimateBeginEvent::checkEventType(event)) {
            readcolor () ;
            readdepth () ;
            readdist () ;
        }
    }

    void readcolor () {
        if (!_reader_color.isOpened()) return ;
        if (!_reader_color.grab()) return ;
        _reader_color.retrieve(d_color.beginEdit()->getImage()) ;
        d_color.endEdit();
    }
    void readdepth () {
        if (!_reader_depth.isOpened())return ;
        if (!_reader_depth.grab()) return ;
        _reader_depth.retrieve(d_depth.beginEdit()->getImage()) ;
        d_depth.endEdit();
    }
    void readdist  () {
        if (_dist_fstream == NULL) {
            std::cerr << "(RealSenseOfflineReader) stream is unopened. check stream state before passing to function" << std::endl ;
            return ;
        }
        if (std::feof(_dist_fstream)) {
            std::cout << "(RealSenseOfflineReader) End of dist file reached" << std::endl ;
            std::fclose(_dist_fstream) ;
            return ;
        }
        RealSenseDistFrame::RealSenseDistStruct diststruct ;
        // write width and height
        std::fread(&diststruct._width, sizeof(size_t), 1, _dist_fstream) ;
        std::fread(&diststruct._height, sizeof(size_t), 1, _dist_fstream) ;

        // write frame data
        diststruct.frame = new float[diststruct._width * diststruct._height] ;
        std::fread (
            diststruct.frame,
            sizeof(float),
            diststruct._width * diststruct._height,
            _dist_fstream
        ) ;
        RealSenseDistFrame distFrm (diststruct) ;
        d_dist.setValue(distFrm);
    }

};

}

}

