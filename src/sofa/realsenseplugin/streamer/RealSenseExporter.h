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
 * \brief The RealSenseExporter class
 * Streamer to use when only one realsense is connected
 */
class RealSenseExporter : public core::objectmodel::BaseObject
{
public:
	SOFA_CLASS( RealSenseExporter , core::objectmodel::BaseObject );
	typedef core::objectmodel::BaseObject Inherited;

    sofa::core::objectmodel::DataFileName d_path_c ;
    sofa::core::objectmodel::DataFileName d_path_d ;
    sofa::core::objectmodel::DataFileName d_path_dist ;

    Data<opencvplugin::ImageData> d_color ;
	Data<opencvplugin::ImageData> d_depth ;
    Data<RealSenseDistFrame> d_dist ;

    Data<double> d_fps ;

    DataCallback c_color, c_depth, c_dist ;

    RealSenseDistFrameExporter distout ;
    cv::VideoWriter writer_rgb, writer_d ;

	RealSenseExporter()
        : Inherited()
        , d_path_c(initData(&d_path_c, "pathc", "path to 3D video to read"))
        , d_path_d(initData(&d_path_d, "pathd", "path to 3D video to read"))
        , d_path_dist(initData(&d_path_dist, "pathdist", "path to 3D video to read"))

        , d_color(initData(&d_color, "color", "color frame"))
        , d_depth(initData(&d_depth, "depth", "depth frame"))
        , d_dist(initData(&d_dist, "dist", "dist frame"))

        , d_fps(initData(&d_fps, 30.0, "fps", "frame rate"))
    {
        c_color.addInputs({&d_color});
        c_color.addCallback(std::bind(&RealSenseExporter::write_color, this));

        c_depth.addInputs({&d_depth});
        c_depth.addCallback(std::bind(&RealSenseExporter::write_depth, this));

        c_dist.addInputs({&d_dist});
        c_dist.addCallback(std::bind(&RealSenseExporter::write_dist, this));
    }

    void write_color () {
        writer_rgb << d_color.getValue().getImage() ;
    }
    void write_depth () {
        writer_d << d_depth.getValue().getImage() ;
    }
    void write_dist () {
        distout.d_distframe.setValue(d_dist.getValue());
    }

    void init () {
        distout.d_filename.setValue(d_path_dist.getValue());
        cv::Mat color = d_color.getValue().getImage() ;
        writer_rgb.open(
            d_path_c.getValue(),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            d_fps.getValue(),
            cv::Size(color.rows, color.cols),
            (color.channels() != 1)
        ) ;
        cv::Mat depth = d_depth.getValue().getImage() ;
        writer_d.open(
            d_path_d.getValue(),
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            d_fps.getValue(),
            cv::Size(depth.rows, depth.cols),
            (depth.channels() != 1)
        ) ;
    }

	~RealSenseExporter () {
        if (writer_rgb.isOpened()) writer_rgb.release();
        if (writer_d.isOpened()) writer_d.release();
    }

};

}

}

