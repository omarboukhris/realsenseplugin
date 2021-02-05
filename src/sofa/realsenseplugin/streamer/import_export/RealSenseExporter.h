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
#include <sofa/core/objectmodel/DataCallback.h>

#include <fstream>

namespace sofa
{

namespace realsenseplugin
{

/*!
 * \brief The RealSenseExporter class
 * Streamer to use when only one realsense is connected
 */
class RealSenseExporter : public core::objectmodel::BaseObject
{
public:
	SOFA_CLASS( RealSenseExporter , core::objectmodel::BaseObject );
	typedef core::objectmodel::BaseObject Inherited;

    /// \brief path to color video
    sofa::core::objectmodel::DataFileName d_path_color ;
    /// \brief path to depth video
    sofa::core::objectmodel::DataFileName d_path_depth ;
    /// \brief path to collection of pointcloud frames
    sofa::core::objectmodel::DataFileName d_path_pcl ;

    /// \brief output data frame
    Data<RealSenseDataFrame> d_rsdataframe ;

    /// \brief fps for saving video
    Data<double> d_fps ;

    core::objectmodel::DataCallback c_dataframe, c_path, c_path_depth, c_path_pcl ;

	RealSenseExporter()
        : Inherited()
        , d_path_color(initData(&d_path_color, "pathcolor", "path to 3D video to write"))
        , d_path_depth(initData(&d_path_depth, "pathdepth", "depth path to 3D video to write"))
        , d_path_pcl(initData(&d_path_pcl, "pathpcl", "path to pointcloud video to write"))

        , d_rsdataframe(initData(&d_rsdataframe, "rsframe", "input rgbd data frame"))

        , d_fps(initData(&d_fps, 25.0, "fps", "frame rate"))
    {
        c_dataframe.addInputs({&d_rsdataframe});
        c_dataframe.addCallback(std::bind(&RealSenseExporter::write_dataframe, this));
        c_path.addInputs({&d_path_color});
        c_path.addCallback(std::bind(&RealSenseExporter::changed_path, this));
        c_path_depth.addInputs({&d_path_depth});
        c_path_depth.addCallback(std::bind(&RealSenseExporter::changed_path_depth, this));
        c_path_pcl.addInputs({&d_path_pcl});
        c_path_pcl.addCallback(std::bind(&RealSenseExporter::changed_path_pcl, this));
    }

    /*!
     * \brief write_pointcloud_to_file
     * write pointcloud from rs data frame to opened file stream
     */
    void write_pointcloud_to_file()
    {
        const rgbd_frame::sofaPointCloud & pcloud = d_rsdataframe.getValue().getsofapointcloud() ;
        int ptsize = pcloud.size() ;
        ptcloud_fstream.write(reinterpret_cast<char*>(&ptsize), sizeof(ptsize)) ;
        for (const defaulttype::Vector3 & pt : pcloud) {
            ptcloud_fstream.write(reinterpret_cast<const char*>(&pt[0]), sizeof(pt[0])) ;
            ptcloud_fstream.write(reinterpret_cast<const char*>(&pt[1]), sizeof(pt[1])) ;
            ptcloud_fstream.write(reinterpret_cast<const char*>(&pt[2]), sizeof(pt[2])) ;
        }
    }

    /*!
     * \brief write_dataframe
     * dispatches data writing to the proper methods for each type of data
     */
    void write_dataframe () {
        // write dataframe to output stream
        if (m_colorstream.isOpened()) {
            m_colorstream << d_rsdataframe.getValue().getcvColor() ;
        }
        if (m_depthstream.isOpened()) {
            m_depthstream << d_rsdataframe.getValue().getcvDepth() ;
        }
        if (ptcloud_fstream.is_open()) {
            write_pointcloud_to_file();
        }
    }

    /*!
     * \brief openVideoStream
     * \param stream reference to the stream to open
     * \param path full path
     * \param img image for size and channels
     */
    void openVideoStream(cv::VideoWriter & stream, const std::string path, const cv::Mat img)
    {
        stream.open(
            path,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            d_fps.getValue(),
            cv::Size(img.cols, img.rows),
            (img.channels() != 1)
        );
    }

    /*!
     * \brief changed_path
     * updates opened color stream if filename changed in simulation
     */
    void changed_path () {
        if (m_colorstream.isOpened()) m_colorstream.release();
	if (d_path_color.getFullPath().empty()) return ;
        openVideoStream(
            m_colorstream,
            d_path_color.getFullPath(),
            d_rsdataframe.getValue().getcvColor() );
    }

    /*!
     * \brief changed_path_depth
     * updates opened depth stream if filename changed in simulation
     */
    void changed_path_depth () {
        if (m_depthstream.isOpened()) m_depthstream.release();
	if (d_path_depth.getFullPath().empty()) return ;
        openVideoStream(
            m_depthstream,
            d_path_depth.getFullPath(),
            d_rsdataframe.getValue().getcvDepth() );
    }

    /*!
     * \brief changed_path_pcl
     * updates opened pointcloud stream if filename changed in simulation
     */
    void changed_path_pcl () {
        if (ptcloud_fstream.is_open()) ptcloud_fstream.close();
        ptcloud_fstream = std::ofstream(d_path_pcl.getFullPath(), std::ios::binary);
    }

    void init () {
        openVideoStream(
            m_colorstream,
            d_path_color.getFullPath(),
            d_rsdataframe.getValue().getcvColor() );
        openVideoStream(
            m_depthstream,
            d_path_depth.getFullPath(),
            d_rsdataframe.getValue().getcvDepth() );
        ptcloud_fstream = std::ofstream(d_path_pcl.getFullPath(), std::ios::binary);
    }

	~RealSenseExporter () {
        if (m_colorstream.isOpened()) m_colorstream.release();
        if (m_depthstream.isOpened()) m_depthstream.release();
        if (ptcloud_fstream.is_open()) ptcloud_fstream.close();
    }
protected :
    /// \brief resp. opencv color and depth streams
    cv::VideoWriter m_colorstream, m_depthstream ;
    /// \brief file stream for reading pointcloud frames
    std::ofstream ptcloud_fstream ;

};

}

}

