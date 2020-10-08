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

//#include <sofa/realsenseplugin/streamer/RealSenseStreamer.h>

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/realsenseplugin/RSData.h>

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

    sofa::core::objectmodel::DataFileName d_path ;

    Data<RealSenseDataFrame> d_rsdataframe ;

    Data<double> d_fps ;

    core::objectmodel::DataCallback c_dataframe, c_path ;

	RealSenseExporter()
        : Inherited()
        , d_path(initData(&d_path, "path", "path to 3D video to write"))

        , d_rsdataframe(initData(&d_rsdataframe, "rsframe", "input rgbd data frame"))

        , d_fps(initData(&d_fps, 25.0, "fps", "frame rate"))
    {
        c_dataframe.addInputs({&d_rsdataframe});
        c_dataframe.addCallback(std::bind(&RealSenseExporter::write_dataframe, this));
        c_path.addInputs({&d_path});
        c_path.addCallback(std::bind(&RealSenseExporter::changed_path, this));
    }

    void write_dataframe () {
        if (m_filestream.is_open()) {
        // write dataframe to output stream
            m_filestream << d_rsdataframe.getValue() ;
        }
    }
    void changed_path () {
        if (m_filestream.is_open()) m_filestream.close();
        m_filestream.open(d_path.getFullPath(), std::ofstream::binary);
    }

    void init () {
        m_filestream.open(d_path.getFullPath(), std::ofstream::binary);
    }

	~RealSenseExporter () {
        if (m_filestream.is_open()) m_filestream.close();
    }
protected :
    std::ofstream m_filestream ;

};

}

}

