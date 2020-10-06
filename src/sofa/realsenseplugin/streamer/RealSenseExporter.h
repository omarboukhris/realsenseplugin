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

	Data<opencvplugin::ImageData> d_color ;
	Data<opencvplugin::ImageData> d_depth ;
	Data<RealSenseDistFrame> d_dist ;

	RealSenseExporter()
        : Inherited()
    {
	}

	~RealSenseExporter () {
    }


    void handleEvent(sofa::core::objectmodel::Event* event) {
    }

};

}

}

