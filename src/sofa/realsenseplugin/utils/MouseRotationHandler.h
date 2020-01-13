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

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/OptionsGroup.h>


#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <librealsense2/rs.hpp>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>

#include <sofa/realsenseplugin/streamer/RealSenseCam.h>
#include <sofa/realsenseplugin/cv-helpers.hpp>

namespace sofa
{

namespace rgbdtracking
{

class MouseRotationHandler : public core::objectmodel::BaseObject {
public :
    SOFA_CLASS( MouseRotationHandler, core::objectmodel::BaseObject);
    typedef core::objectmodel::BaseObject Inherited;

    Data<defaulttype::Vector3> d_rotation ;

    defaulttype::Vector3 phi ; //accumulated rotation angles
    int x_0, y_0, x_1, y_1 ;

    MouseRotationHandler()
        : Inherited()
        , d_rotation(initData(&d_rotation, defaulttype::Vector3(0,0,0), "rotation", "rotation angles"))
        , phi(defaulttype::Vector3(0,0,0))
    {
        this->f_listening.setValue(true) ;
    }

    void handleEvent(sofa::core::objectmodel::Event* event) override {
        if (sofa::core::objectmodel::MouseEvent * ev = dynamic_cast<sofa::core::objectmodel::MouseEvent*>(event)){
            static int selected_dim = -1 ; // 0 x, 1 y, 2 z
            if (ev->getState() == core::objectmodel::MouseEvent::LeftPressed) {
//            set x
                x_0 = ev->getPosX() ; y_0 = ev->getPosY() ;
                selected_dim = 0 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::RightPressed) {
//            set y
                x_0 = ev->getPosX() ; y_0 = ev->getPosY() ;
                selected_dim = 1 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::MiddlePressed) {
//            set z
                x_0 = ev->getPosX() ; y_0 = ev->getPosY() ;
                selected_dim = 2 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::LeftReleased ||
                ev->getState() == core::objectmodel::MouseEvent::RightReleased ||
                ev->getState() == core::objectmodel::MouseEvent::MiddleReleased) {
                selected_dim = -1 ;
            }
            else if (ev->getState() == core::objectmodel::MouseEvent::Move) {
                x_1 = ev->getPosX() ; y_1 = ev->getPosY() ;
                if(x_0 == x_1) {
                    x_1 = 1 ; x_0 = 0 ;
                }
                float a = -(float)(y_1-y_0) / 15.f ;//(float)(x_1-x_0) ;

                if (selected_dim >= 0 && selected_dim <= 2) {
                    phi[selected_dim] += a/2.f ;
                    if ((int)phi[selected_dim] > 180)
                        phi[selected_dim] = 180.f ;
                    else if ((int)phi[selected_dim] < -180)
                        phi[selected_dim] = -180.f ;
                    std::cout << phi << std::endl ;
                }
//                x_0 = x_1 ; y_0 = y_1 ;
            }
            d_rotation.setValue(phi);


        }
    }

} ;

}

}

