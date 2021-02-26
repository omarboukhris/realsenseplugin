#pragma once

#include <sofa/helper/system/thread/CTime.h>
#include <sofa/helper/LCPcalc.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <SofaOpenglVisual/OglModel.h>
#include <math.h>
#include <SofaBaseLinearSolver/FullMatrix.h>
#include <SofaUserInteraction/Controller.h>
#include <sofa/core/objectmodel/BaseNode.h>

namespace sofa
{
namespace conect
{
using namespace defaulttype;

class SwapNodes : public core::objectmodel::BaseObject
{

public:

    SOFA_CLASS(SwapNodes, core::objectmodel::BaseObject);
    typedef Vec<6,bool> Vec6b;
    typedef Vec<6,double> Vec6d;
    typedef helper::vector<Vector3> VecCoord;
    typedef sofa::defaulttype::Vec3dTypes Vec3dTypes;
    typedef defaulttype::Vec3i Vec3i;
    typedef sofa::defaulttype::Rigid3dTypes Rigid3dTypes;

    typedef Rigid3dTypes::Coord Rigid;
    typedef SolidTypes<double>::Transform Transform;


    Data<double> d_t1;
    Data<double> d_t2;
    Data<bool> d_onDemand;
    Data<bool> d_noinit;
    Data<helper::vector<std::string > >  d_nodesD;
    Data<helper::vector<std::string > >  d_nodesA;

    SwapNodes();

//    sofa::core::behavior::SlindingConstraint<defaulttype::Vec3dTypes>* m_constraint;

    void bwdInit();
    void handleEvent(sofa::core::objectmodel::Event *event);
    void swap();
    bool m_swaped=false;
    bool m_t1=false;
    bool m_t2=false;

};

} // namespace conect
} // namespace sofa

