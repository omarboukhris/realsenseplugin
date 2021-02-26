#pragma once

#include "SwapNodes.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/helper/cast.h>

namespace sofa
{
namespace conect
{

SwapNodes::SwapNodes()
: d_t1( initData(&d_t1, (double) -1.0,  "t1", "Swapping time 1") )
, d_t2( initData(&d_t2, (double) -1.0,  "t2", "Swapping time 2") )
, d_nodesD( initData(&d_nodesD, "nodes1", "link to node 1") )
, d_nodesA( initData(&d_nodesA, "nodes2", "link to node 2") )
, d_onDemand(initData(&d_onDemand,false,"onDemand","Swap when this bool is at 1") )
, d_noinit(initData(&d_noinit,false,"noinit","activates the bwdInit if false") )
{
    this->f_listening.setValue(true);
}

void SwapNodes::bwdInit() {
    // this is to (de)activate backward init if needed when scene loads
    // default leave it active
    if (d_noinit.getValue()) return ;
    for (unsigned i=0;i<d_nodesD.getValue().size();i++) {
        sofa::simulation::Node* rootcontext = down_cast<sofa::simulation::Node>(getContext()->getRootContext());
        simulation::Node* node1 = down_cast<sofa::simulation::Node>(rootcontext->getTreeNode(d_nodesD.getValue()[i]));
        if (node1) node1->setActive(true);
        else std::cout << "Cannot find " << d_nodesD.getValue()[i] << std::endl;
    }

    for (unsigned i=0;i<d_nodesA.getValue().size();i++) {
        sofa::simulation::Node* rootcontext = down_cast<sofa::simulation::Node>(getContext()->getRootContext());
        simulation::Node* node1 = down_cast<sofa::simulation::Node>(rootcontext->getTreeNode(d_nodesA.getValue()[i]));
        if (node1) node1->setActive(false);
        else std::cout << "Cannot find " << d_nodesA.getValue()[i] << std::endl;
    }
}

void SwapNodes::swap()
{
    for (unsigned i=0;i<d_nodesD.getValue().size();i++) {
        sofa::simulation::Node* rootcontext = down_cast<sofa::simulation::Node>(getContext()->getRootContext());
        simulation::Node* node1 = down_cast<sofa::simulation::Node>(rootcontext->getTreeNode(d_nodesD.getValue()[i]));
        if (node1)
        {
            node1->setActive(!(node1->isActive()));
            if(node1->isActive())
                node1->init(core::ExecParams::defaultInstance());
        }
    }

    for (unsigned i=0;i<d_nodesA.getValue().size();i++) {
        sofa::simulation::Node* rootcontext = down_cast<sofa::simulation::Node>(getContext()->getRootContext());
        simulation::Node* node1 = down_cast<sofa::simulation::Node>(rootcontext->getTreeNode(d_nodesA.getValue()[i]));
        if (node1)
        {
            node1->setActive(!(node1->isActive()));
            if(node1->isActive())
                node1->init(core::ExecParams::defaultInstance());
        }
    }

}

void SwapNodes::handleEvent(sofa::core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent*>(event)) {
        if (d_t1.getValue() >= 0.0) {
            if (((this->getContext()->getTime()-d_t1.getValue())>=0)&&(!m_t1)) {
                swap();
                m_t1 = true;
            }
        }

        if (d_t2.getValue() >= 0.0) {
            if (((this->getContext()->getTime()-d_t2.getValue())>=0)&&(!m_t2)) {
                swap();
                m_t2 = true;
            }
        }

        if(d_onDemand.getValue()&&(!m_swaped))
        {
           swap();
           m_swaped = true;
        }
    }

    if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)) {
        if (ev->getKey() == 'm' ||ev->getKey() == 'M') {
            swap() ;
        }
    }


}

} // namespace conect

} // namespace sofa
