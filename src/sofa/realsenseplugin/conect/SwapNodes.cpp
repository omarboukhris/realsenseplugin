#include "SwapNodes.inl"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace conect
{

using namespace sofa::defaulttype;

SOFA_DECL_CLASS(SwapNodes)

int SwapNodesClass = core::RegisterObject("Solver to test compliance computation for new articulated system objects")
.add< SwapNodes >();


} // namespace conect

} // namespace sofa
