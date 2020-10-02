#include "MaskFromContour.h"

namespace sofa {

namespace opencvplugin {

namespace tracker {

SOFA_DECL_CLASS(MaskFromContour)

int MaskFromContourClass = core::RegisterObject("MaskFromContour : generates binarized image mask from list of input 2D points forming a contour")
.add<MaskFromContour>();

}

}

}
