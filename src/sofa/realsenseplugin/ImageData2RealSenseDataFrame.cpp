#include "ImageData2RealSenseDataFrame.h"

namespace sofa {

namespace realsenseplugin {

SOFA_DECL_CLASS(ImageData2RealSenseDataFrame)

int ImageData2RealSenseDataFrameClass = core::RegisterObject("ImageData2RealSenseDataFrame : generates binarized image mask from list of input 2D points forming a contour")
.add<ImageData2RealSenseDataFrame>();

}

}

