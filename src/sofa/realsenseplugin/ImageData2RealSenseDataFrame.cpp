#include "ImageData2RealSenseDataFrame.h"

namespace sofa {

namespace realsenseplugin {

SOFA_DECL_CLASS(ImageData2RealSenseDataFrame)

int ImageData2RealSenseDataFrameClass = core::RegisterObject("ImageData2RealSenseDataFrame : makes rgbd data from image data")
.add<ImageData2RealSenseDataFrame>();

SOFA_DECL_CLASS(RealSenseDataFrame2ImageData)

int RealSenseDataFrame2ImageDataClass = core::RegisterObject("RealSenseDataFrame2ImageData : make image data from rgbd data")
.add<RealSenseDataFrame2ImageData>();

}

}

