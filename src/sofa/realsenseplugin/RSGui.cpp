
#include "RSGui.h"
#include <sofa/gui/qt/GenericWidget.h>
#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <QLabel>
#include <sofa/core/objectmodel/BaseData.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/MouseEvent.h>
#include <sofa/helper/Factory.h>
#include <sofa/gui/qt/SimpleDataWidget.h>

namespace sofa {

namespace realsenseplugin {

sofa::helper::Creator< sofa::gui::qt::DataWidgetFactory, GenericDataWidget< Data<RealSenseDataFrame> , QRGBDWidget > > DWClass_graphUCRealSenseDataFrame("RealSenseDataFrame",true);

}

}
