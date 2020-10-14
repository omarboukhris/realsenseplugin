#pragma once

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/DataCallback.h>

#include <sofa/realsenseplugin/RSData.h>

#include <sofa/opencvplugin/BaseOpenCVData.h>

namespace sofa {

namespace realsenseplugin {

using namespace core::objectmodel ;

/*!
 * \class ImageData2RealSenseDataFrame
 * \brief Component for contour detection
 */
class ImageData2RealSenseDataFrame : public sofa::core::objectmodel::BaseObject {

public:
	
    SOFA_CLASS(ImageData2RealSenseDataFrame, sofa::core::objectmodel::BaseObject) ;
	
    Data<opencvplugin::ImageData> d_image;
    Data<RealSenseDataFrame> d_out_image;

    DataCallback c_callback;

	/*!
     * \fn ImageData2RealSenseDataFrame ()
	 * \brief Component constructor
	 */
    ImageData2RealSenseDataFrame()
    : d_image(initData(&d_image, "image", "input image"))
    , d_out_image(initData(&d_out_image, "mask", "output mask image")) {
        c_callback.addInputs({&d_image});
        c_callback.addCallback(std::bind(&ImageData2RealSenseDataFrame::update,this));
	}

    /*!
     * \fn void update ()
     * \brief find the contour in the loaded image
     */
    void update() {
        cv::Mat image = d_image.getValue().getImage() ;
        d_out_image.setValue(RealSenseDataFrame (image, image));
    }

};


}

}
