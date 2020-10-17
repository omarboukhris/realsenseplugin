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
 * \brief make rs data frame from image data
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
    , d_out_image(initData(&d_out_image, "rsframe", "output mask image")) {
        c_callback.addInputs({&d_image});
        c_callback.addCallback(std::bind(&ImageData2RealSenseDataFrame::update,this));
	}

    /*!
     * \fn void update ()
     * \brief set image as rs data frame
     */
    void update() {
        const cv::Mat & image = d_image.getValue().getImage() ;
        d_out_image.setValue(RealSenseDataFrame (image, image));
    }

};


/*!
 * \class RealSenseDataFrame2ImageData
 * \brief make imagedata from rs data frame
 */
class RealSenseDataFrame2ImageData : public sofa::core::objectmodel::BaseObject {

public:

    SOFA_CLASS(RealSenseDataFrame2ImageData, sofa::core::objectmodel::BaseObject) ;

    Data<RealSenseDataFrame> d_image;
    Data<opencvplugin::ImageData> d_out_image;

    DataCallback c_callback;

    /*!
     * \fn RealSenseDataFrame2ImageData()
     * \brief Component constructor
     */
    RealSenseDataFrame2ImageData()
    : d_image(initData(&d_image, "rsframe", "input rgbd image"))
    , d_out_image(initData(&d_out_image, "image", "output image")) {
        c_callback.addInputs({&d_image});
        c_callback.addCallback(std::bind(&RealSenseDataFrame2ImageData::update,this));
    }

    /*!
     * \fn void update ()
     * \brief set color image to proper data channel
     */
    void update() {
        const cv::Mat & image = d_image.getValue().getcvColor() ;
        d_out_image.setValue(opencvplugin::ImageData(image));
    }

};


}

}
