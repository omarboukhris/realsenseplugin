#pragma once

#include <sofa/core/ObjectFactory.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/DataCallback.h>

#include <sofa/realsenseplugin/RSData.h>

namespace sofa {

namespace realsenseplugin {

using namespace core::objectmodel ;

/*!
 * \class MaskFromContour
 * \brief Component for contour detection
 */
class MaskFromContour : public sofa::core::objectmodel::BaseObject {

public:
	
    SOFA_CLASS(MaskFromContour, sofa::core::objectmodel::BaseObject) ;
	
    Data<helper::vector<defaulttype::Vector2> > d_contour;
    Data<RealSenseDataFrame> d_image;
    Data<RealSenseDataFrame> d_out_image;

    DataCallback c_callback;

	/*!
     * \fn MaskFromContour ()
	 * \brief Component constructor
	 */
    MaskFromContour()
    : d_contour( initData(&d_contour, "contour", "this"))
    , d_image(initData(&d_image, "image", "input image"))
    , d_out_image(initData(&d_out_image, "mask", "output mask image")) {
        c_callback.addInputs({&d_image,&d_contour});
        c_callback.addCallback(std::bind(&MaskFromContour::update,this));
	}

    /*!
     * \fn void update ()
     * \brief find the contour in the loaded image
     */
    void update() {
        cv::Mat image ;
        d_image.getValue().getcvColor().copyTo(image); ;
        if (image.empty()) return;

        image = cv::Mat::zeros(image.rows, image.cols, CV_8UC3) ;

        helper::vector<defaulttype::Vector2> pts = d_contour.getValue() ;
        if (pts.size() <= 2) {
            return ;
        }
        std::vector<std::vector<cv::Point> > cvpts (1) ;
        for (const auto & pt : pts) {
            cvpts.back().push_back(cv::Point(pt[0], pt[1]));
        }

        cv::fillPoly(image, cvpts, cv::Scalar(255, 255, 255)) ;
        d_out_image.setValue(RealSenseDataFrame (image, image));
    }

};


}

}
