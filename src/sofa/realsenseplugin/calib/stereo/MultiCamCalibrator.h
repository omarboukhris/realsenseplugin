/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 beta 4      *
*                (c) 2006-2009 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

//#include <opencv2/ccalib/randpattern.hpp>

#include <sofa/defaulttype/Vec.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataFileName.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/BoundingBox.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/objectmodel/Event.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/Quat.h>
#include <sofa/helper/rmath.h>
#include <sofa/helper/OptionsGroup.h>
#include <sofa/helper/system/FileSystem.h>

#include <librealsense2/rs.hpp>

#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/core/types_c.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

#include <fstream>
#include <algorithm>
#include <iostream>
#include <string>
#include <map>

#include <sofa/realsenseplugin/cv-helpers.hpp>

#include <exception>

namespace sofa
{

namespace realsenseplugin
{

static void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners)
{
    corners.resize(0);

    for( int i = 0; i < boardSize.height; i++ )
        for( int j = 0; j < boardSize.width; j++ )
            corners.push_back(cv::Point3f(float(j*squareSize),
                                      float(i*squareSize), 0));
}

/*!
 * \brief The MultiCamCalibrator class
 * Utility component for calibrating offline a multicam setup
 */
class MultiCamCalibrator : public core::objectmodel::BaseObject
{
public :

    typedef core::objectmodel::BaseObject Inherited;
    SOFA_CLASS( MultiCamCalibrator, Inherited);

    /// \brief path for calibration images from camera 1
    Data<std::string> d_calibcam1 ;
    /// \brief path for calibration images from camera 2
    Data<std::string> d_calibcam2 ;
    /// \brief chessboard size, usual chessboard is 7x7
    Data<defaulttype::Vector2> d_chessboardsize ;
    core::objectmodel::DataCallback callback ;

    /// \brief output rotation matrix
    Data<defaulttype::Mat3x3> d_rotation ;
    /// \brief output translation vector
    Data<defaulttype::Vector3> d_translation ;

    /// \brief set of calibration images from camera 1
    std::vector<cv::Mat> calibimage1;
    /// \brief set of calibration images from camera 2
    std::vector<cv::Mat> calibimage2;

    /// \brief computed during calibration
    std::vector<std::vector<cv::Point3f> > obj_points ;
    /// \brief computed during calibration, image points from camera 1
    std::vector<std::vector<cv::Point2f> > img_points1 ;
    /// \brief computed during calibration, image points from camera 2
    std::vector<std::vector<cv::Point2f> > img_points2 ;

    MultiCamCalibrator()
        : Inherited()
        , d_calibcam1(initData(&d_calibcam1, std::string("./"), "calibcam1", "path to folder with calibration images from camera 1"))
        , d_calibcam2(initData(&d_calibcam2, std::string("./"), "calibcam2", "path to folder with calibration images from camera 2"))
        , d_chessboardsize(initData(&d_chessboardsize, defaulttype::Vector2(6,9), "size", "dimensions of the chessboard"))
        //output
        , d_rotation(
            initData(
                &d_rotation,
                defaulttype::Mat3x3(
                    defaulttype::Vector3(1,0,0),
                    defaulttype::Vector3(0,1,0),
                    defaulttype::Vector3(0,0,1)),
                "rotation",
                "rotation matrix for rotating/translating pointcloud"
            )
        ), d_translation(initData(&d_translation, defaulttype::Vector3(0,0,0), "translation", "computed translation vector"))
    {
        callback.addInputs({&d_calibcam1, &d_calibcam2}) ;
        callback.addCallback(std::bind(&MultiCamCalibrator::load_all, this)) ;
        this->f_listening.setValue(true);
    }

    /*!
     * \brief loadImages loads calibration images from specified folder
     * \param path to directory containing images
     * \param output vector of cv::Mat containing list of loaded images
     * \return number of loaded images
     */
    int loadImages (const std::string & path, std::vector<cv::Mat> & output) {
        output.clear() ;
        if (helper::system::FileSystem::isDirectory(path)) {
            std::vector<std::string> png_images ;
            helper::system::FileSystem::listDirectory(path, png_images, std::string(".png")) ;
            std::sort(png_images.begin(), png_images.end()) ;
            for (auto p : png_images) {
                cv::Mat image = cv::imread(path + "/" + p), grey ;
                cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY) ;
                output.push_back(grey);
            }
            return output.size() ;
        }
        return 0 ;
    }

    /*!
     * \brief getCorners computes corners from stereo images
     * \param boardsize chessboard size
     * \param img1 cv::Mat from camera 1
     * \param corners1 output corners for 1st frame
     * \param img2 cv::Mat from camera 2
     * \param corners2 output corners for 2nd frame
     * \return true if corners found on both images, else false
     */
    bool getCorners(
        const cv::Size boardsize,
            cv::Mat img1, std::vector<cv::Point2f> & corners1,
            cv::Mat img2, std::vector<cv::Point2f> & corners2)
    {
        bool found1 = cv::findChessboardCorners(img1, boardsize, corners1, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS) ;
        bool found2 = cv::findChessboardCorners(img2, boardsize, corners2, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS) ;
        if (found1 && found2) {
            cv::cornerSubPix(
                img1, corners1,
                cv::Size(5,5), cv::Size(-1,-1),
                cv::TermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 42, 1e-2)) ;
            cv::cornerSubPix(
                img2, corners2,
                cv::Size(5,5), cv::Size(-1,-1),
                cv::TermCriteria(CV_TERMCRIT_EPS|CV_TERMCRIT_ITER, 42, 1e-2)) ;
            img_points1.push_back(corners1);
            img_points2.push_back(corners2);
            add_obj_points();
            return true ;
        }
        return false ;
    }

    /*!
     * \brief get_image_points computes images points from list of stereo calibration frames
     * \param boardsize chessboard size
     * \param imglist1 camera 1's calibration frames
     * \param imglist2 camera 2's calibration frames
     */
    void get_image_points (
        const cv::Size boardsize,
        std::vector<cv::Mat> & imglist1,
        std::vector<cv::Mat> & imglist2
    ) {
        obj_points.clear();
        img_points1.clear();
        img_points2.clear();
        for (int i = 0 ; i < imglist1.size() ; i++) {
            cv::Mat img1 = imglist1[i], img2 = imglist2[i] ;
            std::vector<cv::Point2f> corners1, corners2 ;
            getCorners(boardsize, img1, corners1, img2, corners2);
        }
    }

    /*!
     * \brief add_obj_points adds static object points for stereo calibration
     */
    void add_obj_points()
    {
        std::vector<cv::Point3f> corners ;
        corners.clear();
        calcChessboardCorners(
            cv::Size(
                d_chessboardsize.getValue()[0],
                d_chessboardsize.getValue()[1]),
            1.f,
            corners
        );
        obj_points.push_back(corners) ;
    }

    /*!
     * \brief calibratemono intrinsic calibration of one camera
     * \param CM output camera matrix
     * \param D output distortion matrix
     * \param imagePoints image points used for calibration
     * \param imgsize images' size
     */
    void calibratemono(cv::Mat & CM, cv::Mat & D, const std::vector<std::vector<cv::Point2f> > & imagePoints, const cv::Size & imgsize)
    {
        std::vector<cv::Mat> rvecs, tvecs;
        CM = cv::Mat(3, 3, CV_32FC1);
        CM.at<float>(0, 0) = 1;
        CM.at<float>(1, 1) = 1;
        cv::calibrateCamera(obj_points, imagePoints, imgsize, CM, D, rvecs, tvecs);
    }

    /*!
     * \brief write2output writes Translation and rotation matrix to sofa data
     * \param T translation vector
     * \param R Rotation matrix
     */
    void write2output(cv::Mat T, cv::Mat R)
    {
        defaulttype::Mat3x3 & rotmat = *d_rotation.beginEdit() ;
        for (int i = 0 ; i < 3 ; i++){
            for (int j = 0 ; j < 3 ; j++) {
                rotmat[i][j] = R.at<float>(i, j) ;
            }
        }
        defaulttype::Vector3 & translation = *d_translation.beginEdit() ;
        for (int i = 0 ; i < 3 ; i++)
            translation[i] = T.at<double>(i) ;
        d_translation.endEdit();
        d_rotation.endEdit();
    }

    /*!
     * \brief checkImgPointsValid simple guard for esthetics
     * \return true if images points's sizes correspont on left and right (image points)
     */
    inline bool checkImgPointsValid() {
        //img_points and calibimages should be aligned in size
        return (img_points1.size() * img_points2.size() == 0) ;
    }

    /*!
     * \brief load_all
     * loads stereo images and computes images points (results are stored into class members)
     */
    void load_all()
    {
        img_points1.clear() ; img_points2.clear();
        int i1 = loadImages(d_calibcam1.getValue(),calibimage1) ;
        int i2 = loadImages(d_calibcam2.getValue(),calibimage2) ;
        if (i1 != i2 || i1 * i2 == 0) {
            std::cerr << "(MultiCamCalibrator) numbers of images is different, check input folders" << std::endl ;
            return ;
        }
        std::cout << "loading done : got " << calibimage1.size() << " images" << std::endl ;

        defaulttype::Vector2 bsize = d_chessboardsize.getValue() ;
        cv::Size boardsize = cv::Size(bsize[0],bsize[1]) ;
        get_image_points(boardsize, calibimage1, calibimage2);
        std::cout << "got img points : " << img_points1.size() << std::endl ;
    }

    /*!
     * \brief process
     * applies mono then stereo calibration on both cameras
     */
    void process () {
//        load images
        if (checkImgPointsValid()) {
            std::cerr << "(MultiCamCalibrator) Error computing image points" << std::endl ;
            return ;
        }
        std::cout << "(MultiCamCalibrator) Valid image points" << std::endl ;

//        call camera calibration then
        std::cout << "(MultiCamCalibrator) Intrinsic calibration" << std::endl ;
        cv::Mat cm1, d1, cm2, d2 ;
        cv::Size imgsize = calibimage1[0].size() ;
        calibratemono(cm1, d1, img_points1, imgsize);
        calibratemono(cm2, d2, img_points2, imgsize);
        std::cout << "(MultiCamCalibrator) Intrinsic calibration done" << cm1 << cm2 << d1 << d2<< std::endl ;

//        call cv::stereoCalibrate()
//        output is extrinsic R|T matrix
        std::cout << "(MultiCamCalibrator) Extrinsic calibration" << std::endl ;
        cv::Mat R, T, E, F;
        cv::stereoCalibrate(
            obj_points,
            img_points1, img_points2,
            cm1, d1,
            cm2, d2,
            imgsize,
            R, T, E, F,
            cv::CALIB_SAME_FOCAL_LENGTH | cv::CALIB_ZERO_TANGENT_DIST,
            cv::TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 100, 1e-5));
        std::cout << "(MultiCamCalibrator) Done calibrating : " << R << " " << T << std::endl ;

        write2output(T, R);
    }

    /*!
     * \brief handleEvent press C to launch calibration
     * \param event
     */
    void handleEvent(sofa::core::objectmodel::Event* event) {
        if (core::objectmodel::KeypressedEvent* ev = dynamic_cast<core::objectmodel::KeypressedEvent*>(event)) {
            if (ev->getKey() == 'c'||ev->getKey() == 'C') {
                load_all() ;
                process();
            }
        }
    }

};

}

}

