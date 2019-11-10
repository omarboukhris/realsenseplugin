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

#include <sofa/realsenseplugin/projector/RealSenseAbstractProjector.h>

namespace sofa {

namespace rgbdtracking {

/*!
 * \brief The RealSenseDeprojector class
 * Online / Offline 2D-3D deprojection
 */
class RealSenseDeprojector : public RealSenseAbstractDeprojector
{

public:
    typedef RealSenseAbstractDeprojector Inherited;
    SOFA_CLASS( RealSenseDeprojector , Inherited);

    Data<opencvplugin::ImageData> d_color ;
    // path to save snapshots to
    Data<std::string> d_snap_path ;

    DataCallback c_image ;

    pcl::PointCloud<pcl::Normal>::Ptr m_cloud_normals ;
    helper::vector<defaulttype::Vector3> m_colors ;

    RealSenseDeprojector()
        : Inherited()
        , d_color(initData(&d_color, "color", "segmented color data image"))
        //offline reco
        , d_snap_path(initData(&d_snap_path, std::string("."), "snap_path", "path to snap shots folder"))
        , m_cloud_normals(new pcl::PointCloud<pcl::Normal>)
    {
        c_image.addInputs({&d_color, &this->d_depth});
        c_image.addCallback(std::bind(&RealSenseDeprojector::deproject_image, this));
    }

    virtual ~RealSenseDeprojector () {
    }

    void draw(const core::visual::VisualParams* vparams) {
        if (!d_drawpcl.getValue()) {
        // don't draw point cloud
            return ;
        }

        for (const auto & pt : *(d_outpcl.getValue().getPointCloud())) {
            defaulttype::Vector3 point = defaulttype::Vector3(pt.x, pt.y, pt.z) ;
            vparams->drawTool()->drawPoint(
                point, sofa::defaulttype::Vector4 (0, 0, 255, 0)
            );
        }
    }

    void handleEvent(sofa::core::objectmodel::Event *event) {
        if (sofa::core::objectmodel::KeypressedEvent* ev = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event)) {
            //std::cout << ev->getKey() << std::endl ;

            // screen shot : ctrl+shift+P
            if ((int)ev->getKey() == 'P') {
                exportSnapShot () ;
            }
        }
    }
private :

    void exportSnapShot () {
        static size_t i = 0 ; //< i is a snap shot 'id'
        // set filenames
        std::string
            snapshot_color_filename =
                d_snap_path.getValue() +
                "/rgb_snap_" + std::to_string(i) + ".png",
            snapshot_depth_filename =
                d_snap_path.getValue() +
                "/depth_snap_" + std::to_string(i) + ".png",
            snapshot_dist_filename =
                d_snap_path.getValue() +
                "/dist_snap_" + std::to_string(i) + ".dist" ;
        i++ ;

        // get images to export
        cv::Mat colormat = d_color.getValue().getImage(),
                depthmat = d_depth.getValue().getImage() ;
        // export pngs
        cv::imwrite(snapshot_color_filename.c_str(), colormat) ;
        std::cout << "(RealSenseCam) exported " << snapshot_color_filename << std::endl ;
        cv::imwrite(snapshot_depth_filename.c_str(), depthmat) ;
        std::cout << "(RealSenseCam) exported " << snapshot_depth_filename << std::endl ;

        // export dist frames
        this->_write_distFrame(snapshot_dist_filename);
        std::cout << "(RealSenseCam) exported " << snapshot_dist_filename << std::endl ;
    }

    void _write_distFrame (std::string snapshot_dist_filename) {
        RealSenseDistFrame distframe = d_distframe.getValue() ;
        RealSenseDistFrame::RealSenseDistStruct diststruct = distframe.getFrame();

        std::FILE* filestream = std::fopen(snapshot_dist_filename.c_str(), "wb") ;
        if (filestream == nullptr) {
            std::cerr << "(RealSenseDeprojector) check writing rights on file : "
                     << snapshot_dist_filename
                     << std::endl ;
            return ;
        }
        // write width and height
        std::fwrite(&diststruct._width, sizeof(size_t), 1, filestream) ;
        std::fwrite(&diststruct._height, sizeof(size_t), 1, filestream) ;

        // write frame data
        std::fwrite (
            diststruct.frame,
            sizeof(float),
            diststruct._width * diststruct._height,
            filestream
        ) ;
        std::fclose(filestream) ;
    }

    virtual void writeOfflineToOutput (RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear() ;
        m_pointcloud->clear();
        for (size_t i = 0 ; i < diststruct._height ; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > 0) {
                    // deprojection
                    float dist = diststruct.frame[i*diststruct._width+j] ;
                    push_to_pointcloud(outpoints, j, diststruct, downSample, i, dist);
                }
            }
        }
        d_output.endEdit();
    }

    virtual void writeOnlineToOutput (rs2::depth_frame & depth, RealSenseDistFrame::RealSenseDistStruct & diststruct, const cv::Mat & depth_im, int downSample) override {
        helper::vector<defaulttype::Vector3> & outpoints = *d_output.beginEdit() ;
        outpoints.clear() ;
        for (size_t i = 0 ; i < diststruct._height; ++i) {
            for (size_t j = 0 ; j < diststruct._width ; ++j) {
                if (depth_im.at<const uchar>(downSample*i,downSample*j) > 0) {
                    // deprojection
                    float dist = depth.get_distance(downSample*j, downSample*i) ;
                    push_to_pointcloud(outpoints, j, diststruct, downSample, i, dist);
                }
            }
        }
        d_output.endEdit();
    }

    void push_to_pointcloud(helper::vector<defaulttype::Vector3> & outpoints, size_t j, RealSenseDistFrame::RealSenseDistStruct& diststruct, int downSample, size_t i, float dist)
    {
        float
            point3d[3] = {0.f, 0.f, 0.f},
            point2d[2] = {downSample*i, downSample*j};
        rs2_deproject_pixel_to_point(
            point3d,
            &cam_intrinsics,
            point2d,
            dist
        );
        // set dist frame for exportation if needed
        diststruct.frame[i*diststruct._width+j] = dist ;

        // set units // switch comments for alignment
        pcl::PointXYZ pt = pcl::PointXYZ(point3d[1], -point3d[0], -point3d[2]) ;
        m_pointcloud->push_back(pt);

        defaulttype::Vector3 point = defaulttype::Vector3(pt.x, pt.y, pt.z) ;
        outpoints.push_back(point) ;
    }

    void compute_pcl_normals () {
        // compute normals for remeshing ?
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne ;
        ne.setInputCloud(m_pointcloud);
        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.03);

        // Compute the features
        m_cloud_normals->clear();
        ne.compute (*m_cloud_normals);
    }
};

}

}

