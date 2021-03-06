#pragma once

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/defaulttype/DataTypeInfo.h>

#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <memory>

#include <librealsense2/rs.hpp>

#include <sofa/helper/fixed_array.h>
#include <sofa/realsenseplugin/cv-helpers.hpp>

#include <CImgPlugin/CImgData.h>

namespace sofa {

namespace realsenseplugin {

using namespace core::objectmodel ;

typedef struct rgbd_frame {
public :
    typedef helper::vector<defaulttype::Vector3> sofaPointCloud ;
    cv::Mat color ;
    cv::Mat depth ;
    sofaPointCloud pointcloud ;

    rgbd_frame() {
        color = cv::Mat() ;
        depth = cv::Mat() ;
        pointcloud = sofaPointCloud () ;
    }
    rgbd_frame (cv::Mat c, cv::Mat d, sofaPointCloud ptcld=sofaPointCloud()) {
        c.assignTo(color);
        d.assignTo(depth);
        pointcloud = ptcld ;
    }
    rgbd_frame(const rgbd_frame & f) {
        f.color.assignTo(color);
        f.depth.assignTo(depth);
        pointcloud = f.pointcloud ;
    }
} rgbd_frame ;

class RealSenseDataFrame {
public:

    friend class QRGBDWidget;
    typedef unsigned char T;

    RealSenseDataFrame() {}
    RealSenseDataFrame(cv::Mat color, cv::Mat depth, rgbd_frame::sofaPointCloud p=rgbd_frame::sofaPointCloud()) {
       m_frame = rgbd_frame(color, depth,p) ;
    }
    RealSenseDataFrame(rgbd_frame frame) {
        m_frame = rgbd_frame (frame) ;
    }

    RealSenseDataFrame(const RealSenseDataFrame & df) {
        m_frame = rgbd_frame(df.m_frame) ;
    }

    rgbd_frame & getRGBD () {
        return m_frame ;
    }
    cv::Mat& getcvColor() {
        return m_frame.color ;
    }
    cv::Mat& getcvDepth() {
        return m_frame.depth;
    }
    rgbd_frame::sofaPointCloud& getsofapointcloud() {
        return m_frame.pointcloud;
    }

    operator rgbd_frame &() { return m_frame; }

    const rgbd_frame & getRGBD () const {
        return m_frame ;
    }
    const cv::Mat& getcvColor() const {
        return m_frame.color ;
    }
    const cv::Mat& getcvDepth() const {
        return m_frame.depth;
    }
    const rgbd_frame::sofaPointCloud& getsofapointcloud() const {
        return m_frame.pointcloud;
    }

    operator const rgbd_frame &() const { return m_frame; }

    unsigned colorwidth() const { return getcvColor().cols; }
    unsigned depthwidth() const { return getcvDepth().cols; }

    unsigned colorheight() const { return getcvColor().rows; }
    unsigned depthheight() const { return getcvDepth().rows; }

    friend std::istream& operator>>(std::istream& in, RealSenseDataFrame&) { return in; }

    friend std::ostream& operator<<(std::ostream& out, const RealSenseDataFrame&) { return out; }

    friend std::ifstream& operator>>(std::ifstream& in, RealSenseDataFrame& df_out) {
        return in;
    }
    friend std::ofstream& operator<<(std::ofstream& out, const RealSenseDataFrame& df) {
        return out;
    }

protected:
    rgbd_frame m_frame ;
};

} // namespace realsenseplugin

namespace defaulttype {

template<>
struct DataTypeInfo< realsenseplugin::RealSenseDataFrame > : public ImageTypeInfo< realsenseplugin::RealSenseDataFrame >
{
    static std::string name() { std::ostringstream o; o << "RealSenseDataFrame<>"; return o.str(); }
};

}  // namespace defaulttype


} // namespace sofa

