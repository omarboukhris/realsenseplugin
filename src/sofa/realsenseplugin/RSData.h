#pragma once


#include <opencv2/highgui.hpp>

#include <opencv2/opencv.hpp>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QApplication>
#include <QtWidgets/QApplication>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QMenuBar>
#include <QTimer>
#include <QLabel>
#include <QMouseEvent>
#include <QSlider>
#include <QGroupBox>
#include <QRadioButton>
#include <sofa/helper/fixed_array.h>
#include <memory>

#include <librealsense2/rs.hpp>
#include <sofa/realsenseplugin/cv-helpers.hpp>

#include <sofa/core/objectmodel/Data.h>
#include <sofa/gui/qt/DataWidget.h>

namespace sofa {

namespace realsenseplugin {

using namespace core::objectmodel ;

class rgbd_frame {
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
        c.copyTo(color);
        d.copyTo(depth);
        pointcloud = ptcld ;
    }
    rgbd_frame(const rgbd_frame & f) {
        f.color.copyTo(color);
        f.depth.copyTo(depth);
        pointcloud = f.pointcloud ;
    }
} ;

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

class QRGBDWidget : public QLabel {
    Q_OBJECT
public:

    typedef core::objectmodel::Data<RealSenseDataFrame> MyData;

    QRGBDWidget(sofa::gui::qt::DataWidget * parent,const MyData& /*data*/) : QLabel(parent) {
        QLabel::setScaledContents(true);
        setSizePolicy( QSizePolicy::MinimumExpanding,QSizePolicy::Expanding);
    }

    void readFromData(const MyData& data) {
        const cv::Mat data_img ;
        cv::hconcat(
            data.getValue().getcvColor(),
            data.getValue().getcvDepth(),
            data_img
        );

        if (data_img.cols*data_img.rows == 0) return;

        QImage::Format format;
        if (data_img.channels() == 1) format = QImage::Format_Grayscale8;
        else if (data_img.channels() == 3) format = QImage::Format_RGB888;
        else if (data_img.channels() == 4) format = QImage::Format_RGBA8888;
        else format = QImage::Format_ARGB32;

        cv::Mat img;
        data_img.copyTo(img); // need a copy because the pixam modify the data

        QPixmap p;
        p.convertFromImage(QImage (img.data,
                                img.cols,
                                img.rows,
                                format).rgbSwapped());

        setPixmap(p);
    }

    void writeToData(MyData& /*d*/) {}

};

} // namespace realsenseplugin

namespace defaulttype {

template<class TDataType>
struct RGBDDataTypeInfo
{
    typedef TDataType DataType;
    typedef typename DataType::T BaseType;
    typedef DataTypeInfo<BaseType> BaseTypeInfo;
    typedef typename BaseTypeInfo::ValueType ValueType;
    typedef DataTypeInfo<ValueType> ValueTypeInfo;

    enum { ValidInfo       = BaseTypeInfo::ValidInfo       }; ///< 1 if this type has valid infos
    enum { FixedSize       = 1                             }; ///< 1 if this type has a fixed size  -> always 1 Image
    enum { ZeroConstructor = 0                             }; ///< 1 if the constructor is equivalent to setting memory to 0  -> I guess so, a default Image is initialzed with nothing
    enum { SimpleCopy      = 0                             }; ///< 1 if copying the data can be done with a memcpy
    enum { SimpleLayout    = 0                             }; ///< 1 if the layout in memory is simply N values of the same base type
    enum { Integer         = 0                             }; ///< 1 if this type uses integer values
    enum { Scalar          = 0                             }; ///< 1 if this type uses scalar values
    enum { Text            = 0                             }; ///< 1 if this type uses text values
    enum { CopyOnWrite     = 1                             }; ///< 1 if this type uses copy-on-write -> it seems to be THE important option not to perform too many copies
    enum { Container       = 0                             }; ///< 1 if this type is a container

    enum { Size = 1 }; ///< largest known fixed size for this type, as returned by size()

    static size_t size() { return 1; }
    static size_t byteSize() { return 1; }

    static size_t size(const DataType& /*data*/) { return 1; }

    static bool setSize(DataType& /*data*/, size_t /*size*/) { return false; }

    template <typename T>
    static void getValue(const DataType &/*data*/, size_t /*index*/, T& /*value*/)
    {
        return;
    }

    template<typename T>
    static void setValue(DataType &/*data*/, size_t /*index*/, const T& /*value*/ )
    {
        return;
    }

    static void getValueString(const DataType &data, size_t index, std::string& value)
    {
        if (index != 0) return;
        std::ostringstream o; o << data; value = o.str();
    }

    static void setValueString(DataType &data, size_t index, const std::string& value )
    {
        if (index != 0) return;
        std::istringstream i(value); i >> data;
    }

    static const void* getValuePtr(const DataType&)
    {
        return nullptr;
    }

    static void* getValuePtr(DataType&)
    {
        return nullptr;
    }
};

//template<>
//struct DataTypeInfo< realsenseplugin::RealSenseDataFrame > : public RGBDDataTypeInfo< realsenseplugin::RealSenseDataFrame >
//{
//    static std::string name() { std::ostringstream o; o << "RealSenseDataFrame<>"; return o.str(); }
//};

}  // namespace defaulttype


} // namespace sofa

