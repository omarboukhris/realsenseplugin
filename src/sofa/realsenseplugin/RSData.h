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

class RealSenseDataFrame {
public:
    typedef struct {
        rs2::video_frame *color ;
        rs2::depth_frame *depth ;
    } RealSenseFrame ;    

    friend class QRGBDWidget;
    typedef unsigned char T;

    RealSenseDataFrame() {}
    RealSenseDataFrame(RealSenseFrame rsframe) {
        m_frame.color = new rs2::video_frame(*(rsframe.color)) ;
        m_frame.depth = new rs2::depth_frame(*(rsframe.depth)) ;
        this->color = getcvColor();
        this->depth = getcvDepth();
    }

    RealSenseDataFrame(rs2::video_frame color, rs2::depth_frame depth) {
        m_frame.color = new rs2::video_frame(color) ;
        m_frame.depth = new rs2::depth_frame(depth) ;
        this->color = getcvColor();
        this->depth = getcvDepth();
    }
    RealSenseDataFrame(cv::Mat color, cv::Mat depth) {
        this->color = color ;
        this->depth = depth ;
    }

    RealSenseFrame & getRGBD () {
        return m_frame ;
    }
    cv::Mat& getcvColor() {
        color = frame_to_mat(*(m_frame.color));
        return color ;
    }
    cv::Mat& getcvDepth(bool colorize=false, int depthscale=10) {
        if (colorize) {
            rizer.set_option(RS2_OPTION_COLOR_SCHEME, 2);
            rs2::frame bw_depth = m_frame.depth->apply_filter(rizer) ;
            depth = frame_to_mat(bw_depth) ;
        } else {
            int widthd = m_frame.depth->get_width();
            int heightd = m_frame.depth->get_height();
            cv::Mat depth16 = cv::Mat(heightd, widthd, CV_16U, (void*)m_frame.depth->get_data()) ;
            depth16.convertTo(depth, CV_8U, 1.f/64*depthscale); //depth32 is output
        }
        return depth;
    }

    operator RealSenseFrame &() { return m_frame; }

    const RealSenseFrame & getRGBD () const {
        return m_frame ;
    }
    const cv::Mat& getcvColor() const {
        return color ;
    }
    const cv::Mat& getcvDepth() const {
        return depth;
    }

    operator const RealSenseFrame &() const { return m_frame; }

    unsigned colorwidth() const { return getcvColor().cols; }
    unsigned depthwidth() const { return getcvDepth().cols; }

    unsigned colorheight() const { return getcvColor().rows; }
    unsigned depthheight() const { return getcvDepth().rows; }

    friend std::istream& operator>>(std::istream& in, RealSenseDataFrame&) { return in; }

    friend std::ostream& operator<<(std::ostream& out, const RealSenseDataFrame&) { return out; }

    friend std::ifstream& operator>>(std::ifstream& in, RealSenseDataFrame& df_out) {
        if (!in) {
            return in ; // stream unopened
        }
        int h, h_d, stride, stride_d ;
        rs2_frame * _color ; rs2_frame * _depth ;
        // same order as writing : first heights
        in.read((char*) &h, sizeof(int)) ;
        in.read((char*) &h_d, sizeof(int)) ;
        // then strides
        in.read((char*) &stride, sizeof(int)) ;
        in.read((char*) &stride_d, sizeof(int)) ;
        // then actual data
        in.read((char*)_color, h*stride) ;
        in.read((char*)_depth, h_d*stride_d) ;
        df_out.m_frame.color = new rs2::video_frame(rs2::frame(_color)) ;
        df_out.m_frame.depth = new rs2::depth_frame(rs2::frame(_depth)) ;
        return in;
    }
    friend std::ofstream& operator<<(std::ofstream& out, const RealSenseDataFrame& df) {
        if (!out) {
            return out ; // stream is unopened
        }
        // color data
        int h = df.m_frame.color->get_height(),
            stride = df.m_frame.color->get_stride_in_bytes() ;
        // depth data
        int h_d = df.m_frame.depth->get_height(),
            stride_d = df.m_frame.depth->get_stride_in_bytes() ;
        // write  sizes first : height
        out.write(reinterpret_cast<const char*>(&h), sizeof (int)) ;
        out.write(reinterpret_cast<const char*>(&h_d), sizeof (int)) ;
        // strides
        out.write(reinterpret_cast<const char*>(&stride), sizeof (int)) ;
        out.write(reinterpret_cast<const char*>(&stride_d), sizeof (int)) ;
        // actual frame
        out.write(static_cast<const char*>(df.m_frame.color->get_data()), h*stride) ;
        out.write(static_cast<const char*>(df.m_frame.depth->get_data()), h_d*stride_d) ;
        return out;
    }

protected:
    RealSenseFrame m_frame ;
    cv::Mat color, depth ;
    /// \brief colorizer used for colorizing depth frame
    rs2::colorizer rizer ;

};

class QRGBDWidget : public QLabel {
    Q_OBJECT
public:

    typedef Data<RealSenseDataFrame> MyData;

    QRGBDWidget(sofa::gui::qt::DataWidget * parent,const MyData& /*data*/) : QLabel(parent) {
        QLabel::setScaledContents(true);
        setSizePolicy( QSizePolicy::MinimumExpanding,QSizePolicy::Expanding);
    }

    void readFromData(const MyData& data) {
        const cv::Mat & data_img = data.getValue().getcvColor();

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

