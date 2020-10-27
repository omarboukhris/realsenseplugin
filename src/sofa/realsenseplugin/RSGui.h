#pragma once

#include <sofa/gui/qt/DataWidget.h>
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

#include <sofa/realsenseplugin/RSData.h>

namespace sofa {

namespace realsenseplugin {

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

} // namespace sofa

