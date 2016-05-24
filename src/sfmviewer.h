#ifndef SFMVIEWER_H
#define SFMVIEWER_H


#include <QGLViewer/qglviewer.h>
#include <QFileDialog>
#include <QLineEdit>
#include <QThreadPool>
#include <Eigen/Eigen>
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "common.h"


class SFMViewer : public QGLViewer
{
    Q_OBJECT

    std::vector<cv::Mat> 			images;
    std::vector<std::string> 		images_names;
    std::vector<cv::Point3d> 		m_pcld;
    std::vector<cv::Vec3b> 			m_pcldrgb;
    std::vector<cv::Matx34d> 		m_cameras;
    std::vector<Eigen::Affine3d> 	m_cameras_transforms;
    Eigen::Affine3d 				m_global_transform;


    float 							vizScale;
    double 							m_scale_cameras_down;

protected :
    virtual void draw();
    virtual void init();

public:
    SFMViewer(QWidget *parent = 0):QGLViewer(QGLFormat::defaultFormat(),parent),vizScale(1.0),m_scale_cameras_down(1.0) {

    }
    ~SFMViewer() { saveStateToFile(); }

    virtual void update(std::vector<cv::Point3d> pcld,
                        std::vector<cv::Vec3b> pcldrgb,
                        std::vector<cv::Matx34d> cameras);

public slots:

    void setVizScale(int i) { vizScale = (float)(i); updateGL(); }
};




#endif // SFMVIEWER_H
