#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileSystemModel>
#include <QDebug>
#include "matching.h"
#include "findcameramatrices.h"
#include "Triangulation.h"
#include "opencv2/core/core.hpp"


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>



typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_path_1_returnPressed();

    void on_path_2_returnPressed();

    void on_file_browser_1_clicked(const QModelIndex &index);

    void on_file_browser_2_clicked(const QModelIndex &index);

    void on_pushButton_Matching_clicked();

    void on_pushButton_Reconstruction_clicked();

    //    void on_pushButton_load_clicked();



private:
    Ui::MainWindow *ui;

    QFileSystemModel        *FSM1;
    QFileSystemModel        *FSM2;
    QImage                *image1;
    QImage                *image2;

    int first_image;
    int second_image;

    std::vector<cv::Mat_<cv::Vec3b> > imgs_orig;
    std::vector<std::vector<cv::KeyPoint> > imgpts;

    Mat descriptors1,descriptors2;

    vector<DMatch> matches;
    //after gaining the fundamental matrix
    vector<KeyPoint> img_goodpts1,img_goodpts2;

    Mat K;
    cv::Mat_<double> Kinv;
    Mat distcoeff;

    std::map<int,cv::Matx34d> Pmats;

    std::vector<CloudPoint> outCloud;

    std::vector<cv::Vec3b> pointCloudRGB;

    void GetRGBForPointCloud(
            const std::vector<struct CloudPoint>& _pcloud,
            std::vector<cv::Vec3b>& RGBforCloud
            );


    //pop the data to the visualization module
    std::vector<cv::Point3d> getPointCloud();

    const std::vector<cv::Vec3b>& getPointCloudRGB();

    std::vector<cv::Matx34d> getCameras();
};

#endif // MAINWINDOW_H
