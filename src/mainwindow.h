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
#include <pcl/visualization/pcl_visualizer.h>

// Boost
#include <boost/math/special_functions/round.hpp>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

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

    void on_pushButton_Estimating_clicked();

    void on_pushButton_Reconstruction_clicked();

    void on_pushButton_load_clicked();

private:
    Ui::MainWindow *ui;

    QFileSystemModel        *FSM1;
    QFileSystemModel        *FSM2;
    QImage                *image1;
    QImage                *image2;


    //original input mat
    Mat img1_orig;
    Mat img2_orig;

    //matching step1 output
    vector<KeyPoint> img1_keypoint,img2_keypoint;
    //matching step2 output img_descriptor
    Mat img1_descriptor,img2_descriptor;
    //matching step3 output all_matches good_matches img_good_keypoint
    vector<DMatch> matches_1;

    //matching filter
    vector<DMatch> matches_2;
    vector<KeyPoint> img1_good_keypoint,img2_good_keypoint;

    //after gaining the fundamental matrix
    vector<DMatch> matches_3;
    vector<KeyPoint> img1_very_good_keypoint,img2_very_good_keypoint;

    cv::Matx34d P;
    cv::Matx34d P1;
    std::vector<CloudPoint> outCloud;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;

    PointCloudT::Ptr cloud_;
};

#endif // MAINWINDOW_H
