#include "mainwindow.h"
#include "ui_mainwindow.h"


#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <qprocess.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <QFileDialog>
#include <unistd.h>
#include <fcntl.h>

using namespace cv;
using namespace std;


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    ui->path_1->setText("/home/sy/set");
    ui->path_2->setText("/home/sy/set");

    on_path_1_returnPressed();
    on_path_2_returnPressed();

    this->image1 = new QImage();
    this->image2 = new QImage();

    K = (Mat_<double>(3,3) << 2759.48, 0, 1520.69,
         0, 2764.16, 1006.81,
         0, 0, 1);
    distcoeff = (Mat_<double>(5,1) << 0.0, 0.0, 0.0, 0, 0);

    first_image  = 0;
    second_image = 1;
    imgpts.resize(2);
    imgs_orig.resize(2);
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_path_1_returnPressed()
{
    QStringList sImageFilters;
    sImageFilters << "*.jpg" << "*.png" << "*.bmp" ;
    QString sPath = ui->path_1->text();
    qDebug() << "sPath:" << sPath;
    FSM1 = new QFileSystemModel(this);
    FSM1->setFilter(QDir::NoDotAndDotDot | QDir::AllEntries);
    FSM1->setRootPath(sPath);
    FSM1->setNameFilters(sImageFilters);

    ui->file_browser_1->setModel(FSM1);
    ui->file_browser_1->setRootIndex(FSM1->index(sPath));
    ui->file_browser_1->hideColumn(1);
    ui->file_browser_1->hideColumn(2);
    ui->file_browser_1->hideColumn(3);
}

void MainWindow::on_path_2_returnPressed()
{
    QStringList sImageFilters;
    sImageFilters << "*.jpg" << "*.png" << "*.bmp" ;
    QString sPath = ui->path_2->text();
    qDebug() << "sPath:" << sPath;
    FSM2 = new QFileSystemModel(this);
    FSM2->setFilter(QDir::NoDotAndDotDot | QDir::AllEntries);
    FSM2->setRootPath(sPath);
    FSM2->setNameFilters(sImageFilters);


    ui->file_browser_2->setModel(FSM2);
    ui->file_browser_2->setRootIndex(FSM2->index(sPath));
    ui->file_browser_2->hideColumn(1);
    ui->file_browser_2->hideColumn(2);
    ui->file_browser_2->hideColumn(3);
}



void MainWindow::on_file_browser_1_clicked(const QModelIndex &index)
{
    QString filestr;
    filestr = ui->path_1->text();
    filestr.append("/");
    filestr.append(index.data().toString());
    cout << filestr.toStdString() <<endl;
    QString fileName = filestr;
    if(fileName != "")
    {
        if(image1->load(fileName))
        {
            *image1 = image1->scaled(ui->graphicsView_1->width()-10,ui->graphicsView_1->height()-10,Qt::KeepAspectRatio);
            QGraphicsScene *scene = new QGraphicsScene;
            scene->addPixmap(QPixmap::fromImage(*image1));
            ui->graphicsView_1->setScene(scene);
            ui->graphicsView_1->show();
        }
    }
    imgs_orig[first_image] = imread(filestr.toStdString(),CV_LOAD_IMAGE_COLOR);
}

void MainWindow::on_file_browser_2_clicked(const QModelIndex &index)
{
    qDebug() << index.data();
    QString filestr;
    filestr = ui->path_2->text();
    filestr.append("/");
    filestr.append(index.data().toString());
    cout << filestr.toStdString() <<endl;
    QString fileName = filestr;
    if(fileName != "")
    {
        if(image2->load(fileName))
        {
            *image2 = image2->scaled(ui->graphicsView_2->width()-10,ui->graphicsView_2->height()-10,Qt::KeepAspectRatio);
            QGraphicsScene *scene = new QGraphicsScene;
            scene->addPixmap(QPixmap::fromImage(*image2));
            ui->graphicsView_2->setScene(scene);
            ui->graphicsView_2->show();
        }
    }
    imgs_orig[second_image] = imread(filestr.toStdString(),CV_LOAD_IMAGE_COLOR);
}

/* ------------------------------------------------------------------------- */
/** \fn void on_pushButton_Matching_clicked()
*
* \brief matches two selected images
*
* match two select images, save imgpts and descriptors into the yml files.
*/
/* ------------------------------------------------------------------------- */
void MainWindow::on_pushButton_Matching_clicked()
{
    cv::initModule_nonfree();

    QString filestr1;
    QString filestr2;
    filestr1 = ui->path_1->text();
    filestr1.append("/data/");
    filestr1.append(ui->file_browser_1->currentIndex().data().toString());
    filestr1.append(".yml");
    filestr2 = ui->path_2->text();
    filestr2.append("/data/");
    filestr2.append(ui->file_browser_2->currentIndex().data().toString());
    filestr2.append(".yml");

    if(((access(filestr1.toStdString().c_str(),F_OK))!=-1) &&
            ((access(filestr2.toStdString().c_str(),F_OK))!=-1))
    {//the yml files exist restore imgpts and descriptors from the file
        restore_descriptors_from_file(filestr1.toStdString(),imgpts[first_image],descriptors1);
        restore_descriptors_from_file(filestr2.toStdString(),imgpts[second_image],descriptors2);
    }
    else
    {//if the yml file doesn't exist, we need to calculate it and store it as a file
        Mat img1,img2;

        cvtColor(imgs_orig[first_image], img1, CV_BGR2GRAY);
        cvtColor(imgs_orig[second_image], img2, CV_BGR2GRAY);

        matching_get_feature_descriptors(img1,imgpts[first_image],descriptors1);
        matching_get_feature_descriptors(img2,imgpts[second_image],descriptors2);

        save_descriptors_to_file(filestr1.toStdString(),imgpts[first_image],descriptors1);
        save_descriptors_to_file(filestr2.toStdString(),imgpts[second_image],descriptors2);
    }

    cout << filestr1.toStdString() << " has " << imgpts[first_image].size()
         << " points (descriptors " << descriptors1.rows << ")" << endl;
    cout << filestr2.toStdString() << " has " << imgpts[second_image].size()
         << " points (descriptors " << descriptors2.rows << ")" << endl;


    //matching the descriptors
    matching_fb_matcher(descriptors1,descriptors2,matches);

    matching_good_matching_filter(matches);

    //visualization matches
    if(ui->check_view_good_matching->isChecked())
    {
        Mat img_matches;
        drawMatches( imgs_orig[first_image], imgpts[first_image], imgs_orig[second_image], imgpts[second_image],
                     matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        cv::namedWindow("All Matches", CV_WINDOW_NORMAL);
        imshow( "All Matches", img_matches );
    }

    ui->pushButton_Reconstruction->setEnabled(true);
}


/* ------------------------------------------------------------------------- */
/** \fn void on_pushButton_Matching_clicked()
*
* \brief Reconstruction two images and show them
*
* Reconstruction can be devide into following step: First calculate the Fundemantal
* Matrix of two images. Then Multiply the Camera matrix to get the Essential matix.
* After getting the Essential matix, We apply SVD method to the E matrix to get the
* Rotation and tansfer of the camera. Finally we use the position information to
* reconstruct the point cloud and show then in the visualization part/.
*/
/* ------------------------------------------------------------------------- */

void MainWindow::on_pushButton_Reconstruction_clicked()
{

    invert(K, Kinv);

    Pmats[0] = cv::Matx34d(1,0,0,0,
                           0,1,0,0,
                           0,0,1,0);
    Pmats[1] = cv::Matx34d(1,0,0,50,
                           0,1,0,0,
                           0,0,1,0);

    FindCameraMatrices(K,Kinv,distcoeff,
                       imgpts[first_image],imgpts[second_image],
                       img_goodpts1,img_goodpts2,
                       Pmats[0],Pmats[1],
            matches,
            outCloud);

    cout<<endl;
    cout<<"sizeof outCloud is "<< outCloud.size()<<endl;

    outCloud.clear();
    std::vector<cv::KeyPoint> correspImg1Pt;

    TriangulatePoints(img_goodpts1, img_goodpts2, K, Kinv,distcoeff, Pmats[0], Pmats[1], outCloud, correspImg1Pt);
    cout << "Generate" << outCloud.size() <<" Points" <<endl;



    imgpts[0] = imgpts[first_image];
    imgpts[1] = imgpts[second_image];



    for (unsigned int i=0; i<outCloud.size(); i++)
    {
        //cout << "surving" << endl;
        outCloud[i].imgpt_for_img.resize(2);

        outCloud[i].imgpt_for_img[0] = matches[i].queryIdx;
        outCloud[i].imgpt_for_img[1] = matches[i].trainIdx;
    }
    cout << "size of matches" << matches.size() <<endl;

    GetRGBForPointCloud(outCloud,pointCloudRGB);
    ui->Result_viewer->update(getPointCloud(),
                              getPointCloudRGB(),
                              getCameras());
}


void MainWindow::GetRGBForPointCloud(
        const std::vector<struct CloudPoint>& _pcloud,
        std::vector<cv::Vec3b>& RGBforCloud
        )
{
    RGBforCloud.resize(_pcloud.size());
    for (unsigned int i=0; i<_pcloud.size(); i++) {
        unsigned int good_view = 0;
        std::vector<cv::Vec3b> point_colors;
        for(; good_view < imgs_orig.size(); good_view++) {
            if(_pcloud[i].imgpt_for_img[good_view] != -1) {
                int pt_idx = _pcloud[i].imgpt_for_img[good_view];
                if(pt_idx >= imgpts[good_view].size()) {
                    std::cerr << "BUG: point id:" << pt_idx << " should not exist for img #" << good_view << " which has only " << imgpts[good_view].size() << std::endl;
                    continue;
                }
                cv::Point _pt = imgpts[good_view][pt_idx].pt;
                assert(good_view < imgs_orig.size() && _pt.x < imgs_orig[good_view].cols && _pt.y < imgs_orig[good_view].rows);
                point_colors.push_back(imgs_orig[good_view].at<cv::Vec3b>(_pt));
            }
        }
        cv::Scalar res_color = cv::mean(point_colors);
        RGBforCloud[i] = (cv::Vec3b(res_color[0],res_color[1],res_color[2])); //bgr2rgb
    }
}


//pop the data to the visualization module
std::vector<cv::Point3d> MainWindow::getPointCloud()
{
    return CloudPointsToPoints(outCloud);
}

const std::vector<cv::Vec3b>& MainWindow::getPointCloudRGB()
{
    return pointCloudRGB;
}

std::vector<cv::Matx34d> MainWindow::getCameras()
{
    std::vector<cv::Matx34d> v;
    for(std::map<int ,cv::Matx34d>::const_iterator it = Pmats.begin(); it != Pmats.end(); ++it )
    {
        v.push_back( it->second );
    }
    return v;
}
