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

    imgpts.clear();

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
}

/* ------------------------------------------------------------------------- */
/** \fn MainWindow::on_pushButton_Matching_clicked()
*
* \brief Matching two iamge
*
*/
/* ------------------------------------------------------------------------- */
void MainWindow::on_pushButton_Matching_clicked()
{
    cv::initModule_nonfree();

    QString filestr1;
    filestr1 = ui->path_1->text();
    filestr1.append("/");
    filestr1.append(ui->file_browser_1->currentIndex().data().toString());
    QString filestr2;
    filestr2 = ui->path_2->text();
    filestr2.append("/");
    filestr2.append(ui->file_browser_2->currentIndex().data().toString());


    img1_orig = imread(filestr1.toStdString(),CV_LOAD_IMAGE_COLOR);
    img2_orig = imread(filestr2.toStdString(),CV_LOAD_IMAGE_COLOR);


    Mat img1,img2;
    cvtColor(img1_orig, img1, CV_BGR2GRAY);
    cvtColor(img2_orig, img2, CV_BGR2GRAY);

    filestr1 = ui->path_1->text();
    filestr1.append("/data/");
    filestr1.append(ui->file_browser_1->currentIndex().data().toString());
    filestr1.append(".yml");

    filestr2 = ui->path_2->text();
    filestr2.append("/data/");
    filestr2.append(ui->file_browser_2->currentIndex().data().toString());
    filestr2.append(".yml");

    //get the keypoint and descriptions
    if(((access(filestr1.toStdString().c_str(),F_OK))!=-1) &&
            ((access(filestr2.toStdString().c_str(),F_OK))!=-1))
    {
        restore_descriptors_from_file(filestr1.toStdString(),imgpts1,descriptors1);
        restore_descriptors_from_file(filestr2.toStdString(),imgpts2,descriptors2);
    }
    else{

        matching_get_feature_descriptors(img1,imgpts1,descriptors1);
        matching_get_feature_descriptors(img2,imgpts2,descriptors2);

        save_descriptors_to_file(filestr1.toStdString(),imgpts1,descriptors1);
        save_descriptors_to_file(filestr2.toStdString(),imgpts2,descriptors2);
    }

    cout << filestr1.toStdString() << " has " << imgpts1.size()
         << " points (descriptors " << descriptors1.rows << ")" << endl;
    cout << filestr2.toStdString() << " has " << imgpts2.size()
         << " points (descriptors " << descriptors2.rows << ")" << endl;


    //matching the descriptors
    matching_fb_matcher(descriptors1,descriptors2,matches);

    matching_good_matching_filter(matches);


    if(ui->check_view_good_matching->isChecked())
    {
        Mat img_matches;
        drawMatches( img1_orig, imgpts1, img2_orig, imgpts2,
                     matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        cv::namedWindow("All Matches", CV_WINDOW_NORMAL);
        imshow( "All Matches", img_matches );
    }

    ui->pushButton_Reconstruction->setEnabled(true);
}




void MainWindow::on_pushButton_Reconstruction_clicked()
{

    invert(K, Kinv);

    P = cv::Matx34d(1,0,0,0,
                    0,1,0,0,
                    0,0,1,0);
    P1 = cv::Matx34d(1,0,0,50,
                     0,1,0,0,
                     0,0,1,0);

    FindCameraMatrices(K,Kinv,distcoeff,
                       imgpts1,imgpts2,
                       img1_very_good_keypoint,img2_very_good_keypoint,
                       P,P1,
                       matches,
                       outCloud);

    cout<<endl;
    cout<<"sizeof outCloud is "<< outCloud.size()<<endl;

    outCloud.clear();
    std::vector<cv::KeyPoint> correspImg1Pt;

    TriangulatePoints(img1_very_good_keypoint, img2_very_good_keypoint, K, Kinv,distcoeff, P, P1, outCloud, correspImg1Pt);
    cout << "Generate" << outCloud.size() <<" Points" <<endl;

    imgpts.resize(2);

    imgpts[0] = imgpts1;
    imgpts[1] = imgpts2;

    imgs_orig.resize(2);
    imgs_orig[0] = img1_orig;
    imgs_orig[1] = img2_orig;

    Pmats[0] = P;
    Pmats[1] = P1;

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

                //				std::stringstream ss; ss << "patch " << good_view;
                //				imshow_250x250(ss.str(), imgs_orig[good_view](cv::Range(_pt.y-10,_pt.y+10),cv::Range(_pt.x-10,_pt.x+10)));
            }
        }
        //		cv::waitKey(0);
        cv::Scalar res_color = cv::mean(point_colors);
        RGBforCloud[i] = (cv::Vec3b(res_color[0],res_color[1],res_color[2])); //bgr2rgb
//        if(good_view == imgs.size()) //nothing found.. put red dot
//            RGBforCloud.push_back(cv::Vec3b(255,0,0));
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
