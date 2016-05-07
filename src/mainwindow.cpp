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

    cloud_.reset (new PointCloudT);
    // The number of points in the cloud
    cloud_->resize (1000);

    // Fill the cloud with random points
    for (size_t i = 0; i < cloud_->points.size (); ++i)
    {
        cloud_->points[i].x = 1024 * (rand () / (RAND_MAX + 1.0f));
        cloud_->points[i].y = 1024 * (rand () / (RAND_MAX + 1.0f));
        cloud_->points[i].z = 1024 * (rand () / (RAND_MAX + 1.0f));
    }
    for (size_t i = 0; i < cloud_->points.size (); ++i)
    {
        cloud_->points[i].r =255;
        cloud_->points[i].g =255;
        cloud_->points[i].b =255;
    }


    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    viewer_->setBackgroundColor (0.3, 0.3, 0.3);
    ui->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    ui->qvtkWidget->update ();
    viewer_->addPointCloud (cloud_, "cloud");
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
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

    cout<<filestr1.toStdString()<<endl;
    cout<<filestr2.toStdString()<<endl;

    if(((access(filestr1.toStdString().c_str(),F_OK))!=-1) &&
       ((access(filestr2.toStdString().c_str(),F_OK))!=-1))
    {
        FileStorage fs1(filestr1.toStdString(), FileStorage::READ);
        FileNode kptFileNode1 = fs1["keypoints"];
        read( kptFileNode1, img1_keypoint );
        fs1["Descriptor"] >> img1_descriptor;
        fs1.release();

        FileStorage fs2(filestr2.toStdString(), FileStorage::READ);
        FileNode kptFileNode2 = fs2["keypoints"];
        read( kptFileNode2, img2_keypoint );
        fs2["Descriptor"] >> img2_descriptor;
        fs2.release();

        cout << "Load keypoint and Descriptor from the files"<<endl;
        cout<< ui->file_browser_1->currentIndex().data().toString().toStdString() << img1_keypoint.size()<<" key points"<<endl;
        cout<< ui->file_browser_2->currentIndex().data().toString().toStdString() << img2_keypoint.size()<<" key points"<<endl;
        cout<< "got the descriptors" <<endl;

    }
    else{

        matching_feature_detector(img1,img2,img1_keypoint,img2_keypoint);

        cout<< ui->file_browser_1->currentIndex().data().toString().toStdString() << img1_keypoint.size()<<" key points"<<endl;
        cout<< ui->file_browser_2->currentIndex().data().toString().toStdString() << img2_keypoint.size()<<" key points"<<endl;

        cout<<"Extracting descriptors from the keypoint"<<endl;
        matching_descriptor_extractor(img1,img2,img1_keypoint,img2_keypoint,
                                      img1_descriptor,img2_descriptor);
        cout<<" Finished"<<endl;

        //saving the keypoint and descriptors into a file, this will save a lot of time when next time use it;
        QString datafilestr;
        datafilestr = ui->path_1->text();
        datafilestr.append("/data/");
        datafilestr.append(ui->file_browser_1->currentIndex().data().toString());
        datafilestr.append(".yml");
        cv::FileStorage fs1(datafilestr.toStdString(), FileStorage::WRITE);
        write( fs1 , "keypoints", img1_keypoint);
        fs1 << "Descriptor" << img1_descriptor << "distCoeffs";
        fs1.release();

        datafilestr = ui->path_2->text();
        datafilestr.append("/data/");
        datafilestr.append(ui->file_browser_2->currentIndex().data().toString());
        datafilestr.append(".yml");
        cv::FileStorage fs2(datafilestr.toStdString(), FileStorage::WRITE);
        write( fs2 , "keypoints", img2_keypoint);
        fs2 << "Descriptor" << img2_descriptor << "distCoeffs";
        fs2.release();
    }




    matching_fb_matcher(img1_descriptor,img2_descriptor,matches_1);


    if(ui->check_view_matching->isChecked())
    {
        Mat img_matches;
        drawMatches( img1_orig, img1_keypoint, img2_orig, img2_keypoint,
                     matches_1, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        cv::namedWindow("All Matches", CV_WINDOW_NORMAL);
        imshow( "All Matches", img_matches );
        //waitKey(100);
        //destroyWindow("Feature Matches");
    }

    matching_good_matching_filter(matches_1,matches_2,
                                  img1_keypoint,img2_keypoint,
                                  img1_good_keypoint,img2_good_keypoint);

    if(ui->check_view_good_matching->isChecked())
    {
        Mat img_matches;
        drawMatches( img1_orig, img1_keypoint, img2_orig, img2_keypoint,
                     matches_2, img_matches, Scalar::all(-1), Scalar::all(-1),
                     vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
        //-- Show detected matches
        cv::namedWindow("Good Matches", CV_WINDOW_NORMAL);
        imshow( "Good Matches", img_matches );
    }

    ui->pushButton_Estimating->setEnabled(true);
}

/* ------------------------------------------------------------------------- */
/** \fn MainWindow::on_pushButton_Estimating_clicked()
*
* \brief Estimating the P and P1 matrix
*
*/
/* ------------------------------------------------------------------------- */
void MainWindow::on_pushButton_Estimating_clicked()
{
    //    GetFundamentalMat(img1_keypoint,img2_keypoint,
    //                      img1_very_good_keypoint,img2_very_good_keypoint,
    //                      matches_2,matches_3);
    Mat K = (Mat_<double>(3,3) << 2000, 0, 0,
             0, 2000, 0,
             0, 0, 1);
    cv::Mat_<double> Kinv;
    Mat distcoeff = (Mat_<double>(5,1) << 0.0, 0.0, 0.0, 0, 0);



    invert(K, Kinv);
    P = cv::Matx34d(1,0,0,0,
                    0,1,0,0,
                    0,0,1,0);
    P1 = cv::Matx34d(1,0,0,50,
                     0,1,0,0,
                     0,0,1,0);

    FindCameraMatrices(K,Kinv,distcoeff,
                       img1_keypoint,img2_keypoint,
                       img1_very_good_keypoint,img2_very_good_keypoint,
                       P,P1,
                       matches_2,matches_3,
                       outCloud);

    cout<<endl;
    cout<<"sizeof outCloud is "<< outCloud.size()<<endl;
}


/* ------------------------------------------------------------------------- */
/** \fn MainWindow::on_pushButton_Reconstruction_clicked()
*
* \brief Reconstruction the point cloud
*
*/
/* ------------------------------------------------------------------------- */
void MainWindow::on_pushButton_Reconstruction_clicked()
{

    Mat K = (Mat_<double>(3,3) << 2000, 0, 0,
             0, 2000, 0,
             0, 0, 1);
    cv::Mat_<double> Kinv;
    Mat distcoeff = (Mat_<double>(5,1) << 0.0, 0.0, 0.0, 0, 0);

    invert(K, Kinv);

    outCloud.clear();
    std::vector<cv::KeyPoint> correspImg1Pt;

    TriangulatePoints(img1_very_good_keypoint, img2_very_good_keypoint, K, Kinv,distcoeff, P, P1, outCloud, correspImg1Pt);
    cout << "Generate" << outCloud.size() <<" Points" <<endl;

    PointCloudT::Ptr cloud_tmp (new PointCloudT);
    unsigned int size = outCloud.size();
    cout << "Generate" << outCloud.size() <<" Points" <<endl;
    cloud_tmp->resize (size);


    std::vector<cv::Vec3b> point_colors;
    // Fill the cloud with random points
    for (size_t i = 0; i < size; ++i)
    {

        cv::Point _pt = correspImg1Pt[i].pt;
        point_colors.push_back(img1_orig.at<cv::Vec3b>(_pt));
        cloud_tmp->points[i].x = outCloud[i].pt.x;
        cloud_tmp->points[i].y = outCloud[i].pt.y;
        cloud_tmp->points[i].z = outCloud[i].pt.z;
    }

    for (size_t i = 0; i < size; ++i)
    {
        cloud_tmp->points[i].r =point_colors[i].val[0];;
        cloud_tmp->points[i].g =point_colors[i].val[1];;
        cloud_tmp->points[i].b =point_colors[i].val[2];;
    }

    if (cloud_tmp->is_dense)
        pcl::copyPointCloud (*cloud_tmp, *cloud_);

    viewer_->updatePointCloud (cloud_, "cloud");
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
}


void MainWindow::on_pushButton_load_clicked()
{
    QString filename = QFileDialog::getOpenFileName (this, tr ("Open point cloud"), "/home/", tr ("Point cloud data (*.pcd *.ply)"));

    PCL_INFO("File chosen: %s\n", filename.toStdString ().c_str ());
    PointCloudT::Ptr cloud_tmp (new PointCloudT);

    if (filename.isEmpty ())
        return;

    int return_status;
    if (filename.endsWith (".pcd", Qt::CaseInsensitive))
        return_status = pcl::io::loadPCDFile (filename.toStdString (), *cloud_tmp);
    else
        return_status = pcl::io::loadPLYFile (filename.toStdString (), *cloud_tmp);

    if (return_status != 0)
    {
        PCL_ERROR("Error reading point cloud %s\n", filename.toStdString ().c_str ());
        return;
    }

    // If point cloud contains NaN values, remove them before updating the visualizer point cloud
    if (cloud_tmp->is_dense)
        pcl::copyPointCloud (*cloud_tmp, *cloud_);
    else
    {
        PCL_WARN("Cloud is not dense! Non finite points will be removed\n");
        std::vector<int> vec;
        pcl::removeNaNFromPointCloud (*cloud_tmp, *cloud_, vec);
    }

    //colorCloudDistances ();
    viewer_->updatePointCloud (cloud_, "cloud");
    viewer_->resetCamera ();
    ui->qvtkWidget->update ();
}






