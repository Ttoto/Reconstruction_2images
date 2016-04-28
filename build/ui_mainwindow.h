/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 4.8.6
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QCheckBox>
#include <QtGui/QGraphicsView>
#include <QtGui/QGridLayout>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QTreeView>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QWidget *layoutWidget;
    QGridLayout *gridLayout;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *path_1;
    QTreeView *file_browser_1;
    QLabel *lable_1;
    QGraphicsView *graphicsView_1;
    QVBoxLayout *verticalLayout_3;
    QLineEdit *path_2;
    QTreeView *file_browser_2;
    QLabel *lable_2;
    QGraphicsView *graphicsView_2;
    QVTKWidget *qvtkWidget;
    QSplitter *splitter;
    QPushButton *pushButton_Matching;
    QCheckBox *check_view_matching;
    QCheckBox *check_view_good_matching;
    QPushButton *pushButton_Estimating;
    QCheckBox *checkBox;
    QPushButton *pushButton_Reconstruction;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QPushButton *pushButton_load;
    QPushButton *pushButton_save;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1384, 559);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        layoutWidget = new QWidget(centralWidget);
        layoutWidget->setObjectName(QString::fromUtf8("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 10, 501, 481));
        gridLayout = new QGridLayout(layoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        path_1 = new QLineEdit(layoutWidget);
        path_1->setObjectName(QString::fromUtf8("path_1"));

        verticalLayout_2->addWidget(path_1);

        file_browser_1 = new QTreeView(layoutWidget);
        file_browser_1->setObjectName(QString::fromUtf8("file_browser_1"));
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(file_browser_1->sizePolicy().hasHeightForWidth());
        file_browser_1->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(file_browser_1);

        lable_1 = new QLabel(layoutWidget);
        lable_1->setObjectName(QString::fromUtf8("lable_1"));

        verticalLayout_2->addWidget(lable_1);

        graphicsView_1 = new QGraphicsView(layoutWidget);
        graphicsView_1->setObjectName(QString::fromUtf8("graphicsView_1"));

        verticalLayout_2->addWidget(graphicsView_1);


        gridLayout->addLayout(verticalLayout_2, 0, 0, 1, 1);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        path_2 = new QLineEdit(layoutWidget);
        path_2->setObjectName(QString::fromUtf8("path_2"));

        verticalLayout_3->addWidget(path_2);

        file_browser_2 = new QTreeView(layoutWidget);
        file_browser_2->setObjectName(QString::fromUtf8("file_browser_2"));
        sizePolicy.setHeightForWidth(file_browser_2->sizePolicy().hasHeightForWidth());
        file_browser_2->setSizePolicy(sizePolicy);

        verticalLayout_3->addWidget(file_browser_2);

        lable_2 = new QLabel(layoutWidget);
        lable_2->setObjectName(QString::fromUtf8("lable_2"));

        verticalLayout_3->addWidget(lable_2);

        graphicsView_2 = new QGraphicsView(layoutWidget);
        graphicsView_2->setObjectName(QString::fromUtf8("graphicsView_2"));

        verticalLayout_3->addWidget(graphicsView_2);


        gridLayout->addLayout(verticalLayout_3, 0, 1, 1, 1);

        qvtkWidget = new QVTKWidget(centralWidget);
        qvtkWidget->setObjectName(QString::fromUtf8("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(720, 10, 640, 480));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(50);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(qvtkWidget->sizePolicy().hasHeightForWidth());
        qvtkWidget->setSizePolicy(sizePolicy1);
        qvtkWidget->setMinimumSize(QSize(640, 480));
        splitter = new QSplitter(centralWidget);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setGeometry(QRect(520, 10, 191, 241));
        splitter->setOrientation(Qt::Vertical);
        pushButton_Matching = new QPushButton(splitter);
        pushButton_Matching->setObjectName(QString::fromUtf8("pushButton_Matching"));
        splitter->addWidget(pushButton_Matching);
        check_view_matching = new QCheckBox(splitter);
        check_view_matching->setObjectName(QString::fromUtf8("check_view_matching"));
        check_view_matching->setChecked(false);
        splitter->addWidget(check_view_matching);
        check_view_good_matching = new QCheckBox(splitter);
        check_view_good_matching->setObjectName(QString::fromUtf8("check_view_good_matching"));
        check_view_good_matching->setChecked(false);
        splitter->addWidget(check_view_good_matching);
        pushButton_Estimating = new QPushButton(splitter);
        pushButton_Estimating->setObjectName(QString::fromUtf8("pushButton_Estimating"));
        pushButton_Estimating->setEnabled(false);
        splitter->addWidget(pushButton_Estimating);
        checkBox = new QCheckBox(splitter);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        splitter->addWidget(checkBox);
        pushButton_Reconstruction = new QPushButton(splitter);
        pushButton_Reconstruction->setObjectName(QString::fromUtf8("pushButton_Reconstruction"));
        pushButton_Reconstruction->setEnabled(true);
        splitter->addWidget(pushButton_Reconstruction);
        widget = new QWidget(splitter);
        widget->setObjectName(QString::fromUtf8("widget"));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        horizontalLayout->setContentsMargins(0, 0, 0, 0);
        pushButton_load = new QPushButton(widget);
        pushButton_load->setObjectName(QString::fromUtf8("pushButton_load"));

        horizontalLayout->addWidget(pushButton_load);

        pushButton_save = new QPushButton(widget);
        pushButton_save->setObjectName(QString::fromUtf8("pushButton_save"));

        horizontalLayout->addWidget(pushButton_save);

        splitter->addWidget(widget);
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1384, 25));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        lable_1->setText(QApplication::translate("MainWindow", "Image Preview", 0, QApplication::UnicodeUTF8));
        lable_2->setText(QApplication::translate("MainWindow", "Image Preview", 0, QApplication::UnicodeUTF8));
        pushButton_Matching->setText(QApplication::translate("MainWindow", "Matching", 0, QApplication::UnicodeUTF8));
        check_view_matching->setText(QApplication::translate("MainWindow", "view matching", 0, QApplication::UnicodeUTF8));
        check_view_good_matching->setText(QApplication::translate("MainWindow", "view good matching", 0, QApplication::UnicodeUTF8));
        pushButton_Estimating->setText(QApplication::translate("MainWindow", "Estimating", 0, QApplication::UnicodeUTF8));
        checkBox->setText(QApplication::translate("MainWindow", "very good matching", 0, QApplication::UnicodeUTF8));
        pushButton_Reconstruction->setText(QApplication::translate("MainWindow", "Reconstruction", 0, QApplication::UnicodeUTF8));
        pushButton_load->setText(QApplication::translate("MainWindow", "Load FIle", 0, QApplication::UnicodeUTF8));
        pushButton_save->setText(QApplication::translate("MainWindow", "Save File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
