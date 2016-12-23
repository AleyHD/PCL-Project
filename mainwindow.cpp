#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    controlBar_(new ControlBar(this))
{
    ui->setupUi(this);
    this->setWindowTitle ("PCL-Task2");

    // setup event filter for key presses
    qApp->installEventFilter(this);

    // setup visualizer
    this->setupVisualizer();

    // setup cropBox
    this->setupCropBox();
    cropBoxEnabled_ = false;

}

MainWindow::~MainWindow()
{
    delete ui;

}

void MainWindow::setupVisualizer()
{
    visualizer_ = new PointCloudVisualizer(ui->widget_cloudVisualizer, this);
    visualizer_->setCameraPosition(100, 100, 100);
    visualizer_->setBackgroundColor(0.1, 0.1, 0.1);
    visualizer_->showCoordinateSystem(50);
    visualizer_->showHeadUpDisplay();
}

void MainWindow::setupCloud(QString &filePath)
{
    // get filename without extension
    QFileInfo fileInfo(filePath);
    QString fileName = fileInfo.baseName();

    PointCloud* cloud = new PointCloud(fileName, this);

    cloud->loadFile(filePath);
    cloud->translateToOrigin();

    this->addCloud(cloud);

    ui->statusBar->showMessage(tr("Cloud Loaded"), 2000);

}

void MainWindow::setupCropBox()
{
    cropBox_ = new CropBox(this);
}

void MainWindow::enableCropBox()
{
    visualizer_->addCropBox(cropBox_);
    cropBoxEnabled_ = true;
}

void MainWindow::disableCropBox()
{
    visualizer_->removeCropBox(cropBox_);
    cropBoxEnabled_ = false;
}

void MainWindow::addCloud(PointCloud *cloud)
{
    // if this is the first cloud set as current cloud
    if (!currentCloud_) {
        currentCloud_ = cloud;
        cropBox_->setActiveCloud(currentCloud_);
        visualizer_->updateHeadUpDisplay(currentCloud_);
    }
    // add the cloud to the visualizer & the cloud container
    visualizer_->addPointCloud(cloud);
    clouds_.push_back(cloud);
}

void MainWindow::delCloud()
{
    if (!currentCloud_) return;

    visualizer_->removePointCloud(currentCloud_);

    int cloudIndex = clouds_.indexOf(currentCloud_);
    clouds_.remove(cloudIndex);
    if (clouds_.isEmpty()) {    // no cloud left
        delete currentCloud_;
        currentCloud_ = NULL;
    }
    else currentCloud_ = clouds_.at(0);
    // update
    cropBox_->setActiveCloud(currentCloud_);
    visualizer_->updateHeadUpDisplay(currentCloud_);
}

void MainWindow::delClouds()
{
    if (!currentCloud_) return;

     visualizer_->removeAllPointClouds();

    qDeleteAll(clouds_);
    clouds_.clear();
    currentCloud_ = NULL;
    // update
    cropBox_->setActiveCloud(currentCloud_);
    visualizer_->updateHeadUpDisplay(currentCloud_);
}

void MainWindow::changeActiveCloud()
{
    // error check
    if (!currentCloud_) return;
    // overwrite active cloud
    size_t number = clouds_.indexOf(currentCloud_);
    if (number == clouds_.size()-1) currentCloud_ = clouds_.first();
    else currentCloud_ = clouds_.at(number+1);

    // update
    cropBox_->setActiveCloud(currentCloud_);
    visualizer_->updateHeadUpDisplay(currentCloud_);

}

void MainWindow::resetCamera()
{
    visualizer_->setCameraPosition(50, 50, 50);
}

void MainWindow::on_action_loadCloud_triggered()
{
    // setup file dialog
    QString title = "Select Cloud";
    QString path = "/";
    QString filter = tr("PCD (*.pcd)");
    QString filePath = QFileDialog::getOpenFileName(this, title, path, filter);
    // check if filePath is ok
    if (filePath.isEmpty()) return;
    // setup cloud
    this->setupCloud(filePath);
}

void MainWindow::on_action_saveCloud_triggered()
{
    if (!currentCloud_) return;
    // setup file dialog
    QString title = "Select Cloud";
    QString path = "/";
    QString filter = tr("PCD (*.pcd)");
    QString filePath = QFileDialog::getSaveFileName(this,title, path, filter);
    // check if filePath is ok
    if (filePath.isEmpty()) return;
    // set file handle
    QFile file(filePath);
    // setup cloud
    if (!filePath.endsWith(".pcd")) filePath += ".pcd";
    currentCloud_->saveFile(filePath);
}

void MainWindow::on_action_unloadCloud_triggered()
{
    if (!currentCloud_) return;
    this->delCloud();
}

void MainWindow::extractCloud()
{
    // error check
    if (!currentCloud_) return;
    // extract and save
    PointCloud* extractedCloud(new PointCloud("plane1", this));
    currentCloud_->extractPlane(extractedCloud);
    //this->addCloud(extractedCloud);
}

bool MainWindow::eventFilter(QObject *object, QEvent *event)
{
    // error check
    if(!currentCloud_) return QObject::eventFilter(object, event);

    if (event->type() == QEvent::KeyPress)
    {
        //if(obj == ui->listWidget)
        //{
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
            if (cropBoxEnabled_) {
            // cropBox Movement
            if(keyEvent->key() == Qt::Key_4) cropBox_->translate(1.0, 0.0, 0.0);
            if(keyEvent->key() == Qt::Key_6) cropBox_->translate(-1.0, 0.0, 0.0);
            if(keyEvent->key() == Qt::Key_8) cropBox_->translate(0.0, -1.0, 0.0);
            if(keyEvent->key() == Qt::Key_2) cropBox_->translate(0.0, 1.0, 0.0);
            if(keyEvent->key() == Qt::Key_9) cropBox_->translate(0.0, 0.0, 1.0);
            if(keyEvent->key() == Qt::Key_7) cropBox_->translate(0.0, 0.0, -1.0);
            // cropBox Misc
            if(keyEvent->key() == Qt::Key_Delete) cropBox_->cropCloud();
            }
            // cloud movement
            if(keyEvent->key() == Qt::Key_A) currentCloud_->translate(1.0, 0.0, 0.0);
            if(keyEvent->key() == Qt::Key_D) currentCloud_->translate(-1.0, 0.0, 0.0);
            if(keyEvent->key() == Qt::Key_W) currentCloud_->translate(0.0, -1.0, 0.0);
            if(keyEvent->key() == Qt::Key_S) currentCloud_->translate(0.0, 1.0, 0.0);
            if(keyEvent->key() == Qt::Key_Y) currentCloud_->translate(0.0, 0.0, 1.0);
            if(keyEvent->key() == Qt::Key_C) currentCloud_->translate(0.0, 0.0, -1.0);
            // cloud features
            if(keyEvent->key() == Qt::Key_Insert) this->extractCloud();
            // ui Misc
            if(keyEvent->key() == Qt::Key_Up) this->changeActiveCloud();
            if(keyEvent->key() == Qt::Key_P) currentCloud_->setResolution(0.1);
        //}
    }
    return QObject::eventFilter(object, event);
}

void MainWindow::on_action_unloadAllClouds_triggered()
{
    this->delClouds();
}

void MainWindow::on_action_showControlBar_triggered()
{
    controlBar_->show();
}
