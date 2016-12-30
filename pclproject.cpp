#include "pclproject.h"

PclProject::PclProject(QObject *parent) : QObject(parent)
{
    // setup visualizer widget
    visualizerWidget_ = new QVTKWidget();

    // setup controlbar
    this->setupControlBar();

    // setup visualizer
    this->setupVisualizer();

    // setup cropBox
    this->setupCropBox();

    // setup mainwindow
    this->setupMainWindow();
}

PclProject::~PclProject()
{
    delete mainWindow_;
    delete controlBar_;
}

void PclProject::setupMainWindow()
{
    mainWindow_ = new MainWindow();
    mainWindow_->setVisualizerWidget(visualizerWidget_);
    mainWindow_->show();

    connect(mainWindow_, SIGNAL(loadCloud(QString)), this, SLOT(loadCloud(QString)));
    connect(mainWindow_, SIGNAL(saveCloud(QString)), this, SLOT(saveCloud(QString)));
    connect(mainWindow_, SIGNAL(unloadCloud()), this, SLOT(unloadCloud()));
    connect(mainWindow_, SIGNAL(unloadAllClouds()), this, SLOT(unloadAllClouds()));
    connect(mainWindow_, SIGNAL(useNextActiveCloud()), this, SLOT(useNextActiveCloud()));
    connect(mainWindow_, SIGNAL(usePreviousActiveCloud()), this, SLOT(usePreviousActiveCloud()));

    connect(mainWindow_, SIGNAL(showControlBar()), this, SLOT(showControlBar()));

    connect(mainWindow_, SIGNAL(translateCropBox(double, double, double)), this, SLOT(translateCropBox(double, double, double)));
    connect(mainWindow_, SIGNAL(cropCloud()), this, SLOT(cropCloud()));
}

void PclProject::setupControlBar()
{
    controlBar_ = new ControlBar();


}

void PclProject::setupVisualizer()
{
    visualizer_ = new PointCloudVisualizer(visualizerWidget_, this);
    visualizer_->setCameraPosition(100, 100, 100);
    visualizer_->setBackgroundColor(0.1, 0.1, 0.1);
    visualizer_->showCoordinateSystem(50);
    visualizer_->showHeadUpDisplay();
}

void PclProject::setupCropBox()
{
    cropBox_ = new CropBox(this);
    cropBoxEnabled_ = false;
}

void PclProject::loadCloud(QString filePath)
{
    // get filename without extension
    QFileInfo fileInfo(filePath);
    QString fileName = fileInfo.baseName();

    PointCloud* cloud = new PointCloud(fileName, this);

    cloud->loadFile(filePath);
    cloud->translateToOrigin();

    this->addCloud(cloud);

    //ui->statusBar->showMessage(tr("Cloud Loaded"), 2000);
}

void PclProject::saveCloud(QString filePath)
{
    if (!currentCloud_) return;
    currentCloud_->saveFile(filePath);
}

void PclProject::enableCropBox()
{
    visualizer_->addCropBox(cropBox_);
    cropBoxEnabled_ = true;
}

void PclProject::disableCropBox()
{
    visualizer_->removeCropBox(cropBox_);
    cropBoxEnabled_ = false;
}

void PclProject::addCloud(PointCloud *cloud)
{
    // if this is the first cloud set as current cloud
    if (!currentCloud_) {
        currentCloud_ = cloud;
        mainWindow_->setActionSaveCloudEnabled(true);
    }

    visualizer_->addPointCloud(cloud);
    clouds_.push_back(cloud);

    // update
    this->publishActiveCloud();
}

void PclProject::unloadCloud()
{
    if (!currentCloud_) return;

    visualizer_->removePointCloud(currentCloud_);

    int cloudIndex = clouds_.indexOf(currentCloud_);
    clouds_.remove(cloudIndex);
    if (clouds_.isEmpty()) {    // no cloud left
        delete currentCloud_;
        currentCloud_ = NULL;
        mainWindow_->setActionSaveCloudEnabled(false);
    }
    else currentCloud_ = clouds_.at(0);

    // update
    this->publishActiveCloud();
}

void PclProject::unloadAllClouds()
{
    if (!currentCloud_) return;

     visualizer_->removeAllPointClouds();

    qDeleteAll(clouds_);
    clouds_.clear();
    currentCloud_ = NULL;
    mainWindow_->setActionSaveCloudEnabled(false);

    // update
    this->publishActiveCloud();
}

void PclProject::useNextActiveCloud()
{
    // error check
    if (!currentCloud_) return;

    // overwrite active cloud
    size_t number = clouds_.indexOf(currentCloud_);
    if (number == clouds_.size()-1) currentCloud_ = clouds_.first();
    else currentCloud_ = clouds_.at(number+1);

    // update
    this->publishActiveCloud();
}

void PclProject::usePreviousActiveCloud()
{
    // error check
    if (!currentCloud_) return;

    // overwrite active cloud
    size_t number = clouds_.indexOf(currentCloud_);
    if (number == 0) currentCloud_ = clouds_.last();
    else currentCloud_ = clouds_.at(number-1);

    // update
    this->publishActiveCloud();
}

void PclProject::publishActiveCloud()
{
    cropBox_->setActiveCloud(currentCloud_);
    visualizer_->updateHeadUpDisplay(currentCloud_);
}

void PclProject::showControlBar()
{
    if (controlBar_->isHidden()) controlBar_->show();
}

void PclProject::translateCropBox(double x, double y, double z)
{
    if (!cropBoxEnabled_) return;
    cropBox_->translate(x, y, z);
}

void PclProject::cropCloud()
{
    if (!cropBoxEnabled_) return;
    cropBox_->cropCloud();
}
