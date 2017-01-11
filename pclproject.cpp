#include "pclproject.h"

PclProject::PclProject(QObject *parent) : QObject(parent)
{
    // setup visualizer widget
    visualizerWidget_ = new QVTKWidget();

    // setup mainwindow (parent of control bar)
    this->setupMainWindow();

    // setup cropBox
    this->setupCropBox();

    // setup visualizer
    this->setupVisualizer();

    // setup controlbar
    this->setupControlBar();

    // fill variables
    transformFilepath_ = QCoreApplication::applicationDirPath()+"/transform.csv";
}

PclProject::~PclProject()
{
    delete mainWindow_;
}

void PclProject::setupMainWindow()
{
    mainWindow_ = new MainWindow();
    mainWindow_->setVisualizerWidget(visualizerWidget_);
    mainWindow_->showMaximized();

    connect(mainWindow_, SIGNAL(loadCloud(QString)), this, SLOT(loadCloud(QString)));
    connect(mainWindow_, SIGNAL(saveCloud(QString)), this, SLOT(saveCloud(QString)));
    connect(mainWindow_, SIGNAL(saveCloud()), this, SLOT(saveCloud()));
    connect(mainWindow_, SIGNAL(unloadCloud()), this, SLOT(unloadCloud()));
    connect(mainWindow_, SIGNAL(unloadAllClouds()), this, SLOT(unloadAllClouds()));

    connect(mainWindow_, SIGNAL(applyVoxelGrid(float)), this, SLOT(applyVoxelGrid(float)));
    connect(mainWindow_, SIGNAL(removeCloudOutliers(int, double)), this, SLOT(removeCloudOutliers(int, double)));

    connect(mainWindow_, SIGNAL(translateCropBox(double, double, double)), this, SLOT(translateCropBox(double, double, double)));
    connect(mainWindow_, SIGNAL(cropCloud()), this, SLOT(cropCloud()));

    connect(mainWindow_, SIGNAL(showControlBar()), this, SLOT(showControlBar()));
}

void PclProject::setupControlBar()
{
    controlBar_ = new ControlBar(mainWindow_);

    connect(controlBar_, SIGNAL(showCos(double)), this, SLOT(showCos(double)));
    connect(controlBar_, SIGNAL(hideCos()), this, SLOT(hideCos()));
    connect(controlBar_, SIGNAL(showHud()), this, SLOT(showHud()));
    connect(controlBar_, SIGNAL(hideHud()), this, SLOT(hideHud()));
    connect(controlBar_, SIGNAL(resetCamera()), this, SLOT(resetCamera()));

    connect(controlBar_, SIGNAL(setActiveCloud(int)), this, SLOT(setActiveCloud(int)));
    connect(controlBar_, SIGNAL(hideActiveCloud(bool)), this, SLOT(hideActiveCloud(bool)));
    connect(controlBar_, SIGNAL(alignToCloud(QString)), this, SLOT(alignToCloud(QString)));
    connect(controlBar_, SIGNAL(appendToCloud(QString)), this, SLOT(appendToCloud(QString)));

    connect(controlBar_, SIGNAL(setCloudPose(double, double, double, double, double, double)), this, SLOT(setCloudPose(double, double, double, double, double, double)));

    connect(controlBar_, SIGNAL(enableCropBox()), this, SLOT(enableCropBox()));
    connect(controlBar_, SIGNAL(disableCropBox()), this, SLOT(disableCropBox()));
    connect(controlBar_, SIGNAL(setCropBoxSize(double)), this, SLOT(setCropBoxSize(double)));
    connect(controlBar_, SIGNAL(setCropBoxMovementFactor(double)), this, SLOT(setCropBoxMovementFactor(double)));

    this->showControlBar();
}

void PclProject::setupVisualizer()
{
    visualizer_ = new PointCloudVisualizer(visualizerWidget_, this);
    this->resetCamera();
    visualizer_->setBackgroundColor(0.1, 0.1, 0.1);
    visualizer_->showCoordinateSystem(20);
    visualizer_->showHeadUpDisplay();
    this->enableCropBox();
}

void PclProject::setupCropBox()
{
    cropBox_ = new CropBox(this);
    cropBoxEnabled_ = true;
}

/******************************** MainWindow ********************************/

void PclProject::loadCloud(QString filePath)
{
    // get filename without extension
    QFileInfo fileInfo(filePath);
    QString fileName = fileInfo.baseName();

    PointCloud* cloud = new PointCloud(fileName, this);

    cloud->loadFile(filePath);
    cloud->loadTransformFromFile(transformFilepath_);
    //cloud->translateToOrigin();

    this->addCloud(cloud);

    QString message = "Cloud Loaded";
    mainWindow_->showMessageOnStatusBar(message, 2000);
}

void PclProject::saveCloud(QString filePath)
{
    if (!currentCloud_) return;

    // hide cloud in case of new name
    visualizer_->hidePointCloud(currentCloud_);

    // save file & transform
    currentCloud_->saveFile(filePath);
    currentCloud_->saveTransformToFile(transformFilepath_);

    // add cloud in visualizer again
    visualizer_->addPointCloud(currentCloud_);

    // update name to the needs
    int cloudIndex = clouds_.indexOf(currentCloud_);
    controlBar_->updateAvailableCloud(cloudIndex, currentCloud_->name());
    this->publishActiveCloud();
}

void PclProject::saveCloud()
{
    if (!currentCloud_) return;

    // save file & transform
    currentCloud_->saveFile();
    currentCloud_->saveTransformToFile(transformFilepath_);
}

void PclProject::addCloud(PointCloud *cloud)
{
    if (!currentCloud_) mainWindow_->setCloudAvailableSettings(true);

    currentCloud_ = cloud;
    visualizer_->addPointCloud(cloud);

    clouds_.push_back(cloud);
    controlBar_->insertAvailableCloud(cloud->name());

    // update
    this->publishActiveCloud();
}

void PclProject::unloadCloud()
{
    if (!currentCloud_) return;

    visualizer_->hidePointCloud(currentCloud_);

    int cloudIndex = clouds_.indexOf(currentCloud_);
    clouds_.remove(cloudIndex);
    controlBar_->removeAvailableCloud(cloudIndex);

    if (clouds_.isEmpty()) {    // no cloud left
        delete currentCloud_;
        currentCloud_ = NULL;
        mainWindow_->setCloudAvailableSettings(false);
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
    controlBar_->removeAvailableClouds();

    currentCloud_ = NULL;
    mainWindow_->setCloudAvailableSettings(false);

    // update
    this->publishActiveCloud();
}

void PclProject::setActiveCloud(int index)
{
    if (clouds_.isEmpty()) return;
    if (index>=clouds_.size() || index<0) return;

    currentCloud_ = clouds_.at(index);

    bool isHidden = (!visualizer_->contains(currentCloud_->name()));
    controlBar_->updateHideActiveCloud(isHidden);

    // update
    this->publishActiveCloud();
}

void PclProject::applyVoxelGrid(float voxelDistance)
{
    if (!currentCloud_) return;
    currentCloud_->applyVoxelGrid(voxelDistance);
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
    this->setDestinationClouds();

    controlBar_->updateCloudTranslation(0.0, 0.0, 0.0);
    controlBar_->updateCloudRotation(0.0, 0.0, 0.0);

    if (!currentCloud_) return;
    QVector3D tvec = currentCloud_->currentTranslation();
    QVector3D rvec = currentCloud_->currentRotation();
    controlBar_->updateCloudTranslation(tvec.x(), tvec.y(), tvec.z());
    controlBar_->updateCloudRotation(rvec.x(), rvec.y(), rvec.z());
}

void PclProject::setDestinationClouds()
{
    QStringList cloudList;
    QVector<PointCloud*>::iterator i;
    for (i = clouds_.begin(); i != clouds_.end(); ++i) {
        if((*i)->name() != currentCloud_->name()) {
            cloudList << (*i)->name();
        }
    }
    controlBar_->updateDestinationClouds(cloudList);
}

void PclProject::hideActiveCloud(bool checked)
{
    if (!currentCloud_) return;
    if (checked) visualizer_->hidePointCloud(currentCloud_);
    else visualizer_->showPointCloud(currentCloud_);
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

void PclProject::showControlBar()
{
    if (controlBar_->isHidden()) {
        QRect screenSize = QApplication::desktop()->screenGeometry();
        double x = screenSize.width() - controlBar_->width();
        double y = screenSize.height() - controlBar_->height();
        controlBar_->move(x, y);
        controlBar_->show();
    }
}

/******************************** ControlBar ********************************/

void PclProject::showCos(double scale)
{
    if (scale > 0) visualizer_->showCoordinateSystem(scale);
    else this->hideCos();
}

void PclProject::hideCos()
{
    visualizer_->hideCoordinateSystem();
}

void PclProject::showHud()
{
    visualizer_->showHeadUpDisplay();
    visualizer_->updateHeadUpDisplay(currentCloud_);
}

void PclProject::hideHud()
{
    visualizer_->hideHeadUpDisplay();
}

void PclProject::resetCamera()
{
    visualizer_->setCameraPosition(50, 50, 50);
}

void PclProject::setCloudPose(double posX, double posY, double posZ, double rotX, double rotY, double rotZ)
{
    if (!currentCloud_) return;
    currentCloud_->setPose(posX, posY, posZ, rotX, rotY, rotZ);
}

void PclProject::removeCloudOutliers(int neighbors, double deviation)
{
    if (!currentCloud_) return;
    currentCloud_->removeStatisticalOutliers(neighbors, deviation);
}

void PclProject::alignToCloud(QString cloudName)
{
    QVector<PointCloud*>::iterator i;
    for (i = clouds_.begin(); i != clouds_.end(); ++i) {
        if((*i)->name() == cloudName) {
            currentCloud_->alignToCloud((*i));
            break;
        }
    }

    QVector3D tvec = currentCloud_->currentTranslation();
    QVector3D rvec = currentCloud_->currentRotation();
    controlBar_->updateCloudTranslation(tvec.x(), tvec.y(), tvec.z());
    controlBar_->updateCloudRotation(rvec.x(), rvec.y(), rvec.z());
}

void PclProject::appendToCloud(QString cloudName)
{
    QVector<PointCloud*>::iterator i;
    for (i = clouds_.begin(); i != clouds_.end(); ++i) {
        if((*i)->name() == cloudName) {
            (*i)->appendCloud(currentCloud_);
            this->unloadCloud();
            currentCloud_ = (*i);
            break;
        }
    }
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

void PclProject::setCropBoxSize(double size)
{
    QVector3D boxSize(size, size, size);

    cropBox_->setSize(boxSize);

    if (!cropBoxEnabled_) return;
    visualizer_->removeCropBox(cropBox_);
    visualizer_->addCropBox(cropBox_);
}

void PclProject::setCropBoxMovementFactor(double factor)
{
    mainWindow_->setCropBoxMovementFactor(factor);
}
