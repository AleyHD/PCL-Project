#include "pointcloudvisualizer.h"

PointCloudVisualizer::PointCloudVisualizer(QVTKWidget *visualizerWidget, QObject *parent) : QObject(parent)
{
    // Set up the QVTK widget
    visualizer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    visualizerWidget_ = visualizerWidget;
    visualizerWidget_->SetRenderWindow(visualizer_->getRenderWindow());
    visualizer_->setupInteractor(visualizerWidget_->GetInteractor(), visualizerWidget_->GetRenderWindow());

    // initialize camera
    visualizer_->initCameraParameters();

    // disable show FPS
    visualizer_->setShowFPS(false) ;

    // initialize members
    coordinateSystemId_ = "cos";
    coordinateSystemEnabled_ = false;
    headUpDisplayEnabled_ = false;

    this->updateWidget();
}

void PointCloudVisualizer::addPointCloud(PointCloud *cloud)
{
    // get PCL Cloud & Cloud ID
    pcl::PointCloud<PointT>::Ptr pclCloud = cloud->pclCloud();
    std::string id = cloud->name().toStdString();

    // add cloud
    if (!visualizer_->addPointCloud(pclCloud, id)) return;

    // connect signals to slots
    connect(cloud, SIGNAL(updated()), this, SLOT(updateCloud()));
    this->updateWidget();
}

void PointCloudVisualizer::addPointCloud(PointCloud *cloud, int red, int green, int blue)
{
    // get PCL Cloud & Cloud ID
    pcl::PointCloud<PointT>::Ptr pclCloud = cloud->pclCloud();
    std::string id = cloud->name().toStdString();

    // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler(pclCloud, red, green, blue);

    // add cloud
    visualizer_->addPointCloud(pclCloud, color_handler, id);

    // connect signals to slots
    connect(cloud, SIGNAL(updated()), this, SLOT(updateCloud()));

    // update
    this->updateWidget();
}

void PointCloudVisualizer::addCropBox(CropBox* cropBox, double red, double green, double blue)
{
    // get parameters
    QVector3D pMin = cropBox->pMin();
    QVector3D pMax = cropBox->pMax();
    std::string name = cropBox->name().toStdString();

    // setup box
    visualizer_->addCube(pMin.x(),pMax.x(),pMin.y(),pMax.y(),pMin.z(),pMax.z(), red, green, blue, name);
    this->updateCropBoxPose(cropBox);

    // set to wireframe
    visualizer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                             pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                                             name);

    // connect signals to slots
    connect(cropBox, SIGNAL(transformApplied(CropBox*)), this, SLOT(updateCropBoxPose(CropBox*)));
    connect(cropBox, SIGNAL(cloudCropped(PointCloud*)), this, SLOT(updateCloud(PointCloud*)));

    // update
    this->updateWidget();
}

void PointCloudVisualizer::removePointCloud(PointCloud *cloud)
{
    visualizer_->removePointCloud(cloud->name().toStdString());
    this->updateWidget();
}

void PointCloudVisualizer::removeAllPointClouds()
{
    visualizer_->removeAllPointClouds();
    this->updateWidget();
}

void PointCloudVisualizer::removeCropBox(CropBox *cropBox)
{
    visualizer_->removeShape(cropBox->name().toStdString());
    this->updateWidget();
}

void PointCloudVisualizer::removeAllShapes()
{
    visualizer_->removeAllShapes();
    this->updateWidget();
}

void PointCloudVisualizer::updateCropBoxPose(CropBox* cropBox)
{
    visualizer_->updateShapePose(cropBox->name().toStdString(), cropBox->transformMatrix());

    // update
    this->updateWidget();
}

void PointCloudVisualizer::setBackgroundColor(const double red, const double green, const double blue)
{
    // Setting background color in RGB between 0.0 & 1.0
    visualizer_->setBackgroundColor(red, green, blue);
    this->updateWidget();
}

void PointCloudVisualizer::setCameraPosition(double posX, double posY, double posZ)
{
    double viewX = 0, viewY = 0, viewZ = 0;
    double upX = 0, upY = 0, upZ = 1;
    //visualizer_->getRenderWindow()->GetRenderers()->GetFirstRenderer()->GetActiveCamera()->SetParallelProjection(true);
    visualizer_->setCameraPosition(posX,posY,posZ,viewX,viewY,viewZ,upX,upY,upZ);
    this->updateWidget();
}

void PointCloudVisualizer::setCameraZoom(double zoomLevel)
{
    visualizer_->setCameraPosition(zoomLevel,zoomLevel,zoomLevel,0,0,0,0,0,1);
    this->updateWidget();
}

void PointCloudVisualizer::showCoordinateSystem(double scale)
{
    if (coordinateSystemEnabled_) this->hideCoordinateSystem();
    visualizer_->addCoordinateSystem(scale, coordinateSystemId_, 0);
    coordinateSystemEnabled_ = true;
    this->updateWidget();
}

void PointCloudVisualizer::hideCoordinateSystem()
{
    visualizer_->removeCoordinateSystem(coordinateSystemId_);
    coordinateSystemEnabled_ = false;
    this->updateWidget();
}

void PointCloudVisualizer::showHeadUpDisplay()
{
    QString id, text;
    size_t posY = 10;

    // active cloud
    text = "Active Cloud: ";
    id = text;
    this->insertText(text, 10, posY, id);
    posY += 20;

    // 3D Points
    text = "3D Points: ";
    id = text;
    this->insertText(text, 10, posY, id);
    posY += 20;

    headUpDisplayEnabled_ = true;
}

void PointCloudVisualizer::updateHeadUpDisplay(PointCloud *cloud)
{
    if (!headUpDisplayEnabled_) return;

    QString id, text;
    QVector2D pos;

    // name
    id = "Active Cloud: ";
    text = id;
    if (!cloud) text += "None";
    else text += cloud->name();
    pos = texts_.value(id);
    this->insertText(text, pos.x(), pos.y(), id);

    // points
    id = "3D Points: ";
    text = id;
    if (!cloud) text += "0";
    else text += QString::number(cloud->points());
    pos = texts_.value(id);
    this->insertText(text, pos.x(), pos.y(), id);
}

void PointCloudVisualizer::hideHeadUpDisplay()
{
    QMap<QString, QVector2D>::iterator i;
    for (i = texts_.begin(); i != texts_.end(); ++i) {
        visualizer_->updateText(" ", i.value().x(), i.value().y(), i.key().toStdString());
    }
    headUpDisplayEnabled_ = false;

    this->updateWidget();
}

void PointCloudVisualizer::insertText(const QString text, int posX, int posY, const QString id)
{
    QVector2D position(posX, posY);
    if (!texts_.contains(id)) {
        texts_.insert(id, position);
        visualizer_->addText(text.toStdString(), posX, posY, 16, 1.0, 1.0, 1.0, id.toStdString());
    } else {
        visualizer_->updateText(text.toStdString(), posX, posY, id.toStdString());
    }

    this->updateWidget();
}

void PointCloudVisualizer::updateWidget()
{
    visualizerWidget_->update();
}

void PointCloudVisualizer::updateCloud()
{
    // identify sender
    PointCloud* cloud = (PointCloud*)sender();

    // get PCL Cloud & Cloud ID
    pcl::PointCloud<PointT>::Ptr pclCloud = cloud->pclCloud();
    std::string id = cloud->name().toStdString();

    // execute update
    this->updateHeadUpDisplay(cloud);
    visualizer_->updatePointCloud(pclCloud, id);
    visualizerWidget_->update();
}

void PointCloudVisualizer::updateCloud(PointCloud* cloud)
{
    // get PCL Cloud & Cloud ID
    pcl::PointCloud<PointT>::Ptr pclCloud = cloud->pclCloud();
    std::string id = cloud->name().toStdString();

    // execute update
    this->updateHeadUpDisplay(cloud);
    visualizer_->updatePointCloud(pclCloud, id);
    visualizerWidget_->update();
}

void PointCloudVisualizer::setCloudResolution(PointCloud *cloud, float resolution)
{
    // downsample the dataset to a desired minimal point distance (resolution)
/*
    // initialize variables
    pcl::PointCloud<PointT>::Ptr downsampledCloud(new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> voxelGrid;

    // apply downsampling
    voxelGrid.setInputCloud(cloud->pclCloud());
    voxelGrid.setLeafSize(resolution, resolution, resolution);
    voxelGrid.filter(*downsampledCloud);

    // save downsampled cloud

    pointCloudMap_.insert(cloud->name(), downsampledCloud);*/
}
