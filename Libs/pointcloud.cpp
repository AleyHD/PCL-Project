#include "pointcloud.h"

PointCloud::PointCloud(QString name, QObject *parent) :
    QObject(parent),
    cloud_(new pcl::PointCloud<PointT>)
{
    name_ = name;
    fileIO_ = new FileIO(this);
}

void PointCloud::loadFile(const QString &filePath)
{
    pcl::io::loadPCDFile<PointT>(filePath.toStdString(), *cloud_);
    filePath_ = filePath;

    // publish changes
    emit updated();
}

void PointCloud::saveFile(const QString &filePath)
{
    pcl::io::savePCDFileASCII(filePath.toStdString(), *cloud_);

    // get filename without extension
    QFileInfo fileInfo(filePath);
    QString fileName = fileInfo.baseName();

    name_ = fileName;
    filePath_ = filePath;
}

void PointCloud::saveFile()
{
    pcl::io::savePCDFileASCII(filePath_.toStdString(), *cloud_);
}

size_t PointCloud::points()
{
    size_t size = cloud_->width * cloud_->height;
    return size;
}

void PointCloud::setTranslation(float x, float y, float z)
{
    QVector3D abs(x, y, z);
    QVector3D rel(x, y, z);

    // calculate relative value
    rel -= currentTranslation_;

    // update absolute value
    currentTranslation_ = abs;

    // manipulate cloud
    this->translatePclCloud(rel.x(), rel.y(), rel.z());
}

void PointCloud::translate(float x, float y, float z)
{
    QVector3D rel(x, y, z);

    // update abs transform
    currentTranslation_ += rel;

    this->translatePclCloud(rel.x(), rel.y(), rel.z());
}

void PointCloud::translateToOrigin()
{
    QVector4D centroid = this->compute3DCentroid();
    this->translate(-centroid.x(), -centroid.y(), -centroid.z());

    // publish changes
    emit updated();
}

void PointCloud::applyVoxelGrid(float voxelDistance)
{
    pcl::VoxelGrid<PointT> voxelGrid;

    // apply downsampling
    voxelGrid.setInputCloud(cloud_);
    voxelGrid.setLeafSize(voxelDistance, voxelDistance, voxelDistance);
    voxelGrid.filter(*cloud_);

    // publish changes
    emit updated();
}

void PointCloud::setRotationDegree(Axis axis, float angle)
{
    float rel = angle;

    switch (axis) {
        case AxisX:
            rel -= currentRotation_.x();
            currentRotation_.setX(angle);
            break;
        case AxisY:
            rel -= currentRotation_.y();
            currentRotation_.setY(angle);
            break;
        case AxisZ:
            rel -= currentRotation_.z();
            currentRotation_.setZ(angle);
            break;
    }

    // manipulate cloud
    this->rotatePclCloud(axis, rel);
}

void PointCloud::rotateDegree(Axis axis, float angle)
{
    // update abs rotation
    switch (axis) {
        case AxisX:
            currentRotation_.setX(currentRotation_.x() + angle);
            break;
        case AxisY:
            currentRotation_.setY(currentRotation_.y() + angle);
            break;
        case AxisZ:
            currentRotation_.setZ(currentRotation_.y() + angle);
            break;
    }

    // manipulate cloud
    this->rotatePclCloud(axis, angle);
}

void PointCloud::setPose(float posX, float posY, float posZ, float rotX, float rotY, float rotZ)
{
    QVector3D posAbs(posX, posY, posZ);
    QVector3D posRel(posX, posY, posZ);
    QVector3D rotAbs(rotX, rotY, rotZ);
    QVector3D rotRel(rotX, rotY, rotZ);

    // calculate relative value
    posRel -= currentTranslation_;
    rotRel -= currentRotation_;

    // update absolute value
    currentTranslation_ = posAbs;
    currentRotation_ = rotAbs;

    this->transformPclCloud(posRel.x(), posRel.y(), posRel.z(), rotRel.x(), rotRel.y(), rotRel.z());
}

void PointCloud::extractPlane(PointCloud *outputPlane, int maxIterations, double distanceThreshold, double cloudThreshold, bool cut)
{
    // Segment the largest planar component
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // indices extraction filter
    pcl::ExtractIndices<PointT> extract;

    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr remainingCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_, *remainingCloud);
    int i = 0, nr_points = (int) cloud_->points.size ();

    // While 30% of the original cloud is still there
    while (remainingCloud->points.size () > cloudThreshold * nr_points) {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud(remainingCloud);
        // get inliers & coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size () == 0) return;


        extract.setInputCloud(remainingCloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_p);
        outputPlane->setPclCloud(cloud_p);

        // Create the filtering object
        extract.setNegative(true);
        extract.filter(*cloud_f);
        remainingCloud.swap(cloud_f);
    }
    if (cut) cloud_ = remainingCloud;

    // publish changes
    emit updated();
}

void PointCloud::removeStatisticalOutliers(int neighbors, double deviationThreshold)
{
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_);
    sor.setMeanK(neighbors);
    sor.setStddevMulThresh(deviationThreshold);
    sor.filter(*cloud_);

    // publish changes
    emit updated();
}

QVector4D PointCloud::compute3DCentroid()
{
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_, centroid);
    QVector4D qCentroid(centroid(0), centroid(1), centroid(2), centroid(3));
    return qCentroid;
}

bool PointCloud::alignToCloud(PointCloud* cloud)
{
    // set parameters
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setInputSource(cloud_);
    icp.setInputTarget(cloud->pclCloud());

    // align cloud
    icp.align(*cloud_);

    // get transformation
    Eigen::Matrix4f transform4f =  icp.getFinalTransformation();
    Eigen::Affine3f transform(transform4f);
    float x, y, z, angleX, angleY, angleZ;
    pcl::getTranslationAndEulerAngles(transform, x, y, z, angleX, angleY, angleZ);

    // save transformation

    QVector3D translation(x, y, z);
    QVector3D rotation(angleX, angleY, angleZ);

    currentTranslation_ += translation;
    currentRotation_ += rotation;

    // publish changes
    emit updated();

    return icp.hasConverged();
}

void PointCloud::appendCloud(PointCloud *cloud2Add)
{
    *cloud_ += *(cloud2Add->pclCloud());

    // publish changes
    emit updated();
}

void PointCloud::appendCloudSBS(PointCloud *cloud2AddSBS)
{
    pcl::concatenateFields(*cloud_, *(cloud2AddSBS->pclCloud()), *cloud_);

    // publish changes
    emit updated();
}

bool PointCloud::loadTransformFromFile(const QString &filePath)
{
    // read file in QStringList
    QStringList lines = fileIO_->readTextFile(filePath);
    if (lines.isEmpty()) return false;
    // search cloud
    size_t lineIndex = 0;
    QStringList lineElements;
    for (QStringList::iterator it=lines.begin(); it!=lines.end(); ++it) {
        QString line = *it;
        lineElements = line.split(';');
        if (!lineElements.size()) continue;
        if (lineElements.at(0) == name_) break;
        ++lineIndex;
    }
    // check if line is correct
    if (lineIndex == lines.size()) return false;
    if (lineElements.size() != 7) return false;
    // convert to numbers
    QVector3D translation(lineElements.at(1).toDouble(),
                          lineElements.at(2).toDouble(),
                          lineElements.at(3).toDouble());
    QVector3D rotation(lineElements.at(4).toDouble(),
                       lineElements.at(5).toDouble(),
                       lineElements.at(6).toDouble());
    // update values
    currentTranslation_ = translation;
    currentRotation_ = rotation;
    return true;
}

bool PointCloud::saveTransformToFile(const QString &filePath)
{
    // read file in QStringList
    QStringList lines = fileIO_->readTextFile(filePath);
    // search cloud
    size_t lineIndex = 0;
    for (QStringList::iterator it=lines.begin(); it!=lines.end(); ++it) {
        QString line = *it;
        QStringList lineElements = line.split(';');
        if (!lineElements.size()) continue;
        if (lineElements.at(0) == name_) break;
        ++lineIndex;
    }
    // get current transform
    QString transform = this->transformString();
    // add/replace transform
    if (lineIndex < lines.size()) lines[lineIndex] = transform;
    else lines += transform;
    // update file
    return fileIO_->saveTextFile(filePath, lines);
}

QString PointCloud::transformString()
{
    QString translation = QString(";%1;%2;%3").arg(currentTranslation_.x())
                                              .arg(currentTranslation_.y())
                                              .arg(currentTranslation_.z());

    QString rotation = QString(";%1;%2;%3").arg(currentRotation_.x())
                                           .arg(currentRotation_.y())
                                           .arg(currentRotation_.z());

    return name_+translation+rotation;
}

void PointCloud::setPclCloud(pcl::PointCloud<PointT>::Ptr cloud)
{
    cloud_ = cloud;

    // publish changes
    emit updated();
}

void PointCloud::translatePclCloud(float x, float y, float z)
{
    // setup Identity Matrix
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // Define a translation of x, y, z meters.
    transform.translation() << x, y, z;

    // Executing the transformation
    pcl::transformPointCloud(*cloud_, *cloud_, transform);

    // publish changes
    emit updated();
}

void PointCloud::rotatePclCloud(PointCloud::Axis axis, float theta)
{
    // setup Identity Matrix
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    // theta from degree in radians
    theta = (theta/180)*M_PI;

    switch (axis) {
    case AxisX:
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
        break;
    case AxisY:
        transform.rotate(Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitY()));
        break;
    case AxisZ:
        transform.rotate(Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
        break;
    }
    // Executing the transformation
    pcl::transformPointCloud (*cloud_, *cloud_, transform);

    // publish changes
    emit updated();
}

void PointCloud::transformPclCloud(float posX, float posY, float posZ, float rotX, float rotY, float rotZ)
{
    // setup Identity Matrix
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();

    // define translation
    transform.translation() << posX, posY, posZ;

    // define rotation
    transform.rotate(Eigen::AngleAxisf(rotX, Eigen::Vector3f::UnitX()));
    transform.rotate(Eigen::AngleAxisf(rotY, Eigen::Vector3f::UnitY()));
    transform.rotate(Eigen::AngleAxisf(rotZ, Eigen::Vector3f::UnitZ()));

    // apply the transformation
    pcl::transformPointCloud(*cloud_, *cloud_, transform);

    // publish changes
    emit updated();
}
