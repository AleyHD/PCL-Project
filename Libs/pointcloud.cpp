#include "pointcloud.h"

PointCloud::PointCloud(QString name, QObject *parent) :
    QObject(parent),
    cloud_(new pcl::PointCloud<PointT>)
{
    name_ = name;
}

void PointCloud::loadFile(const QString &filePath)
{
    pcl::io::loadPCDFile<PointT>(filePath.toStdString(), *cloud_);

    // publish changes
    emit updated();
}

void PointCloud::saveFile(const QString &filePath)
{

    pcl::io::savePCDFileASCII(filePath.toStdString(), *cloud_);
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

void PointCloud::setResolution(float resolution)
{
    pcl::VoxelGrid<PointT> voxelGrid;

    // apply downsampling
    voxelGrid.setInputCloud(cloud_);
    voxelGrid.setLeafSize(resolution, resolution, resolution);
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

void PointCloud::extractPlane(PointCloud *outputPlane, bool cut, int maxIterations, double threshold)
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
    seg.setMaxIterations(10);
    seg.setDistanceThreshold(1);

    // indices extraction filter
    pcl::ExtractIndices<PointT> extract;

    pcl::PointCloud<PointT>::Ptr cloud_p (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
    pcl::PointCloud<PointT>::Ptr remainingCloud (new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud_, *remainingCloud);
    int i = 0, nr_points = (int) cloud_->points.size ();

    // While 30% of the original cloud is still there
    while (remainingCloud->points.size () > 0.3 * nr_points) {
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
