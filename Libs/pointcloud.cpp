#include "pointcloud.h"

PointCloud::PointCloud(QString name, QObject *parent) :
    QObject(parent),
    cloud_(new pcl::PointCloud<PointT>)
{
    name_ = name;
    currentTranslation_.fill(0, 3);     // fill with zeros
    currentRotation_.fill(0, 3);        // fill with zeros
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
    // save values
    float abs[3] = {x, y, z};
    float rel[3] = {x, y, z};
    for (int i=0; i<currentTranslation_.size(); ++i) {
        // calculate relative value
        rel[i] -= currentTranslation_.at(i);
        // update absolute value
        currentTranslation_.replace(i, abs[i]);
    }
    // manipulate cloud
    this->translatePclCloud(rel[0], rel[1], rel[2]);
}

void PointCloud::translate(float x, float y, float z)
{
    float rel[3] = {x, y, z};
    // update abs transform
    for (int i=0; i<currentTranslation_.size(); ++i) {
        currentTranslation_.replace(i, currentTranslation_.at(i)+rel[i]);
    }
    this->translatePclCloud(rel[0], rel[1], rel[2]);
}

void PointCloud::translateToOrigin()
{
    QVector4D centroid = this->compute3DCentroid();
    this->translate(-centroid.x(), -centroid.y(), -centroid.z());

    // set origin as zero transform
    currentTranslation_.fill(0, 3);    // fill with zeros

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

void PointCloud::setRotationDegree(PointCloud::Axis axis, float theta)
{
    // calculate relative value
    float rel = theta;
    rel -= currentRotation_.at(axis);

    // update absolute value
    currentTranslation_.replace(axis, theta);

    // manipulate cloud
    this->rotatePclCloud(axis, rel);
}

void PointCloud::rotateDegree(Axis axis, float theta)
{
    // update abs rotation
    currentRotation_.replace(axis, currentRotation_.at(axis)+theta);

    // manipulate cloud
    this->rotatePclCloud(axis, theta);
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

bool PointCloud::alignCloud(PointCloud* cloud2Align)
{
    // set parameters
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(cloud2Align->pclCloud());
    icp.setInputTarget(cloud_);

    // align clouds
    //icp.align(*cloud_);

    // status
    double score = icp.getFitnessScore();
    //Eigen::Matrix4f m = icp.getFinalTransformation();
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
