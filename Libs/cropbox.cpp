#include "cropbox.h"

CropBox::CropBox(QVector3D size, QString name, QObject *parent) : QObject(parent)
{
    Eigen::Vector4f vMin, vMax;

    vMin << 0.0, 0.0, 0.0, 0.0;
    vMax << size.x(), size.y(), size.z(), 0.0;

    cropBox_.setMin(vMin);
    cropBox_.setMax(vMax);

    size_ = size;
    name_ = name;

    currentTranslation_.fill(0, 3);     // fill with zeros
    currentRotation_.fill(0, 3);        // fill with zeros
    transform_ = cropBox_.getTransform().inverse();

    trackInvolvedPoints_ = false;
}


CropBox::CropBox(QObject *parent) : QObject(parent)
{
    Eigen::Vector4f vMin, vMax;

    vMin << 0.0, 0.0, 0.0, 0.0;
    vMax << 10.0, 10.0, 10.0, 0.0;

    cropBox_.setMin(vMin);
    cropBox_.setMax(vMax);

    size_ = QVector3D(vMax(0), vMax(1), vMax(2));

    name_ = "cropbox";

    currentTranslation_.fill(0, 3);     // fill with zeros
    currentRotation_.fill(0, 3);        // fill with zeros
    transform_ = cropBox_.getTransform().inverse();

    trackInvolvedPoints_ = false;
}

void CropBox::translate(float x, float y, float z)
{
    float rel[3] = {x, y, z};
    // update abs transform
    for (int i=0; i<currentTranslation_.size(); ++i) {
        currentTranslation_.replace(i, currentTranslation_.at(i)+rel[i]);
    }

    // manipulate cropbox
    this->translatePclCropBox(rel[0], rel[1], rel[2]);
}

void CropBox::setTranslation(float x, float y, float z)
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
    // manipulate cropbox
    this->translatePclCropBox(rel[0], rel[1], rel[2]);
}

void CropBox::rotateDegree(CropBox::Axis axis, float theta)
{
    // update abs rotation
    currentRotation_.replace(axis, currentRotation_.at(axis)+theta);

    // manipulate cropbox
    this->rotatePclCropBox(axis, theta);
}

void CropBox::setRotationDegree(CropBox::Axis axis, float theta)
{
    // calculate relative value
    float rel = theta;
    rel -= currentRotation_.at(axis);

    // update absolute value
    currentRotation_.replace(axis, theta);

    // manipulate cropbox
    this->rotatePclCropBox(axis, rel);
}

void CropBox::setSize(QVector3D size)
{
    Eigen::Vector4f vMax;

    vMax << size.x(), size.y(), size.z(), 0.0;

    cropBox_.setMax(vMax);

    size_ = size;

    emit sizeChanged();
}

void CropBox::cropCloud()
{
    // error check
    if (!activeCloud_) return;

    // filter cloud
    cropBox_.setNegative(true);
    cropBox_.filter(*(activeCloud_->pclCloud()));

    // publish changes
    emit cloudCropped(activeCloud_);
}

void CropBox::setActiveCloud(PointCloud *cloud) {
    // check for valid cloud pointer
    if (!cloud) {
        activeCloud_ = NULL;
        return;
    }
    cropBox_.setInputCloud(cloud->pclCloud());
    activeCloud_ = cloud;
}

void CropBox::updateInvolvedPoints()
{
    pcl::PointCloud<PointT>::Ptr points;
    cropBox_.setNegative(false);
    cropBox_.filter(*points);
    involvedPoints_->setPclCloud(points);
}

void CropBox::translatePclCropBox(float x, float y, float z)
{
    // get transform matrix
    transform_ = cropBox_.getTransform().inverse();

    // Add the new translation of x, y, z in meters.
    Eigen::Vector3f vector(x, y, z);
    transform_.translate(vector);

    // set transform matrix
    cropBox_.setTransform(transform_.inverse());

    if (trackInvolvedPoints_) updateInvolvedPoints();

    // publish changes
    emit transformApplied(this);
}

void CropBox::rotatePclCropBox(CropBox::Axis axis, float theta)
{
    // get transform matrix
    transform_ = cropBox_.getTransform().inverse();

    // theta from degree in radians
    theta = (theta/180)*M_PI;

    switch (axis) {
        case AxisX:
            transform_.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
            break;
        case AxisY:
            transform_.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
            break;
        case AxisZ:
            transform_.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
            break;
    }

    // set transform matrix
    cropBox_.setTransform(transform_.inverse());

    if (trackInvolvedPoints_) updateInvolvedPoints();

    // publish changes
    emit transformApplied(this);
}
