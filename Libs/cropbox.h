#ifndef CROPBOX_H
#define CROPBOX_H

#include <QObject>
#include <QVector3D>
#include "pointcloud.h"

// Point Cloud Library (PCL)
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>

typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;

class CropBox : public QObject
{
    Q_OBJECT

public:
    enum Axis { AxisX, AxisY, AxisZ };

public:
    explicit CropBox(QVector3D size, QString name, QObject *parent = 0);
    explicit CropBox(QObject *parent = 0);
    void translate(float x, float y, float z);
    void setTranslation(float x, float y, float z);
    void rotateDegree(Axis axis, float theta);
    void setRotationDegree(Axis axis, float theta);
    void setSize(QVector3D size);
    void cropCloud();

    void setActiveCloud(PointCloud *cloud);
    void trackInvolvedPoints(bool decision) { trackInvolvedPoints_ = decision; }

    QVector3D size() { return size_; }
    QString name() { return name_; }
    Eigen::Affine3f transformMatrix() { return transform_; }
    PointCloud* involvedPoints() { return involvedPoints_; }

signals:
    void transformApplied(CropBox*);
    void cloudCropped(PointCloud*);
    void sizeChanged();

public slots:

private:
    void updateInvolvedPoints();
    void translatePclCropBox(float x, float y, float z);
    void rotatePclCropBox(Axis axis, float theta);

private:
    bool trackInvolvedPoints_;
    pcl::CropBox<PointT> cropBox_;
    PointCloud* activeCloud_ = NULL;
    PointCloud* involvedPoints_ = NULL;
    QVector<float> currentTranslation_;
    QVector<float> currentRotation_;
    QString name_;
    QVector3D size_;
    Eigen::Affine3f transform_;
};

#endif // CROPBOX_H
