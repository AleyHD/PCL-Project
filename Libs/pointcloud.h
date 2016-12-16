#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QVector>
#include <QVector4D>

// Point Cloud Library (PCL)
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/registration/icp.h>

typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;

class PointCloud : public QObject
{
    Q_OBJECT

public:
    enum Axis { AxisX, AxisY, AxisZ };

public:
    explicit PointCloud(QString name, QObject *parent = 0);
    void loadFile(const QString &filePath);
    void saveFile(const QString &filePath);
    void setTranslation(float x, float y, float z);
    void translate(float x, float y, float z);
    void translateToOrigin();
    void setResolution(float resolution);
    void setRotationDegree(Axis axis, float theta);
    void rotateDegree(Axis axis, float theta);
    void extractPlane(PointCloud *outputPlane, bool cut = true, int maxIterations = 100, double threshold = 1.0);
    QVector4D compute3DCentroid();
    bool alignCloud(PointCloud *cloud2Align);
    void appendCloud(PointCloud *cloud2Add);
    void appendCloudSBS(PointCloud *cloud2AddSBS);

    size_t points();
    pcl::PointCloud<PointT>::Ptr pclCloud() { return cloud_; }
    void setPclCloud(pcl::PointCloud<PointT>::Ptr cloud);
    QString name() { return name_; }


signals:
    void updated();

public slots:

private:
    void translatePclCloud(float x, float y, float z);
    void rotatePclCloud(Axis axis, float theta);

private:
    pcl::PointCloud<PointT>::Ptr cloud_;
    QVector<float> currentTranslation_;
    QVector<float> currentRotation_;
    QString name_;
};

#endif // POINTCLOUD_H
