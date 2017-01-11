#ifndef POINTCLOUD_H
#define POINTCLOUD_H

#include <QObject>
#include <QVector>
#include <QVector3D>
#include <QVector4D>
#include <QFileInfo>

#include "Libs/fileio.h"

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
#include <pcl/filters/statistical_outlier_removal.h>

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
    void saveFile();
    void setTranslation(float x, float y, float z);
    void translate(float x, float y, float z);
    void translateToOrigin();
    void applyVoxelGrid(float voxelDistance);
    void setRotationDegree(Axis axis, float angle);
    void rotateDegree(Axis axis, float angle);
    void setPose(float posX, float posY, float posZ, float rotX, float rotY, float rotZ);
    void extractPlane(PointCloud *outputPlane, int maxIterations = 100, double distanceThreshold = 1.0, double cloudThreshold = 0.3, bool cut = true);
    void removeStatisticalOutliers(int neighbors = 50, double deviationThreshold = 1.0);
    QVector4D compute3DCentroid();
    bool alignToCloud(PointCloud *cloud);
    void appendCloud(PointCloud *cloud2Add);
    void appendCloudSBS(PointCloud *cloud2AddSBS);
    bool loadTransformFromFile(const QString &filePath);
    bool saveTransformToFile(const QString &filePath);

    size_t points();
    pcl::PointCloud<PointT>::Ptr pclCloud() { return cloud_; }
    void setPclCloud(pcl::PointCloud<PointT>::Ptr cloud);
    QString name() { return name_; }
    QString filePath() { return filePath_; }
    QVector3D currentTranslation() { return currentTranslation_; }
    QVector3D currentRotation() { return currentRotation_; }


signals:
    void updated();

public slots:

private:
    QString transformString();
    void translatePclCloud(float x, float y, float z);
    void rotatePclCloud(Axis axis, float theta);
    void transformPclCloud(float posX, float posY, float posZ, float rotX, float rotY, float rotZ);

private:
    pcl::PointCloud<PointT>::Ptr cloud_;
    QVector3D currentTranslation_;
    QVector3D currentRotation_;
    QString name_;
    QString filePath_;
    FileIO* fileIO_;
};

#endif // POINTCLOUD_H
