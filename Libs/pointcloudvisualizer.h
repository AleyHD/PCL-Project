#ifndef POINTCLOUDVISUALIZER_H
#define POINTCLOUDVISUALIZER_H

#include <QObject>
#include <QMap>
#include <QVector2D>

#include "pointcloud.h"
#include "cropbox.h"

// Point Cloud Library (PCL)
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>

// Visualization Toolkit (VTK)
#include <QVTKWidget.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>

typedef pcl::PointXYZRGB PointT;
//typedef pcl::PointXYZ PointT;

class PointCloudVisualizer : public QObject
{
    Q_OBJECT

public:
    explicit PointCloudVisualizer(QVTKWidget *visualizerWidget, QObject *parent = 0);

    void addPointCloud(PointCloud* cloud);
    void addPointCloud(PointCloud* cloud, int red, int green, int blue);
    void addCropBox(CropBox* cropBox, double red = 1.0, double green = 1.0, double blue = 1.0);
    void removeCropBox(CropBox* cropBox);
    void removePointCloud(PointCloud* cloud);
    void removeAllPointClouds();
    void removeAllShapes();
    void setBackgroundColor(const double red, const double green, const double blue);
    void setCameraPosition(double posX, double posY, double posZ);
    void setCameraZoom(double zoomLevel);
    void setCloudResolution(PointCloud *cloud, float resolution);
    void showCoordinateSystem(double scale = 1.0);
    void hideCoordinateSystem();
    void showHeadUpDisplay();
    void updateHeadUpDisplay(PointCloud* cloud = NULL);
    void hideHeadUpDisplay();

public slots:
    void updateCloud();
    void updateCloud(PointCloud* cloud);
    void updateWidget();

private slots:
    void updateShapePose();
    void insertText(const QString text, int posX, int posY, const QString id ="");

private:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer_;
    QVTKWidget *visualizerWidget_;
    QMap<QString, QVector2D> texts_;    // id, posX/posY
    std::string coordinateSystemId_;
    bool headUpDisplayEnabled_;
    bool coordinateSystemEnabled_;
};

#endif // POINTCLOUDVISUALIZER_H
