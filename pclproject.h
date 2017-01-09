#ifndef PCLPROJECT_H
#define PCLPROJECT_H

#include <QObject>
#include <QVector>
#include <QStringList>

#include "mainwindow.h"
#include "controlbar.h"
#include "Libs/pointcloud.h"
#include "Libs/pointcloudvisualizer.h"
#include "Libs/cropbox.h"

class PclProject : public QObject
{
    Q_OBJECT
public:
    explicit PclProject(QObject *parent = 0);
   ~PclProject();

signals:

public slots:
    // mainwindow
    void loadCloud(QString filePath);
    void saveCloud(QString filePath);
    void unloadCloud();
    void unloadAllClouds();
    void setActiveCloud(int index);
    void useNextActiveCloud();
    void usePreviousActiveCloud();

    void showControlBar();

    void translateCropBox(double x, double y, double z);
    void cropCloud();

    // controlbar
    void showCos(double scale);
    void hideCos();
    void showHud();
    void hideHud();
    void resetCamera();

    void setCloudPose(double posX, double posY, double posZ, double rotX, double rotY, double rotZ);
    void removeCloudOutliers(int neighbors, double deviation);
    void alignToCloud(QString cloudName);
    void appendToCloud(QString cloudName);

    void enableCropBox();
    void disableCropBox();
    void setCropBoxSize(double min, double max);
    void setCropBoxMovementFactor(double factor);

private:
    void setupMainWindow();
    void setupControlBar();
    void setupVisualizer();
    void setupCropBox();

    void addCloud(PointCloud* cloud);
    void publishActiveCloud();
    void setCurrentCloud(PointCloud* cloud);
    void setDestinationClouds();

private:
    MainWindow* mainWindow_;
    ControlBar* controlBar_;
    PointCloud* currentCloud_ = NULL;
    QVector<PointCloud*> clouds_;
    PointCloudVisualizer *visualizer_ = NULL;
    QVTKWidget* visualizerWidget_ = NULL;
    CropBox *cropBox_ = NULL;
    bool cropBoxEnabled_;
};

#endif // PCLPROJECT_H
