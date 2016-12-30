#ifndef PCLPROJECT_H
#define PCLPROJECT_H

#include <QObject>

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
    void useNextActiveCloud();
    void usePreviousActiveCloud();

    void showControlBar();

    void translateCropBox(double x, double y, double z);
    void cropCloud();

    // controlbar
    void enableCropBox();
    void disableCropBox();

private:
    void setupMainWindow();
    void setupControlBar();
    void setupVisualizer();
    void setupCropBox();

    void addCloud(PointCloud* cloud);
    void publishActiveCloud();

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
