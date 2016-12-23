#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QVector>
#include <QFileDialog>
#include <QFileDialog>
#include <QKeyEvent>

#include "controlbar.h"
#include "Libs/pointcloud.h"
#include "Libs/pointcloudvisualizer.h"
#include "Libs/cropbox.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

protected:
    bool eventFilter(QObject *object, QEvent *event);

private slots:

    void resetCamera();

    void on_action_loadCloud_triggered();
    void on_action_saveCloud_triggered();
    void on_action_unloadCloud_triggered();
    void on_action_unloadAllClouds_triggered();

    void on_action_showControlBar_triggered();

private:
    void setupVisualizer();
    void setupCloud(QString &filePath);
    void addCloud(PointCloud* cloud);
    void delCloud();
    void delClouds();
    void changeActiveCloud();
    void setupCropBox();
    void enableCropBox();
    void disableCropBox();
    void extractCloud();

private:
    Ui::MainWindow *ui;
    ControlBar* controlBar_;
    PointCloud* currentCloud_ = NULL;
    QVector<PointCloud*> clouds_;
    PointCloudVisualizer *visualizer_ = NULL;
    CropBox *cropBox_ = NULL;
    bool cropBoxEnabled_;

};

#endif // MAINWINDOW_H
