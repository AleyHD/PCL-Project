#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QLayout>
#include <QDialogButtonBox>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QAction>
#include <QKeyEvent>
#include <QVTKWidget.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setVisualizerWidget(QVTKWidget *widget);
    void showMessageOnStatusBar(QString &message, int timeout);
    void setCloudAvailableSettings(bool decision);
    void setCropBoxMovementFactor(double factor) { cropBoxMovementFactor_ = factor; }

signals:
    void loadCloud(QString);
    void saveCloud(QString);
    void saveCloud();
    void unloadCloud();
    void unloadAllClouds();
    void useNextActiveCloud();
    void usePreviousActiveCloud();

    void showControlBar();

    void applyVoxelGrid(float);
    void removeCloudOutliers(int, double);

    void translateCropBox(double, double, double);
    void rotateCropBox(double, double, double);
    void cropCloud();

protected:
    bool eventFilter(QObject *object, QEvent *event);

private slots:

    void on_action_fileLoadCloud_triggered();
    void on_action_fileSaveCloud_triggered();
    void on_action_fileSaveCloudAs_triggered();
    void on_action_fileUnloadCloud_triggered();
    void on_action_fileUnloadAllClouds_triggered();

    void on_action_showControlBar_triggered();

    void on_action_filterVoxelGrid_triggered();
    void on_action_filterStatisticalOutlierRemoval_triggered();

private:
    Ui::MainWindow *ui;
    double cropBoxMovementFactor_;
    float voxelLeafsize_;
    int statisticalNeighbors_;
    double statisticalDeviation_;
};

#endif // MAINWINDOW_H
