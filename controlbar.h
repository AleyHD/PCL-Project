#ifndef CONTROLBAR_H
#define CONTROLBAR_H

#include <QDialog>
#include <QDoubleSpinBox>
#include <QComboBox>
#include <QCheckBox>
#include <QPushButton>
#include <QStringList>

#include "Libs/pointcloud.h"

namespace Ui {
class ControlBar;
}

class ControlBar : public QDialog
{
    Q_OBJECT

public:
    explicit ControlBar(QWidget *parent = 0);
    ~ControlBar();

    void updateDestinationClouds(QStringList clouds);
    void updateHideActiveCloud(bool checked);
    void insertAvailableCloud(QString cloud);
    void removeAvailableCloud(int index);
    void updateAvailableCloud(int index, QString cloud);
    void removeAvailableClouds();
    void updateCloudTranslation(double x, double y, double z);
    void updateCloudRotation(double angleX, double angleY, double angleZ);

signals:

    // visualizer
    void showCos(double);
    void hideCos();
    void showHud();
    void hideHud();
    void resetCamera();

    // cloud
    void setActiveCloud(int);
    void hideActiveCloud(bool);
    void setCloudPose(double, double, double, double, double, double);
    void alignToCloud(QString, int, double);
    void appendToCloud(QString);

    // cropbox
    void enableCropBox();
    void disableCropBox();
    void setCropBoxSize(double);
    void setCropBoxMovementFactor(double);

private slots:
    // visualizer
    void on_checkBox_enableCos_clicked(bool checked);
    void on_doubleSpinBox_visualizerCosScale_valueChanged(double arg1) { emit showCos(arg1); }
    void on_checkBox_visualizerShowHeadUpDisplay_clicked(bool checked);
    void on_pushButton_visualizerResetCamera_clicked() { emit resetCamera(); }

    // cloud
    void on_checkBox_cloudHideCloud_clicked(bool checked) { hideActiveCloud(checked); }
    void on_pushButton_cloudMoveCloud_clicked();
    void on_comboBox_cloudSetActiveCloud_currentIndexChanged(int index) { emit setActiveCloud(index); }
    void on_pushButton_cloudAlign_clicked();
    void on_pushButton_cloudAppend_clicked();

    // cropbox
    void on_checkBox_cropBoxEnableCropBox_clicked(bool checked);
    void on_doubleSpinBox_cropBoxSize_valueChanged(double arg1) { emit setCropBoxSize(arg1); }
    void on_doubleSpinBox_cropBoxMovementFactor_valueChanged(double arg1) { emit setCropBoxMovementFactor(arg1); }

private:
    Ui::ControlBar *ui;
};

#endif // CONTROLBAR_H
