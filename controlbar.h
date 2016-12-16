#ifndef CONTROLBAR_H
#define CONTROLBAR_H

#include <QDialog>
#include <QDoubleSpinBox>

namespace Ui {
class ControlBar;
}

class ControlBar : public QDialog
{
    Q_OBJECT

public:
    enum Axis { AxisX, AxisY, AxisZ };

public:
    explicit ControlBar(QWidget *parent = 0);
    ~ControlBar();

signals:

    // visualizer
    void showCos(double);
    void hideCos();
    void showHud();
    void hideHud();
    void resetCamera();

    // cloud
    void setActiveCloud(QString);
    void setCloudTranslation(double, double, double);
    void setCloudRotation(Axis, double);


private slots:
    // visualizer
    void on_checkBox_enableCos_clicked(bool checked);
    void on_doubleSpinBox_visualizerCosScale_valueChanged(double arg1) { emit showCos(arg1); }
    void on_checkBox_visualizerShowHeadUpDisplay_clicked(bool checked);
    void on_pushButton_visualizerResetCamera_clicked() { emit resetCamera(); }

    // cloud
    void on_comboBox_cloudActiveCloud_activated(const QString &arg1) { emit setActiveCloud(arg1);}
    void on_doubleSpinBox_cloudTranslationX_valueChanged(double arg1);
    void on_doubleSpinBox_cloudTranslationY_valueChanged(double arg1);
    void on_doubleSpinBox_cloudTranslationZ_valueChanged(double arg1);
    void on_doubleSpinBox_cloudRotationX_valueChanged(double arg1);
    void on_doubleSpinBox_cloudRotationY_valueChanged(double arg1);
    void on_doubleSpinBox_cloudRotationZ_valueChanged(double arg1);

    // cropbox
    void on_checkBox_cropBoxEnableCropBox_clicked(bool checked);
    void on_checkBox_cropBoxHighlight_clicked(bool checked);
    void on_doubleSpinBox_cropBoxSizeX_valueChanged(double arg1);
    void on_doubleSpinBox_cropBoxSizeY_valueChanged(double arg1);
    void on_doubleSpinBox_cropBoxMovementFactor_valueChanged(double arg1);

private:
    Ui::ControlBar *ui;
};

#endif // CONTROLBAR_H
