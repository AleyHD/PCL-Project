#include "controlbar.h"
#include "ui_controlbar.h"

ControlBar::ControlBar(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ControlBar)
{
    ui->setupUi(this);

    this->setWindowTitle ("Control Bar");
}

ControlBar::~ControlBar()
{
    delete ui;
}

void ControlBar::on_checkBox_enableCos_clicked(bool checked)
{
    if (checked) emit showCos(ui->doubleSpinBox_visualizerCosScale->value());
    else emit hideCos();
}

void ControlBar::on_checkBox_visualizerShowHeadUpDisplay_clicked(bool checked)
{
    if (checked) emit showHud();
    else emit hideHud();
}

void ControlBar::on_doubleSpinBox_cloudTranslationX_valueChanged(double arg1)
{
    double x, y, z;
    x = arg1;
    y = ui->doubleSpinBox_cloudTranslationY->value();
    z = ui->doubleSpinBox_cloudTranslationZ->value();
    emit setCloudTranslation(x, y, z);
}

void ControlBar::on_doubleSpinBox_cloudTranslationY_valueChanged(double arg1)
{
    double x, y, z;
    x = ui->doubleSpinBox_cloudTranslationX->value();
    y = arg1;
    z = ui->doubleSpinBox_cloudTranslationZ->value();
    emit setCloudTranslation(x, y, z);
}

void ControlBar::on_doubleSpinBox_cloudTranslationZ_valueChanged(double arg1)
{
    double x, y, z;
    x = ui->doubleSpinBox_cloudTranslationX->value();
    y = ui->doubleSpinBox_cloudTranslationY->value();
    z = arg1;
    emit setCloudTranslation(x, y, z);
}

void ControlBar::on_doubleSpinBox_cloudRotationX_valueChanged(double arg1)
{
    Axis axis = AxisX;
    emit setCloudRotation(axis, arg1);
}

void ControlBar::on_doubleSpinBox_cloudRotationY_valueChanged(double arg1)
{
    Axis axis = AxisY;
    emit setCloudRotation(axis, arg1);
}

void ControlBar::on_doubleSpinBox_cloudRotationZ_valueChanged(double arg1)
{
    Axis axis = AxisZ;
    emit setCloudRotation(axis, arg1);
}

void ControlBar::on_checkBox_cropBoxEnableCropBox_clicked(bool checked)
{

}

void ControlBar::on_checkBox_cropBoxHighlight_clicked(bool checked)
{

}

void ControlBar::on_doubleSpinBox_cropBoxSizeX_valueChanged(double arg1)
{

}

void ControlBar::on_doubleSpinBox_cropBoxSizeY_valueChanged(double arg1)
{

}

void ControlBar::on_doubleSpinBox_cropBoxMovementFactor_valueChanged(double arg1)
{

}
