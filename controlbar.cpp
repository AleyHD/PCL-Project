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

void ControlBar::updateDestinationClouds(QStringList clouds)
{
    ui->comboBox_cloudDestination->clear();
    if (!clouds.isEmpty()) ui->comboBox_cloudDestination->addItems(clouds);
}

void ControlBar::updateCloudTranslation(double x, double y, double z)
{
    ui->doubleSpinBox_cloudTranslationX->setValue(x);
    ui->doubleSpinBox_cloudTranslationY->setValue(y);
    ui->doubleSpinBox_cloudTranslationZ->setValue(z);
}

void ControlBar::updateCloudRotation(double angleX, double angleY, double angleZ)
{
    ui->doubleSpinBox_cloudRotationX->setValue(angleX);
    ui->doubleSpinBox_cloudRotationY->setValue(angleY);
    ui->doubleSpinBox_cloudRotationZ->setValue(angleZ);
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
    PointCloud::Axis axis = PointCloud::AxisX;
    emit setCloudRotation(axis, arg1);
}

void ControlBar::on_doubleSpinBox_cloudRotationY_valueChanged(double arg1)
{
    PointCloud::Axis axis = PointCloud::AxisY;
    emit setCloudRotation(axis, arg1);
}

void ControlBar::on_doubleSpinBox_cloudRotationZ_valueChanged(double arg1)
{
    PointCloud::Axis axis = PointCloud::AxisZ;
    emit setCloudRotation(axis, arg1);
}

void ControlBar::on_pushButton_cloudAlign_clicked()
{
    QString cloud = ui->comboBox_cloudDestination->currentText();
    if (cloud.isEmpty()) return;
    emit alignToCloud(cloud);
}

void ControlBar::on_pushButton_cloudAppend_clicked()
{
    QString cloud = ui->comboBox_cloudDestination->currentText();
    if (cloud.isEmpty()) return;
    emit appendToCloud(cloud);
}

void ControlBar::on_checkBox_cropBoxEnableCropBox_clicked(bool checked)
{
    if (checked) emit enableCropBox();
    else emit disableCropBox();
}

void ControlBar::on_checkBox_cropBoxHighlight_clicked(bool checked)
{
    if (checked) emit enableCropBoxHighlight();
    else emit disableCropBoxHighlight();
}

void ControlBar::on_doubleSpinBox_cropBoxSizeX_valueChanged(double arg1)
{
    double x, y;
    x = arg1;
    y = ui->doubleSpinBox_cropBoxSizeY->value();
    emit setCropBoxSize(x, y);
}

void ControlBar::on_doubleSpinBox_cropBoxSizeY_valueChanged(double arg1)
{
    double x, y;
    x = ui->doubleSpinBox_cropBoxSizeX->value();
    y = arg1;
    emit setCropBoxSize(x, y);
}
