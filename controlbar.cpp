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

void ControlBar::updateHideActiveCloud(bool checked)
{
    ui->checkBox_cloudHideCloud->setChecked(checked);
}

void ControlBar::insertAvailableCloud(QString cloud)
{
    ui->comboBox_cloudSetActiveCloud->addItem(cloud);
    ui->comboBox_cloudSetActiveCloud->setCurrentIndex(ui->comboBox_cloudSetActiveCloud->count()-1);
}

void ControlBar::removeAvailableCloud(int index)
{
    ui->comboBox_cloudSetActiveCloud->removeItem(index);
}

void ControlBar::removeAvailableClouds()
{
    ui->comboBox_cloudSetActiveCloud->clear();
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

void ControlBar::on_pushButton_cloudMoveCloud_clicked()
{
    double posX, posY, posZ, rotX, rotY, rotZ;

    posX = ui->doubleSpinBox_cloudTranslationX->value();
    posY = ui->doubleSpinBox_cloudTranslationY->value();
    posZ = ui->doubleSpinBox_cloudTranslationZ->value();

    rotX = ui->doubleSpinBox_cloudRotationX->value();
    rotY = ui->doubleSpinBox_cloudRotationY->value();
    rotZ = ui->doubleSpinBox_cloudRotationZ->value();

    emit setCloudPose(posX, posY, posZ, rotX, rotY, rotZ);
}

void ControlBar::on_pushButton_cloudRemoveOutliers_clicked()
{
    int neighbors = ui->spinBox_cloudRemoveOutliersNeighbors->value();
    double deviation = ui->doubleSpinBox_cloudRemoveOutliersDeviation->value();

    emit removeCloudOutliers(neighbors, deviation);
}

void ControlBar::on_pushButton_cloudVoxelGrid_clicked()
{
    float leafsize = ui->doubleSpinBox_cloudVoxelGrid->value();
    emit applyVoxelGrid(leafsize);
}
