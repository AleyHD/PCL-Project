#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->setWindowTitle ("Visualization");

    // setup event filter for key presses
    qApp->installEventFilter(this);

    // set initiale values
    cropBoxMovementFactor_ = 1.0;
    voxelLeafsize_ = 0.1;
    statisticalNeighbors_ = 50;
    statisticalDeviation_ = 1.0;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setVisualizerWidget(QVTKWidget *widget)
{
    ui->verticalLayout->addWidget(widget);
}

void MainWindow::showMessageOnStatusBar(QString &message, int timeout)
{
    ui->statusBar->showMessage(message, timeout);
}

void MainWindow::setCloudAvailableSettings(bool decision)
{
    ui->action_fileSaveCloud->setEnabled(decision);
    ui->action_fileSaveCloudAs->setEnabled(decision);
    ui->action_filterStatisticalOutlierRemoval->setEnabled(decision);
    ui->action_filterVoxelGrid->setEnabled(decision);
}

void MainWindow::on_action_fileLoadCloud_triggered()
{
    // setup file dialog
    QString title = "Select Cloud";
    QString path = "/";
    QString filter = tr("PCD (*.pcd)");
    QString filePath = QFileDialog::getOpenFileName(this, title, path, filter);
    // check if filePath is ok
    if (filePath.isEmpty()) return;
    // setup cloud
    emit loadCloud(filePath);
}

void MainWindow::on_action_fileSaveCloud_triggered()
{
    emit saveCloud();
}

void MainWindow::on_action_fileSaveCloudAs_triggered()
{
    // setup file dialog
    QString title = "Select Cloud";
    QString path = "/";
    QString filter = tr("PCD (*.pcd)");
    QString filePath = QFileDialog::getSaveFileName(this,title, path, filter);
    // check if filePath is ok
    if (filePath.isEmpty()) return;
    // set file handle
    QFile file(filePath);
    // setup cloud
    if (!filePath.endsWith(".pcd")) filePath += ".pcd";
    emit saveCloud(filePath);
}

void MainWindow::on_action_fileUnloadCloud_triggered()
{
    emit unloadCloud();
}

void MainWindow::on_action_fileUnloadAllClouds_triggered()
{
    emit unloadAllClouds();
}

void MainWindow::on_action_showControlBar_triggered()
{
    emit showControlBar();
}

void MainWindow::on_action_filterVoxelGrid_triggered()
{
    // setup dialog
    QDialog* dialog = new QDialog(this);
    dialog->setWindowTitle("Voxel Grid");

    // setup widgets
    QLabel* label = new QLabel(dialog);
    label->setText("Voxel Leafsize:");
    QDoubleSpinBox* valueBox = new QDoubleSpinBox(dialog);
    valueBox->setValue(voxelLeafsize_);

    // add layout for widgets
    QHBoxLayout* hLayout = new QHBoxLayout();
    hLayout->addWidget(label);
    hLayout->addWidget(valueBox);

    // setup buttons
    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    connect(buttonBox, SIGNAL(accepted()), dialog, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), dialog, SLOT(reject()));

    // setup dialog layout
    QVBoxLayout* vLayout = new QVBoxLayout();
    vLayout->addLayout(hLayout);
    vLayout->addWidget(buttonBox);

    // show dialog
    dialog->setLayout(vLayout);
    if (dialog->exec() == QDialog::Accepted) {
        voxelLeafsize_ = valueBox->value();
        emit applyVoxelGrid(voxelLeafsize_);
    }
}

void MainWindow::on_action_filterStatisticalOutlierRemoval_triggered()
{
    // setup dialog
    QDialog* dialog = new QDialog(this);
    dialog->setWindowTitle("Outlier Removal");

    // setup widgets
    QLabel* label1 = new QLabel(dialog);
    label1->setText("Neighbors to Analyze:");
    QSpinBox* valueBox1 = new QSpinBox(dialog);
    valueBox1->setValue(statisticalNeighbors_);

    QLabel* label2 = new QLabel(dialog);
    label2->setText("Deviation Multiplier:");
    QDoubleSpinBox* valueBox2 = new QDoubleSpinBox(dialog);
    valueBox2->setValue(statisticalDeviation_);

    // add layout for widgets
    QHBoxLayout* hLayout1 = new QHBoxLayout();
    hLayout1->addWidget(label1);
    hLayout1->addWidget(valueBox1);

    QHBoxLayout* hLayout2 = new QHBoxLayout();
    hLayout2->addWidget(label2);
    hLayout2->addWidget(valueBox2);

    // setup buttons
    QDialogButtonBox* buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);

    connect(buttonBox, SIGNAL(accepted()), dialog, SLOT(accept()));
    connect(buttonBox, SIGNAL(rejected()), dialog, SLOT(reject()));

    // setup dialog layout
    QVBoxLayout* vLayout = new QVBoxLayout();
    vLayout->addLayout(hLayout1);
    vLayout->addLayout(hLayout2);
    vLayout->addWidget(buttonBox);

    // show dialog
    dialog->setLayout(vLayout);
    if (dialog->exec() == QDialog::Accepted) {
        statisticalNeighbors_ = valueBox1->value();
        statisticalDeviation_ = valueBox2->value();
        emit removeCloudOutliers(statisticalNeighbors_, statisticalDeviation_);
    }
}

bool MainWindow::eventFilter(QObject *object, QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

        // cropBox Movement
        double value = 1.0*cropBoxMovementFactor_;
        if(keyEvent->key() == Qt::Key_1) emit rotateCropBox(0.0, 0.0, -value);
        if(keyEvent->key() == Qt::Key_3) emit rotateCropBox(0.0 ,0.0, value);
        if(keyEvent->key() == Qt::Key_4) emit translateCropBox(value, 0.0, 0.0);
        if(keyEvent->key() == Qt::Key_6) emit translateCropBox(-value, 0.0, 0.0);
        if(keyEvent->key() == Qt::Key_8) emit translateCropBox(0.0, -value, 0.0);
        if(keyEvent->key() == Qt::Key_2) emit translateCropBox(0.0, value, 0.0);
        if(keyEvent->key() == Qt::Key_9) emit translateCropBox(0.0, 0.0, value);
        if(keyEvent->key() == Qt::Key_7) emit translateCropBox(0.0, 0.0, -value);
        // cropBox Misc
        if(keyEvent->key() == Qt::Key_Delete) emit cropCloud();
    }

    return QObject::eventFilter(object, event);
}
