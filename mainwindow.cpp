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

void MainWindow::setActionSaveCloudEnabled(bool decision)
{
    ui->action_saveCloud->setEnabled(decision);
}

void MainWindow::on_action_loadCloud_triggered()
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

void MainWindow::on_action_saveCloud_triggered()
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

void MainWindow::on_action_unloadCloud_triggered()
{
    emit unloadCloud();
}

void MainWindow::on_action_unloadAllClouds_triggered()
{
    emit unloadAllClouds();
}

void MainWindow::on_action_showControlBar_triggered()
{
    emit showControlBar();
}

bool MainWindow::eventFilter(QObject *object, QEvent *event)
{
    if (event->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

        // cropBox Movement
        if(keyEvent->key() == Qt::Key_4) emit translateCropBox(1.0, 0.0, 0.0);
        if(keyEvent->key() == Qt::Key_6) emit translateCropBox(-1.0, 0.0, 0.0);
        if(keyEvent->key() == Qt::Key_8) emit translateCropBox(0.0, -1.0, 0.0);
        if(keyEvent->key() == Qt::Key_2) emit translateCropBox(0.0, 1.0, 0.0);
        if(keyEvent->key() == Qt::Key_9) emit translateCropBox(0.0, 0.0, 1.0);
        if(keyEvent->key() == Qt::Key_7) emit translateCropBox(0.0, 0.0, -1.0);
        // cropBox Misc
        if(keyEvent->key() == Qt::Key_Delete) emit cropCloud();

        // ui Misc
        if(keyEvent->key() == Qt::Key_Up) emit useNextActiveCloud();
        if(keyEvent->key() == Qt::Key_Down) emit usePreviousActiveCloud();
    }
    return QObject::eventFilter(object, event);
}
