#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QLayout>
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
    void setActionSaveCloudEnabled(bool decision);

signals:
    void loadCloud(QString);
    void saveCloud(QString);
    void unloadCloud();
    void unloadAllClouds();
    void useNextActiveCloud();
    void usePreviousActiveCloud();

    void showControlBar();

    void translateCropBox(double, double, double);
    void cropCloud();

protected:
    bool eventFilter(QObject *object, QEvent *event);

private slots:

    void on_action_loadCloud_triggered();
    void on_action_saveCloud_triggered();
    void on_action_unloadCloud_triggered();
    void on_action_unloadAllClouds_triggered();

    void on_action_showControlBar_triggered();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
