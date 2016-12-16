#ifndef POINTCLOUDMANAGER_H
#define POINTCLOUDMANAGER_H

#include <QObject>
#include "pointcloud.h"

class PointCloudManager : public QObject
{
    Q_OBJECT
public:
    explicit PointCloudManager(QObject *parent = 0);

    bool addCloud(const QString &cloudName);

signals:

public slots:

private:
    QMap<QString, PointCloud*> cloudIdentifyMap_;
};

#endif // POINTCLOUDMANAGER_H
