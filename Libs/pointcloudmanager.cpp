#include "pointcloudmanager.h"

PointCloudManager::PointCloudManager(QObject *parent) : QObject(parent)
{

}

bool PointCloudManager::addCloud(const QString &cloudName)
{
    // adds a new cloud
    if (cloudIdentifyMap_.contains(cloudName)) return false;
    PointCloud *cloud = new PointCloud(this);
    cloudIdentifyMap_.insert(cloudName, cloud);
    return true;
}
