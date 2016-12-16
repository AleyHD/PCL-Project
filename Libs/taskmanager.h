#ifndef TASKMANAGER_H
#define TASKMANAGER_H

#include <QObject>

class TaskManager : public QObject
{
    Q_OBJECT
public:
    explicit TaskManager(QObject *parent = 0);

signals:

public slots:
};

#endif // TASKMANAGER_H