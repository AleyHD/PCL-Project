#ifndef THREADMANAGER_H
#define THREADMANAGER_H

#include <QObject>
#include <QThread>
#include <QStringList>
#include <QMap>

class ThreadManager : public QObject
{
    Q_OBJECT
public:
    explicit ThreadManager(const QString &threadName, QObject *parent = 0);
    ~ThreadManager();

    bool addThread(const QString &threadName);
    bool delThread(const QString &threadName);
    void moveObjectToThread(QObject *object, const QString &threadName);

    // getters
    QStringList objectList(const QString &threadName);


signals:

public slots:

private slots:


private:
    QMap<QString, QThread*> threadIdentifyMap_;
    QMap<QThread*, QList<QObject*> > objectIdentifyMap_;

};

#endif // THREADMANAGER_H
