#include "threadmanager.h"

ThreadManager::ThreadManager(const QString &threadName, QObject *parent) : QObject(parent)
{
    this->addThread(threadName);
}

ThreadManager::~ThreadManager()
{
    // destructor
    // every thread setup via ThreadManager will be deleted
    for (QThread *thread : threadIdentifyMap_) {
        thread->quit();
        thread->wait();
    }
}

bool ThreadManager::addThread(const QString &threadName)
{
    // adds a new thread
    if (threadIdentifyMap_.contains(threadName)) return false;
    QThread *thread = new QThread();
    thread->start();
    threadIdentifyMap_.insert(threadName, thread);
    objectIdentifyMap_.insert(thread, QList<QObject*>());
    return true;
}

bool ThreadManager::delThread(const QString &threadName)
{
    // deletes desired thread including all belonging objects

    // if thread name does not exist return failure
    if (!threadIdentifyMap_.contains(threadName)) return false;
    // delete desired thread
    QThread *thread = threadIdentifyMap_.value(threadName);
    thread->quit();
    thread->wait();
    // return success
    return true;
}

void ThreadManager::moveObjectToThread(QObject *object, const QString &threadName)
{
    // if thread name does not exist return failure
    if (!threadIdentifyMap_.contains(threadName)) this->addThread(threadName);
    // move object to desired thread
    QThread *thread = threadIdentifyMap_.value(threadName);
    object->moveToThread(thread);
    // append object to list
    objectIdentifyMap_[thread].append(object);
    // connect signals to slots
    connect(thread,SIGNAL(finished()),object,SLOT(deleteLater()));
}

QStringList ThreadManager::objectList(const QString &threadName)
{
    // returns a list of object names belonging to a thread
    // if no object names are defined, the thread does not exist
    // or no objects belonging to the thread, an empty list is returned
    QStringList stringList;
    if (threadIdentifyMap_.contains(threadName)) {
        QThread *thread = threadIdentifyMap_.value(threadName);
        QList<QObject*> objects = objectIdentifyMap_.value(thread);
        for (QObject *object : objects) stringList << object->objectName();
    }
    return stringList;
}
