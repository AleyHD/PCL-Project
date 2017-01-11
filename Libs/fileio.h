#ifndef FILEIO_H
#define FILEIO_H

#include <QObject>
#include <QFile>
#include <QTextStream>
#include <QStringList>

class FileIO : public QObject
{
    Q_OBJECT
public:
    explicit FileIO(QObject *parent = 0);

    QStringList readTextFile(QString filepath);
    bool saveTextFile(QString filepath, QStringList linesToWrite);

signals:

public slots:
};

#endif // FILEIO_H
