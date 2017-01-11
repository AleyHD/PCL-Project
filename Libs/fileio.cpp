#include "fileio.h"

FileIO::FileIO(QObject *parent) : QObject(parent)
{

}

QStringList FileIO::readTextFile(QString filepath)
{
    QFile file(filepath);

    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return QStringList();
    QTextStream stream(&file);
    QStringList lines;
    while (!stream.atEnd())
        lines << stream.readLine().simplified();
    file.close();

    return lines;
}

bool FileIO::saveTextFile(QString filepath, QStringList linesToWrite)
{
    QFile file(filepath);

    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return false;
    QTextStream stream(&file);
    for (QStringList::iterator it=linesToWrite.begin(); it!=linesToWrite.end(); ++it) {
        stream << *it << endl;
    }
    file.close();

    return true;
}
