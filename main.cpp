#include "pclproject.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    PclProject project;

    return a.exec();
}
