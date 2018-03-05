#include "QtViewer.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    QtViewer v;
    v.resize(1000,1000);
    v.show();
    return a.exec();
}
