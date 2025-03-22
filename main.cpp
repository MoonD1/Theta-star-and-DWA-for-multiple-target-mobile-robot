#include "widget.h"
#include "start.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Start s;
    s.show();
    //Widget w;
    //w.show();
    return a.exec();
}
