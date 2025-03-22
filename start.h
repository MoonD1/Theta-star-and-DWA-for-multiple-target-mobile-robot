#ifndef START_H
#define START_H

#include <QWidget>
#include <QLineEdit>

#include "widget.h"

class Start: public QWidget
{
    Q_OBJECT

    QLineEdit* heightEdit;
    QLineEdit* widthEdit;
    Widget* widget;

public:
    Start(QWidget* parent = nullptr);
    ~Start();
};

#endif // START_H
