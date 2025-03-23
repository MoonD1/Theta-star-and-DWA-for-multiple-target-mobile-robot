#include "setmoveo.h"

SetMoveO::SetMoveO(QWidget *parent)
    : QWidget{parent}
{
    QVBoxLayout* layout = new QVBoxLayout(this);

    QLabel* text = new QLabel("test2");

    layout -> addWidget(text);

}

SetMoveO::~SetMoveO(){}
