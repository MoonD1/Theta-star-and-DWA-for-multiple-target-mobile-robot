#include "setseo.h"

SetSEO::SetSEO(QWidget *parent)
    : QWidget{parent}
{
    QVBoxLayout* layout = new QVBoxLayout(this);

    QLabel* text = new QLabel(  "Instructions：\n"
                                "Click the left mouse button to set the obstacle. Click the right \n"
                                "mouse button for the first time to set the starting point. Then \n"
                                "click the right mouse button to set the end point. Finally click \n"
                                "the \"Search Path\" button to start the search.");
    QLabel* text2 = new QLabel("Tip: a grid = 20m * 20m");

    layout -> addWidget(text);
    layout -> addWidget(text2);
}


SetSEO::~SetSEO(){}
