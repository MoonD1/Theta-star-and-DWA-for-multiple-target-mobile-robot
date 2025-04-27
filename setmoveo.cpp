#include "setmoveo.h"



SetMoveO::SetMoveO(QWidget *parent)
    : QWidget{parent}
{
    QVBoxLayout* layout = new QVBoxLayout(this);
    QHBoxLayout* line1 = new QHBoxLayout();
    QHBoxLayout* line2 = new QHBoxLayout();
    QHBoxLayout* line3 = new QHBoxLayout();

    QLabel* radiusLabel = new QLabel("radius:");
    QLabel* speedLabel = new QLabel("speed:");

    radiusEdit = new QLineEdit(this);
    speedEdit = new QLineEdit(this);

    radiusEdit -> setPlaceholderText("m, default 20.0");
    speedEdit -> setPlaceholderText("m/s, default 2.0");

    QPushButton* create = new QPushButton("Create");
    connect(create, &QPushButton::clicked, this, &SetMoveO::createRobot);
    QPushButton* save = new QPushButton("Save");
    connect(save, &QPushButton::clicked, this, &MobileObs::save);

    line1 -> addWidget(radiusLabel);
    line1 -> addWidget(radiusEdit);
    line2 -> addWidget(speedLabel);
    line2 -> addWidget(speedEdit);
    line3 -> addWidget(create);
    line3 -> addWidget(save);

    layout -> addLayout(line1);
    layout -> addLayout(line2);
    layout -> addLayout(line3);

}

void SetMoveO::createRobot(){
    bool isRadius;
    bool isSpeed;
    double radius = radiusEdit -> text().toDouble(&isRadius);
    double speed = speedEdit -> text().toDouble(&isSpeed);
    radius = isRadius ? radius : 20.0;
    speed = isSpeed ? speed : 2.0;
    MobileRobot* robot = new MobileRobot(radius, speed);
    MobileObs::setCurRobot(robot);
}


SetMoveO::~SetMoveO(){}
