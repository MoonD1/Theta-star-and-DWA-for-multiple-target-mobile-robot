#include "start.h"

#include <QRegularExpressionValidator>
#include <QLabel>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>

#include <QDebug>

Start::Start(QWidget* parent)
    : QWidget(parent)
{
    // Set window title.
    this->setWindowTitle("Path Simulation");

    // Set window size.
    this->setFixedSize(300, 150);

    // Set Height and Width.
    QLabel* heightLabel = new QLabel("网格高度:", this);
    heightEdit = new QLineEdit(this);
    heightEdit -> setPlaceholderText("1~50, 默认20");
    // heightEdit -> setGeometry(50, 30, 200, 30);

    QLabel* widthLabel = new QLabel("网格宽度:", this);
    widthEdit = new QLineEdit(this);
    widthEdit -> setPlaceholderText("1~50, 默认20");
    // widthEdit -> setGeometry(50, 80, 200, 30);

    QRegularExpression sizeRegExp("^(?:[1-9]|[1-4][0-9]|[5][0])$");
    heightEdit -> setValidator(new QRegularExpressionValidator(sizeRegExp, heightEdit));
    widthEdit -> setValidator(new QRegularExpressionValidator(sizeRegExp, widthEdit));

    QPushButton* start = new QPushButton("start", this);



    connect(start, &QPushButton::clicked, [&](){
        int height = heightEdit -> text().toInt();
        int width = widthEdit -> text().toInt();
        widget = new Widget(nullptr, height ? height : 20, width ? width : 20);
        widget -> show();
        this -> close();
    });

    QVBoxLayout* bigLayout = new QVBoxLayout(this);
    QHBoxLayout* layout1 = new QHBoxLayout();
    layout1 -> addWidget(heightLabel);
    layout1 -> addWidget(heightEdit);
    QHBoxLayout* layout2 = new QHBoxLayout();
    layout2 -> addWidget(widthLabel);
    layout2 -> addWidget(widthEdit);
    bigLayout -> addLayout(layout1);
    bigLayout -> addLayout(layout2);
    bigLayout -> addWidget(start);


}


Start::~Start(){}
