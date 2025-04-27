#ifndef SETMOVEO_H
#define SETMOVEO_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>
#include <mobileobs.h>

class SetMoveO : public QWidget
{
    Q_OBJECT
    QLineEdit* radiusEdit;
    QLineEdit* speedEdit;


    void createRobot();


public:
    explicit SetMoveO(QWidget *parent = nullptr);
    ~SetMoveO();
signals:
};

#endif // SETMOVEO_H
