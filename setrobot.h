#ifndef SETROBOT_H
#define SETROBOT_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QPushButton>

class SetRobot : public QWidget
{
    Q_OBJECT

    QLineEdit* maxSpeedEdit;
    QLineEdit* minSpeedEdit;
    QLineEdit* maxYawRateEdit;
    QLineEdit* maxAccelEdit;
    QLineEdit* maxDeltaYawRateEdit;
    QLineEdit* vResolutionEdit;
    QLineEdit* yawRateResolutionEdit;
    QLineEdit* dtEdit;
    QLineEdit* predictTimeEdit;
    QLineEdit* toGoalCostCoefficientEdit;
    QLineEdit* speedCostCoefficientEdit;
    QLineEdit* obstacleCostCoefficientEdit;
    QLineEdit* mobileObsCostCoefficientEdit;
    QLineEdit* robotStuckFlagConsEdit;
    QLineEdit* robotRadiusEdit;

    void setRobot();

public:
    explicit SetRobot(QWidget *parent = nullptr);
    ~SetRobot();
signals:
};

#endif // SETROBOT_H
