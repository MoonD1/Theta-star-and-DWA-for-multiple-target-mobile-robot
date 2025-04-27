#ifndef MOBILEOBS_H
#define MOBILEOBS_H

#include <QVector>

struct MobileRobot{
    double x;
    double y;
    double radius;
    double speed;
    QVector<std::pair<double, double>> path;
    QVector<double> pathLength;
    double curLength;
    double curTime;

    MobileRobot(double radius, double speed): x(-1), y(-1), radius(radius), speed(speed), path({}), pathLength({}), curLength(0), curTime(0){}
};


class MobileObs
{
    static MobileRobot* curRobot;
    static QVector<MobileRobot*> robots;

public:
    MobileObs();

    static void setCurRobot(MobileRobot* robot);
    static void setPath(double x, double y);
    static void save();
    static QVector<MobileRobot*>& getRobots();

    ~MobileObs();
};

#endif // MOBILEOBS_H
