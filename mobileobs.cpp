#include "mobileobs.h"
#include <QDebug>

MobileObs::MobileObs() {}

MobileRobot* MobileObs::curRobot = nullptr;
QVector<MobileRobot*> MobileObs::robots = {};

void MobileObs::setCurRobot(MobileRobot* robot){
    // qDebug() << "Setting curRobot to " << robot;
    curRobot = robot;
    robots.push_back(curRobot);
}

void MobileObs::setPath(double x, double y){
    // qDebug() << "curRobot " << curRobot;
    if(curRobot == nullptr){
        return;
    }

    if(curRobot -> x == -1){
        curRobot -> x = x;
        curRobot -> y = y;
    }
    else{
        double lastx = curRobot -> path.back().first;
        double lasty = curRobot -> path.back().second;
        double length = sqrt((x - lastx) * (x - lastx) + (y - lasty) * (y - lasty));
        if(curRobot -> pathLength.size() == 0){
            curRobot -> pathLength.push_back(0);
        }
        curRobot -> pathLength.push_back(length + curRobot -> pathLength.back());
    }
    curRobot -> path.push_back({x, y});
}

void MobileObs::save(){
    if(curRobot == nullptr){
        return;
    }
    if(curRobot -> x == -1){
        robots.remove(robots.size() - 1);
    }
    curRobot = nullptr;
}

QVector<MobileRobot*>& MobileObs::getRobots(){
    return robots;
}

MobileObs::~MobileObs(){}
