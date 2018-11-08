//
// Created by Thomas Milas on 11/1/18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>

#include "state-machine.h"
#include "utility.h"
#include "spline.h"
#include "unordered_map"
#include "behavior-planner.h"


struct Point {
    double s;
    double d;
};

class Trajectory {
private:
    std::vector<double> m_x;
    std::vector<double> m_y;
    int points_ahead = 100;
public:
    void setPointsAhead(int points_ahead) {
        Trajectory::points_ahead = points_ahead;
    }

public:
    Trajectory() = default;
    void update(std::vector<double> &x, std::vector<double> &y, double last_s, double last_d) {
        m_x = x;
        m_y = y;
    }
    void add(double x, double y){
        m_x.push_back(x);
        m_y.push_back(y);
    }
    bool is_full() {
        return points_ahead <= m_x.size();
    }
    std::vector<double> getXPath() {return m_x;};
    std::vector<double> getYPath() {return m_y;};
};

class Map {
private:

public:
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> s;
    std::vector<double> dx;
    std::vector<double> dy;
    std::unordered_map<double, int> m_map_s;
    Map(std::vector<double> &x,
        std::vector<double> &y,
        std::vector<double> &s,
        std::vector<double> &dx,
        std::vector<double> &dy) : x(x), y(y), s(s), dx(dx), dy(dy) {

        for (int i = 0; i < s.size(); ++i) {
            m_map_s[s[i]] = i;
        }
    };

    std::vector<double> getNextWaypointByS(double S) {
        int index = m_map_s[S] + 1;
        return std::vector<double> {s[index], x[index] - dx[index], y[index] - dy[index]};
    }
};

enum VehicleState {KeepingLane, ChangingLaneLeft, ChangingLaneRight};
enum EventTrigger {KeepLane, ChangeLaneRight, ChangeLaneLeft};

class Vehicle {
private:
    StateMachine<VehicleState, EventTrigger> m_state;
    BehaviorPlanner m_planner;
    Trajectory m_trajectory;
    int m_lane{1};
    double m_target_velocity{45.};
    double m_x{0};
    double m_y{0};
    double m_last_d;
    double m_last_s;
    double m_prev_x;
    double m_prev_y;
    std::vector<std::vector<double>> m_sensor_fusion;

    // action methods
    void keepLane();
    void changeLaneLeft();
    void changeLeftEntry();
    void changeLaneRight();
    void changeRightEntry();

    double getIdealD() {
        return m_lane * 4 + 2;
    }

public:
    Vehicle(): m_state(KeepingLane) {
        m_state.registerAction(KeepingLane, [this] () { keepLane();});


        m_state.registerAction(ChangingLaneLeft, [this] () { changeLaneLeft();});
        m_state.onEntry(ChangingLaneLeft, [this] () {changeLeftEntry();});


        m_state.registerAction(ChangingLaneRight, [this] () { changeLaneRight();});
        m_state.onEntry(ChangingLaneRight, [this] () {changeRightEntry();});

        m_state.allow(KeepingLane, ChangingLaneLeft, ChangeLaneLeft);
        m_state.allow(KeepingLane, ChangingLaneRight, ChangeLaneRight);
        m_state.allow(ChangingLaneLeft, KeepingLane, KeepLane);
        m_state.allow(ChangingLaneRight, KeepingLane, KeepLane);
    };
    const Trajectory &getTrajectory() const {return m_trajectory;};
    void update(double x, double y, double yaw, std::vector<double> &prev_x, std::vector<double> &prev_y, double last_s, double last_d, Map &map, std::vector<std::vector<double>> sensor_fusion);
};
#endif //PATH_PLANNING_VEHICLE_H
