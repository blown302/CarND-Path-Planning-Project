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
    Map(std::vector<double> x,
        std::vector<double> y,
        std::vector<double> s,
        std::vector<double> dx,
        std::vector<double> dy) : x(x), y(y), s(s), dx(dx), dy(dy) {

        for (int i = 0; i < s.size(); ++i) {
            m_map_s[s[i]] = i;
        }
    };

    std::vector<double> getNextWaypointByS(double S) {
        int index = m_map_s[S] + 1;
        return std::vector<double> {s[index], x[index] - dx[index], y[index] - dy[index]};
    }
};

enum VehicleState {KeepLane, ChangeLaneLeft, ChangeLaneRight};
class Vehicle {
private:
    StateMachine<VehicleState> state;
    Trajectory m_trajectory;
    unsigned int lane{0};
    double target_velocity{45.};

    void keepLane();
public:
    Vehicle(): state(KeepLane) {
        state.registerAction(KeepLane, [this] () { keepLane();});
    };
    const Trajectory &getTrajectory() const {return m_trajectory;};
    void update(double x, double y, double yaw, std::vector<double> &prev_x, std::vector<double> &prev_y, double last_s, double last_d, Map &map);
};


#endif //PATH_PLANNING_VEHICLE_H
