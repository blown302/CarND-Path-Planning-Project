#include <utility>

//
// Created by Thomas Milas on 11/1/18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <vector>

#include "state-machine.h"
#include "utility.h"
#include "spline.h"
#include "behavior-planner.h"

class Trajectory {
private:
    std::vector<double> m_x;
    std::vector<double> m_y;
    int points_ahead = 30;
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
    Map(std::vector<double> &x,
        std::vector<double> &y,
        std::vector<double> &s,
        std::vector<double> &dx,
        std::vector<double> &dy) : x(x), y(y), s(s), dx(dx), dy(dy) {
    };
};

enum VehicleState {KeepingLane, ChangingLaneLeft, ChangingLaneRight};
enum EventTrigger {KeepLane, ChangeLaneRight, ChangeLaneLeft};

class Vehicle {
private:
    StateMachine<VehicleState, EventTrigger> m_state;
    BehaviorPlanner m_planner;
    Trajectory m_trajectory;
    int m_lane{1};
    double m_x;
    double m_y;
    double m_yaw;
    double m_v;
    double m_max_velocity{48.};
    double m_target_velocity{1.};
    double m_overridden_velocity{0};
    bool m_override_speed{false};
    double m_ref_x{0};
    double m_ref_y{0};
    double m_ref_theta{0};
    double m_last_s;
    Map m_map;
    double m_s {};
    tk::spline m_spline;

    std::vector<double> m_spline_x;
    std::vector<double> m_spline_y;

    std::vector<std::vector<double>> m_sensor_fusion;

    // action methods

    /**
     * Action to perform when vehicle is in the KeepLane state.
     */
    void keepLane();

    /**
     * Transition action to perform on the entry of ChangingLaneLeft state.
     */
    void changeLeftEntry();

    /**
     * Action to perform when vehicle is in the ChangingLaneRight or ChangingLaneLeft states.
     * Monitors the completion of Lane changes.
     */
    void changeLane();

    /**
     * Transition action to perform on the entry of ChangingLaneRight state.
     */
    void changeRightEntry();

    /**
     * Gets the ideal frenet D Coordinate based on target lane.
     * @return D coordinate.
     */
    double getIdealD() {
        return m_lane * 4 + 2;
    }

    /**
     * Updates spline used for mapping trajectory with new coordinates.
     * Uses residual previous trajectory and most resent target coordinates.
     */
    void updateSpline();
    double getInterval(double target_distance, double x_dist);
public:
    Vehicle(Map map): m_state(KeepingLane), m_map(std::move(map)) {
        m_state.registerAction(KeepingLane, [this] () { keepLane();});

        m_state.registerAction(ChangingLaneLeft, [this] () { changeLane();});
        m_state.onEntry(ChangingLaneLeft, [this] () {changeLeftEntry();});


        m_state.registerAction(ChangingLaneRight, [this] () { changeLane();});
        m_state.onEntry(ChangingLaneRight, [this] () {changeRightEntry();});

        m_state.allow(KeepingLane, ChangingLaneLeft, ChangeLaneLeft);
        m_state.allow(KeepingLane, ChangingLaneRight, ChangeLaneRight);
        m_state.allow(ChangingLaneLeft, KeepingLane, KeepLane);
        m_state.allow(ChangingLaneRight, KeepingLane, KeepLane);
    };
    const Trajectory &getTrajectory() const {return m_trajectory;};

    /**
     * Updates the vehicle with the latest telemetry.
     * @param x
     * @param y
     * @param yaw
     * @param prev_x
     * @param prev_y
     * @param last_s
     * @param last_d
     * @param sensor_fusion
     * @param v
     */
    void update(double x, double y, double yaw, std::vector<double> &prev_x, std::vector<double> &prev_y, double last_s, double last_d, std::vector<std::vector<double>> sensor_fusion, double v);
};
#endif //PATH_PLANNING_VEHICLE_H
