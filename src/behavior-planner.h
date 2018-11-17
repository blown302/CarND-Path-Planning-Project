//
// Created by Thomas Milas on 11/5/18.
//

#ifndef PATH_PLANNING_BEHAVIOR_PLANNER_H
#define PATH_PLANNING_BEHAVIOR_PLANNER_H

#include <vector>
#include <deque>
#include <unordered_map>
#include <cmath>
#include "utility.h"
#include <iostream>
#include <utility>

class PossibleTrajectory {
private:
    int m_lane_id;
    double m_min_d;
    double m_max_d;
    bool m_override_speed;
    double m_speed;
public:
    PossibleTrajectory() = default;
    PossibleTrajectory(int lane_id, double min_d, double max_d): m_lane_id(lane_id), m_min_d(min_d), m_max_d(max_d), m_override_speed(false), m_speed(0) {}
    bool isInRange(double d) const {
        return d > m_min_d and d < m_max_d;
    }
    int getLaneId() const {
        return m_lane_id;
    }
    bool isOverrideSpeed() const {
        return m_override_speed;
    }
    void setOveriddenSpeed(double speed) {
        m_override_speed = true;
        m_speed = speed;
    }

    double getOverriddenSpeed() const {
        return m_speed;
    }

    void resetSpeed() {
        m_override_speed = false;
    }
};

class TrajectoryCost {
public:
    TrajectoryCost(double cost, PossibleTrajectory& trajectory): cost(cost), trajectory(trajectory) {}
    double cost;
    PossibleTrajectory trajectory;
};

template <class T>
class Buffer {
private:
    std::deque<T> m_storage;
    int m_max_size;
    bool isFull() {return m_max_size <= m_storage.size();}
    void dropElement() {m_storage.pop_back();}
public:
    explicit Buffer(int max_size = 10): m_max_size(max_size) {}
    void add(T element) {
        m_storage.push_front(element);

        if (isFull()) dropElement();
    }
    T getNext() {
        return m_storage.front();
    }
};

struct DetectedVehicle {
    double x;
    double y;
    double v;
    double s;
    double d;
};

class BehaviorPlanner {
private:
    std::vector<PossibleTrajectory> m_possible_trajectories;
    std::unordered_map<double, Buffer<DetectedVehicle>> m_detected_vehicles;

    /**
     * Adds lane cost to a trajectory that changes lanes.
     *
     * @param detected_vehicle
     * @param trajectory
     * @param lane
     * @param s
     * @return cost of changing lanes.
     */
    double speedCost(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory);
    double changeLaneCost(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory, int lane);
    /**
     *
     * @param detected_vehicle
     * @param s
     * @param trajectory
     * @param lane
     * @return
     */
    double calculateCost(DetectedVehicle detected_vehicle, double s, PossibleTrajectory trajectory, int lane);
    /**
     * Cost function to that measures distance to vehicle.
     * Increases cost when within a threshold distance of a detected vehicle.
     * @param detected_vehicle
     * @param s
     * @param trajectory
     * @param lane
     * @return
     */
    double distanceFromVehicle(DetectedVehicle detected_vehicle, double s, PossibleTrajectory trajectory, int lane);
public:
    BehaviorPlanner() {
        const auto speed_delta = .25;

        m_possible_trajectories.emplace_back(PossibleTrajectory{0, 0., 4.});
        m_possible_trajectories.emplace_back(PossibleTrajectory{1, 4., 8.});
        m_possible_trajectories.emplace_back(PossibleTrajectory{2, 8., 12.});
    }

    /**
     * Calculates the costs of all of the possible trajectories.
     * @param sensor_fusion
     * @param s
     * @param speed
     * @param lane
     * @return
     */
    const std::vector<TrajectoryCost> calculateTrajectoryCosts(std::vector<std::vector<double>> &sensor_fusion, double s, double speed, int lane);
};

#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
