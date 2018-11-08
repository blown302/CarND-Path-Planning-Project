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
public:
    PossibleTrajectory() = default;
    PossibleTrajectory(int lane_id, double min_d, double max_d): m_lane_id(lane_id), m_min_d(min_d), m_max_d(max_d) {}
    bool isInRange(double d) const {
        return d > m_min_d and d < m_max_d;
    }
    int getLaneId() const {
        return m_lane_id;
    }
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
    double vx;
    double vy;
    double v;
    double s;
    double d;
};

class BehaviorPlanner {
private:
    std::vector<PossibleTrajectory> m_possible_trajectories;
    std::unordered_map<double, Buffer<DetectedVehicle>> m_detected_vehicles;

    double sameLaneAsVehicle(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory);
    double calculateCost(DetectedVehicle detected_vehicle, double x, double y, double last_s, PossibleTrajectory trajectory);
    double distanceFromVehicle(DetectedVehicle detected_vehicle, double x, double y, double last_s, PossibleTrajectory trajectory);
public:
    BehaviorPlanner() {
        m_possible_trajectories.emplace_back(PossibleTrajectory{0, 0., 4.});
        m_possible_trajectories.emplace_back(PossibleTrajectory{1, 4., 8.});
        m_possible_trajectories.emplace_back(PossibleTrajectory{2, 8., 12.});
    }
    int determineBestTrajectory(std::vector<std::vector<double>> &sensor_fusion, double x, double y, double last_s);
};



#endif //PATH_PLANNING_BEHAVIOR_PLANNER_H
