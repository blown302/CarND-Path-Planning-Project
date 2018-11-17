//
// Created by Thomas Milas on 11/5/18.
//

#include "behavior-planner.h"

using namespace std;

double BehaviorPlanner::distanceFromVehicle(DetectedVehicle detected_vehicle, double s, PossibleTrajectory trajectory, int lane) {
    const auto high_thresh = 500.;
    const auto low_thresh = -20.;
    const auto weight = 2;

    auto min_lane = min(lane, trajectory.getLaneId());
    auto max_lane = max(lane, trajectory.getLaneId());

    auto vehicle_lane_id = getLane(detected_vehicle.d);

    if (trajectory.isInRange(detected_vehicle.d) or (max_lane > vehicle_lane_id and min_lane < vehicle_lane_id)) {
        auto dist = detected_vehicle.s - s;

        if (dist > high_thresh or dist < low_thresh) return 0;

        double cost;

        cost = (high_thresh - abs(dist)) / high_thresh;

        if (dist < 20. and dist > -15.) cost *= 10000;

        if (!trajectory.isInRange(detected_vehicle.d) and dist > 40) return 0;

        return cost * weight;
    }

    return 0;
}

double BehaviorPlanner::changeLaneCost(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory, int lane) {
    if (trajectory.getLaneId() == lane) return 0;

    return .1;
}

double BehaviorPlanner::calculateCost(DetectedVehicle detected_vehicle, double s, PossibleTrajectory trajectory, int lane) {
    auto cost = 0.;

    cost += distanceFromVehicle(detected_vehicle, s, trajectory, lane);
    cost += changeLaneCost(detected_vehicle, trajectory, lane);
    cost += speedCost(detected_vehicle, trajectory);

    return cost;
}

const vector<TrajectoryCost> BehaviorPlanner::calculateTrajectoryCosts(std::vector<std::vector<double>> &sensor_fusion, double s, double speed, int lane) {
    for (auto & c: sensor_fusion) {
        auto id = c[0];
        auto x = c[1];
        auto y = c[2];
        auto vx = c[3];
        auto vy = c[4];
        auto v = sqrt(pow(vx, 2) + pow(vy, 2)) * 2.24;
        auto s = c[5];
        auto d = c[6];

        if (d <= 0. or d >= 12.) continue;

        m_detected_vehicles[id].add({x, y, v, s, d});
    }

    vector<TrajectoryCost> calcd_traj;

    for (auto &traj: m_possible_trajectories) {
        auto cost = 0., car_in_front_dist = 1000., car_in_front_speed = 0.;
        traj.resetSpeed();
        for(auto &kv : m_detected_vehicles) {
            auto vehicle = kv.second.getNext();
            auto dist = vehicle.s - s;
            if (traj.isInRange(vehicle.d) and dist > 0 and dist < car_in_front_dist) {
                car_in_front_dist = dist;
                car_in_front_speed = vehicle.v;
            }

            cost += calculateCost(vehicle, s, traj, lane);
        }

        if (car_in_front_dist < 20) traj.setOveriddenSpeed(car_in_front_speed);
        calcd_traj.emplace_back(TrajectoryCost{cost, traj});
    }
    return calcd_traj;
}

double BehaviorPlanner::speedCost(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory) {
    auto weight = 1.;
    const auto target_speed = 50.;

    auto cost = (target_speed - detected_vehicle.v) / target_speed;
    return cost * weight;
}

