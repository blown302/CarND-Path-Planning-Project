//
// Created by Thomas Milas on 11/5/18.
//

#include "behavior-planner.h"


using namespace std;

double BehaviorPlanner::distanceFromVehicle(DetectedVehicle detected_vehicle, double x, double y, double last_s, PossibleTrajectory trajectory) {

    const auto high_thresh = 30.;
    const auto low_thesh = 10.;
    const auto weight = 1.;
    if (!trajectory.isInRange(detected_vehicle.d)) return 0;

//    auto dist = detected_vehicle.s - last_s;
    auto dist = distance(x, y, detected_vehicle.x, detected_vehicle.y);
    if (dist > high_thresh) return 0;
    if (dist < low_thesh) return 1;

    auto cost = (high_thresh - dist) / high_thresh;
//    cout <<  "d: " << detected_vehicle.d << " with distance " << dist << " cost " << cost << endl;
    return cost * weight;
}


double BehaviorPlanner::sameLaneAsVehicle(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory) {
    const auto weight = 1;
    if (trajectory.isInRange(detected_vehicle.d)) return weight;

    return 0;
}

double BehaviorPlanner::calculateCost(DetectedVehicle detected_vehicle, double x, double y, double last_s, PossibleTrajectory trajectory) {

    auto cost = 0.;

    cost += distanceFromVehicle(detected_vehicle, x, y, last_s, trajectory);
//    cout << "after distance " << cost << endl;
//    cost += sameLaneAsVehicle(detected_vehicle, trajectory);
//    cout << "after same lane distance " <<  cost << endl;


    return cost;
}

vector<TrajectoryCost> BehaviorPlanner::calculateTrajectoryCosts(std::vector<std::vector<double>> &sensor_fusion, double car_x,
                                              double car_y, double last_s) {
    for (auto c: sensor_fusion) {
        auto id = c[0];
        auto x = c[1];
        auto y = c[2];
        auto vx = c[3];
        auto vy = c[4];
        auto v = sqrt(pow(vx, 2) + pow(vy, 2));
        auto s = c[5];
        auto d = c[6];

        if (d < 0. or d > 12.) continue;

        m_detected_vehicles[id].add({x, y, vx, vy, v, s,d});
    }

    vector<TrajectoryCost> calcd_traj;

    for (auto &traj: m_possible_trajectories) {
        auto cost = 0.;
        for(auto &kv : m_detected_vehicles) {
            auto id = kv.first;
            auto vehicle = kv.second.getNext();

            cost += calculateCost(vehicle, car_x, car_y, last_s, traj);

        }
        calcd_traj.emplace_back(TrajectoryCost{cost, traj});
    }



//    cout << "min cost: " << min_traj->first << endl;

    return move(calcd_traj);
}


