//
// Created by Thomas Milas on 11/5/18.
//

#include "behavior-planner.h"


using namespace std;

double BehaviorPlanner::distanceFromVehicle(DetectedVehicle detected_vehicle, double x, double y, double s, PossibleTrajectory trajectory) {

    const auto high_thresh = 50.;
    const auto weight = 100.;
    if (!trajectory.isInRange(detected_vehicle.d)) return 0;

    auto dist = detected_vehicle.s - s;

    if (dist > high_thresh or dist < 0) return 0;

    cout << "Lane " << trajectory.getLaneId() << " distance: " << dist << endl;
    auto cost = (high_thresh - dist) / high_thresh;
    return cost * weight;
}


double BehaviorPlanner::changeLaneCost(DetectedVehicle detected_vehicle, PossibleTrajectory trajectory, int lane) {
    const auto weight = 1.;
    if (trajectory.getLaneId() != lane) return weight;

    return 0;
}

double BehaviorPlanner::calculateCost(DetectedVehicle detected_vehicle, double x, double y, double s, PossibleTrajectory trajectory, int lane) {

    auto cost = 0.;

    cost += distanceFromVehicle(detected_vehicle, x, y, s, trajectory);
//    cout << "after distance " << cost << endl;
    cost += changeLaneCost(detected_vehicle, trajectory, lane);
//    cout << "after same lane distance " <<  cost << endl;


    return cost;
}

vector<TrajectoryCost> BehaviorPlanner::calculateTrajectoryCosts(std::vector<std::vector<double>> &sensor_fusion, double car_x,
                                              double car_y, double s, int lane) {
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
            auto vehicle = kv.second.getNext();

            cost += calculateCost(vehicle, car_x, car_y, s, traj, lane);

        }
        calcd_traj.emplace_back(TrajectoryCost{cost, traj});
    }



//    cout << "min cost: " << min_traj->first << endl;

    return move(calcd_traj);
}


