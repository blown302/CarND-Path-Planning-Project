#include <utility>

//
// Created by Thomas Milas on 11/1/18.
//

#include "vehicle.h"

using namespace std;

void Vehicle::update(double x, double y, double yaw, vector<double> &prev_x, vector<double> &prev_y, double last_s, double last_d, Map &map, vector<vector<double>> sensor_fusion) {
    m_x = x;
    m_y = y;
    m_last_d = last_d;
    m_last_s = last_s;
    m_sensor_fusion = std::move(sensor_fusion);


//    cout << "using lane: " << m_lane << endl;

    vector<double> spline_x;
    vector<double> spline_y;

    auto prev_size = prev_x.size();

    double ref_x, ref_y, prev_ref_x, prev_ref_y;
    if (prev_x.size() < 2) {
        prev_ref_x = x - cos(deg2rad(yaw));
        prev_ref_y = y - sin(deg2rad(yaw));

        ref_x = x;
        ref_y = y;

    } else {
        ref_x = prev_x[prev_size -1];
        ref_y = prev_y[prev_y.size() -1];

        prev_ref_x = prev_x[prev_size -2];
        prev_ref_y = prev_y[prev_size -2];
    }

    m_prev_x = ref_x;
    m_prev_y = ref_y;
    m_state.process();

    // push previous points point onto the spline vectors.
    spline_x.push_back(prev_ref_x);
    spline_y.push_back(prev_ref_y);
    spline_x.push_back(ref_x);
    spline_y.push_back(ref_y);

    auto d = 2 + m_lane * 4;
    double wp_s = map.s[NextWaypoint(x, y, deg2rad(yaw), map.x, map.y)];
    auto xy = getXY(wp_s, d, map.s, map.x, map.y);

    while (xy[0] < ref_x) {
        wp_s = map.getNextWaypointByS(wp_s)[0];
        xy = getXY(wp_s, d, map.s, map.x, map.y);
    }

    spline_x.push_back(xy[0]);
    spline_y.push_back(xy[1]);

    for (int i = 0; i < 3; ++i) {
        wp_s = map.getNextWaypointByS(wp_s)[0];
        xy = getXY(wp_s, d, map.s, map.x, map.y);
        spline_x.push_back(xy[0]);
        spline_y.push_back(xy[1]);
    }

    tk::spline spline;
    spline.set_points(spline_x, spline_y);

    m_trajectory.update(prev_x, prev_y, last_s, last_d);

    // set speed
    auto target_x = ref_x + 30;
    auto target_y = spline(target_x);
    auto target_distance = sqrt(pow(ref_x - target_x, 2) + pow(ref_y - target_y, 2));


    auto points_ahead = static_cast<int>(target_distance / (.02 * m_target_velocity / 2.24));
    auto point_interval = target_distance / points_ahead;
    m_trajectory.setPointsAhead(points_ahead);
    while (!m_trajectory.is_full()) {
        ref_x += point_interval;
        ref_y = spline(ref_x);
        m_trajectory.add(ref_x, ref_y);
    }
}

void Vehicle::keepLane() {
//    cout << "running state keeping lane" << endl;
    auto trajectory_costs = m_planner.calculateTrajectoryCosts(m_sensor_fusion, m_prev_x, m_prev_y, m_last_s);

    cout << "calc'd trajectories: ";

    auto min = 10000., current_lane_cost = 0.;
    auto min_lane_id = m_lane;
    for (auto &t: trajectory_costs) {

        if (t.cost < min) {
            min = t.cost;
            min_lane_id = t.trajectory.getLaneId();
        }
        if (t.trajectory.getLaneId() == m_lane)
            current_lane_cost = t.cost;

        cout << "lane: " << t.trajectory.getLaneId() << " with cost: " << t.cost << " ";
    }

    if (min != current_lane_cost) {
        auto delta = min_lane_id - m_lane;
        if (delta == 1) {
            m_state.fire(ChangeLaneRight);
        } else if (delta == -1) {
            m_state.fire(ChangeLaneLeft);
        } else {
            cout << "keeping lane: " << m_lane << endl;
        }
    } else {
        cout << "keeping lane: " << m_lane << endl;
    }




}

void Vehicle::changeLaneLeft() {
//    cout << "running state changing left" << endl;
    // TODO: consolidate or extract method.
    const auto lane_tolerance = .25;
    auto d = getIdealD();

    if (m_last_d > d - lane_tolerance and m_last_d < d + lane_tolerance) {
        m_state.fire(KeepLane);
    }
}

void Vehicle::changeLaneRight() {
//    cout << "running state changing right" << endl;
    const auto lane_tolerance = 1;
    auto d = getIdealD();

    if (m_last_d > d - lane_tolerance and m_last_d < d + lane_tolerance) {
        m_state.fire(KeepLane);
    }
}

void Vehicle::changeLeftEntry() {
    m_lane--;
    cout << "running entry of change left state to lane: " << m_lane << endl;
}

void Vehicle::changeRightEntry() {
    m_lane++;
    cout << "running entry of change right state to lane: " << m_lane << endl;
}


