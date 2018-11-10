#include <utility>

//
// Created by Thomas Milas on 11/1/18.
//

#include "vehicle.h"

using namespace std;

void Vehicle::update(double x, double y, double yaw, vector<double> &prev_x, vector<double> &prev_y, double last_s, double last_d, vector<vector<double>> sensor_fusion, double v) {
    m_last_d = last_d;
    m_last_s = last_s;
    m_yaw = yaw;
    m_x = x;
    m_y = y;
    m_sensor_fusion = std::move(sensor_fusion);
//    m_target_velocity = v;


//    cout << "using lane: " << m_lane << endl;

    m_spline_x = vector<double>();
    m_spline_y = vector<double>();

    auto prev_size = prev_x.size();

    double prev_ref_x, prev_ref_y;
    if (prev_x.size() < 2) {
        prev_ref_x = x - cos(deg2rad(yaw));
        prev_ref_y = y - sin(deg2rad(yaw));

        m_ref_x = x;
        m_ref_y = y;

    } else {
        m_ref_x = prev_x[prev_size -1];
        m_ref_y = prev_y[prev_y.size() -1];

        prev_ref_x = prev_x[prev_size -2];
        prev_ref_y = prev_y[prev_size -2];
    }

    m_prev_x = m_ref_x;
    m_prev_y = m_ref_y;

    // push previous points point onto the spline vectors.
    m_spline_x.push_back(prev_ref_x);
    m_spline_y.push_back(prev_ref_y);
    m_spline_x.push_back(m_ref_x);
    m_spline_y.push_back(m_ref_y);

    m_state.process();


    m_trajectory.update(prev_x, prev_y, last_s, last_d);

    // set speed
    auto target_x = m_ref_x + 30;
    auto target_y = m_spline(target_x);
    auto target_distance = sqrt(pow(m_ref_x - target_x, 2) + pow(m_ref_y - target_y, 2));


    auto interval = target_distance / static_cast<int>(target_distance / (.02 * m_target_velocity / 2.24));
    while (!m_trajectory.is_full()) {
        m_ref_x += interval;
        m_ref_y = m_spline(m_ref_x);
        m_trajectory.add(m_ref_x, m_ref_y);
        if (m_max_velocity > m_target_velocity) {
            m_target_velocity += .2;
            interval = target_distance / static_cast<int>(target_distance / (.02 * m_target_velocity / 2.24));
        }
    }
}

void Vehicle::updateSpline() {
    // TODO: make this dry between the different states.
    double wp_s = m_map.s[NextWaypoint(m_x, m_y, deg2rad(m_yaw), m_map.x, m_map.y)];
    auto xy = getXY(wp_s, m_d, m_map.s, m_map.x, m_map.y);

    while (xy[0] < m_ref_x) {
        wp_s = m_map.getNextWaypointByS(wp_s)[0];
        xy = getXY(wp_s, m_d, m_map.s, m_map.x, m_map.y);
    }

    m_spline_x.push_back(xy[0]);
    m_spline_y.push_back(xy[1]);

    for (int i = 0; i < 5; ++i) {
        wp_s = m_map.getNextWaypointByS(wp_s)[0];
        xy = getXY(wp_s, m_d, m_map.s, m_map.x, m_map.y);
        m_spline_x.push_back(xy[0]);
        m_spline_y.push_back(xy[1]);
    }

    m_spline = tk::spline();
    m_spline.set_points(m_spline_x, m_spline_y);
}


void Vehicle::keepLane() {
    cout << "running state keeping lane" << endl;

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
            updateSpline();
            cout << "keeping lane: " << m_lane << endl;
        }
    } else {
        updateSpline();
        cout << "keeping lane: " << m_lane << endl;
    }
}

void Vehicle::changeLaneLeft() {
    cout << "running state changing left" << endl;
    // TODO: consolidate or extract method.
    const auto lane_tolerance = .1;
    auto ideal_d = getIdealD();

    cout << "left d " << m_d << endl;
    if (m_last_d > ideal_d - lane_tolerance and m_last_d < ideal_d + lane_tolerance)
        m_state.fire(KeepLane);

}

void Vehicle::changeLaneRight() {
    cout << "running state changing right" << endl;
    const auto lane_tolerance = .1;
    auto ideal_d = getIdealD();

    cout << "left d " << m_d << endl;
    if (m_d > ideal_d - lane_tolerance and m_d < ideal_d + lane_tolerance)
        m_state.fire(KeepLane);
}

void Vehicle::changeLeftEntry() {
    m_lane--;

    m_d = getIdealD();

    updateSpline();

    cout << "running entry of change left state to lane: " << m_lane << endl;
}

void Vehicle::changeRightEntry() {
    m_lane++;
    m_d = getIdealD();

    updateSpline();
    cout << "running entry of change right state to lane: " << m_lane << endl;
}


