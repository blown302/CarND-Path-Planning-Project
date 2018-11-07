#include <utility>

//
// Created by Thomas Milas on 11/1/18.
//

#include "vehicle.h"

using namespace std;

void Vehicle::update(double x, double y, double yaw, vector<double> &prev_x, vector<double> &prev_y, double last_s, double last_d, Map &map, vector<vector<double>> sensor_fusion) {
    m_x = x;
    m_y = y;
    m_yaw = yaw;
    m_sensor_fusion = std::move(sensor_fusion);

    m_state.process();

    cout << "using lane: " << m_lane << endl;

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
    cout << "running state keeping lane" << endl;
    auto lane = m_planner.determineBestTrajectory(m_sensor_fusion, m_x, m_y);
    auto delta = lane - m_lane;
    if (delta == 1) {
        m_state.fire(ChangeLaneRight);
    } else if (delta == -1) {
        m_state.fire(ChangeLaneLeft);
    }
}

void Vehicle::changeLaneLeft() {
    cout << "running state changing left" << endl;

}

void Vehicle::changeLaneRight() {
    cout << "running state changing right" << endl;

}

void Vehicle::changeLeftEntry() {
    cout << "running entry of change left state" << endl;
    m_lane--;
}

void Vehicle::changeRightEntry() {
    cout << "running entry of change right state" << endl;
    m_lane++;
}


