#include <utility>

//
// Created by Thomas Milas on 11/1/18.
//

#include "vehicle.h"

using namespace std;

void Vehicle::update(double x, double y, double yaw, vector<double> &prev_x, vector<double> &prev_y, double last_s, double last_d, vector<vector<double>> sensor_fusion, double v) {
    m_last_s = last_s;
    m_yaw = yaw;
    m_x = x;
    m_y = y;
    m_v = v;
    m_sensor_fusion = std::move(sensor_fusion);

    auto frenet = getFrenet(x, y, rad2deg(yaw), m_map.x, m_map.y);
    m_s = frenet[0];

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

    // push previous points point onto the spline vectors.
    m_spline_x.push_back(prev_ref_x);
    m_spline_y.push_back(prev_ref_y);
    m_spline_x.push_back(m_ref_x);
    m_spline_y.push_back(m_ref_y);

    m_ref_theta = atan2(m_ref_y - prev_ref_y, m_ref_x - prev_ref_x);

    m_state.process();


    m_trajectory.update(prev_x, prev_y, last_s, last_d);

    // set speed
    const auto x_dist = 30.;
    auto target_x = x_dist;
    auto target_y = m_spline(target_x);
    auto target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));

    auto interval = getInterval(target_distance, x_dist);
    double x_temp = 0., y_temp;

    while (!m_trajectory.is_full()) {
        x_temp += interval;
        y_temp = m_spline(x_temp);

        auto x_point = (x_temp * cos(m_ref_theta) - y_temp * sin(m_ref_theta));
        auto y_point = (x_temp * sin(m_ref_theta) + y_temp * cos(m_ref_theta));

        m_trajectory.add(m_ref_x + x_point, m_ref_y + y_point);

        const auto target_velocity_increment = .2;
        if (m_override_speed) {
            if (m_overridden_velocity < m_target_velocity) {
                m_target_velocity -= target_velocity_increment;
                interval = getInterval(target_distance, x_dist);
            } else if (m_overridden_velocity > m_target_velocity) {
                m_target_velocity += target_velocity_increment;
                interval = getInterval(target_distance, x_dist);
            }
        } else {
            if (m_max_velocity > m_target_velocity ) {
                m_target_velocity += target_velocity_increment;
                interval = getInterval(target_distance, x_dist);
            }
        }
    }

    if (v > 70.) throw "too fast";
}

void Vehicle::updateSpline() {
    double wp_s;

    if (m_last_s == 0.) {
        wp_s = m_s;
    } else {
        wp_s = m_last_s;
    }

    for (int i = 0; i < 3; ++i) {
        wp_s += 30.;
        auto xy = getXY(wp_s, getIdealD(), m_map.s, m_map.x, m_map.y);
        m_spline_x.push_back(xy[0]);
        m_spline_y.push_back(xy[1]);
    }

    for (auto i = 0; i < m_spline_x.size(); ++i) {
        auto shift_x = m_spline_x[i] - m_ref_x;
        auto shift_y = m_spline_y[i] - m_ref_y;

        m_spline_x[i] = (shift_x * cos(-m_ref_theta) - shift_y * sin(-m_ref_theta));
        m_spline_y[i] = (shift_x * sin(-m_ref_theta) + shift_y * cos(-m_ref_theta));
    }

    m_spline = tk::spline();
    m_spline.set_points(m_spline_x, m_spline_y);
}


void Vehicle::keepLane() {
    auto trajectory_costs = m_planner.calculateTrajectoryCosts(m_sensor_fusion, m_s, m_target_velocity, m_lane);

    auto min = numeric_limits<double>::infinity();
    auto min_lane_id = m_lane;
    PossibleTrajectory trajectory{};
    for (auto &t: trajectory_costs) {
        if (t.cost < min) {
            min = t.cost;
            min_lane_id = t.trajectory.getLaneId();
            trajectory = t.trajectory;
        }
    }

    if (trajectory.isOverrideSpeed()) {
        m_overridden_velocity = trajectory.getOverriddenSpeed();
        m_override_speed = true;
    } else {
        m_override_speed = false;
    }

    auto delta = min_lane_id - m_lane;
    if (delta > 0) {
        m_state.fire(ChangeLaneRight);
    } else if (delta < 0) {
        m_state.fire(ChangeLaneLeft);
    } else {
        updateSpline();
    }
}

void Vehicle::changeLane() {
    const auto lane_tolerance = .1;
    auto ideal_d = getIdealD();

    updateSpline();

    auto frenet = getFrenet(m_x, m_y, deg2rad(m_yaw), m_map.x, m_map.y);
    auto d = frenet[1];

    if (d > ideal_d - lane_tolerance and d < ideal_d + lane_tolerance)
        m_state.fire(KeepLane);
}

void Vehicle::changeLeftEntry() {
    m_lane--;

    updateSpline();
}

void Vehicle::changeRightEntry() {
    m_lane++;

    updateSpline();
}

double Vehicle::getInterval(double target_distance, double x_dist) {
    const auto time_interval = .02;
    const auto conversion_to_meters_constant = 2.24;

    auto points = (target_distance / (time_interval * m_target_velocity / conversion_to_meters_constant));
    return x_dist / points;
}



