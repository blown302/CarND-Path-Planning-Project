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
    m_v = v;
    m_sensor_fusion = std::move(sensor_fusion);

    auto frenet = getFrenet(x, y, rad2deg(yaw), m_map.x, m_map.y);
    m_s = frenet[0];
    m_d = frenet[1];

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

    m_ref_theta = atan2(m_ref_y - prev_ref_y, m_ref_x - prev_ref_x);

    m_state.process();


    m_trajectory.update(prev_x, prev_y, last_s, last_d);

    // set speed
    auto x_dist = 30.;
    auto target_x = x_dist;
    auto target_y = m_spline(target_x);
    auto target_distance = sqrt(pow(target_x, 2) + pow(target_y, 2));


    auto points = (target_distance / (.02 * m_target_velocity / 2.24));
    auto interval = (target_x / points);
    double x_temp = 0., y_temp;

    while (!m_trajectory.is_full()) {
        x_temp += interval;
        y_temp = m_spline(x_temp);

        auto x_point = (x_temp * cos(m_ref_theta) - y_temp * sin(m_ref_theta));
        auto y_point = (x_temp * sin(m_ref_theta) + y_temp * cos(m_ref_theta));

        m_trajectory.add(m_ref_x + x_point, m_ref_y + y_point);

        cout << "updating trajectory with: " << m_override_speed << " speed override " << m_overridden_velocity << " target " << m_target_velocity << endl;

        if (m_override_speed) {
            if (m_overridden_velocity < m_target_velocity) {
                m_target_velocity -= .2;
                cout << "reducing speed to: " << m_target_velocity << endl;
                points = (target_distance / (.02 * m_target_velocity / 2.24));
                interval = x_dist / points;
            } else if (m_overridden_velocity > m_target_velocity) {
                m_target_velocity += .2;
                points = (target_distance / (.02 * m_target_velocity / 2.24));
                interval = x_dist / points;
            }
        } else {
            if (m_max_velocity > m_target_velocity ) {
                m_target_velocity += .2;
                // TODO: extract to be reused.
                points = (target_distance / (.02 * m_target_velocity / 2.24));
                interval = x_dist / points;
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
    cout << "running state keeping lane" << endl;

    auto trajectory_costs = m_planner.calculateTrajectoryCosts(m_sensor_fusion, m_s, m_target_velocity, m_lane);
    cout << "return size: " << trajectory_costs.size() << endl;
    cout << "calc'd trajectories: ";

    auto min = numeric_limits<double>::infinity();
    auto min_lane_id = m_lane;
    PossibleTrajectory trajectory{};
    for (auto &t: trajectory_costs) {
        cout << " reference cost " << t.cost << " override: " << t.trajectory.isOverrideSpeed() << endl;
        if (t.cost < min) {
            min = t.cost;
            min_lane_id = t.trajectory.getLaneId();
            trajectory = t.trajectory;
        }
    }

    if (trajectory.isOverrideSpeed()) {
        m_overridden_velocity = trajectory.getOverriddenSpeed();
        m_override_speed = true;
        cout << "speed overridden with: " << m_overridden_velocity << endl;
    } else {
        m_override_speed = false;
    }
    cout << "using lane: " << min_lane_id << " min: " << min << " speed: " << m_target_velocity << " is overridden " << trajectory.isOverrideSpeed() << endl;

    auto delta = min_lane_id - m_lane;
    if (delta > 0) {
        m_state.fire(ChangeLaneRight);
    } else if (delta < 0) {
        m_state.fire(ChangeLaneLeft);
    } else {
        updateSpline();
    }
}



void Vehicle::changeLaneLeft() {
    cout << "running state changing left" << endl;
    // TODO: consolidate or extract method.
    const auto lane_tolerance = .1;
    auto ideal_d = getIdealD();
    updateSpline();

    auto frenet = getFrenet(m_x, m_y, deg2rad(m_yaw), m_map.x, m_map.y);
    auto d = frenet[1];

    if (d > ideal_d - lane_tolerance and d < ideal_d + lane_tolerance)
        m_state.fire(KeepLane);

}

void Vehicle::changeLaneRight() {
    cout << "running state changing right" << endl;
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

    cout << "running entry of change left state to lane: " << m_lane << endl;
}

void Vehicle::changeRightEntry() {
    m_lane++;

    updateSpline();
    cout << "running entry of change right state to lane: " << m_lane << endl;
}



