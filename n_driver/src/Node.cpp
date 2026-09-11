// Copyright by BeeX [2026]

#include <n_driver/Node.h>
#include <n_driver/Time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace reach {

Node::Node(std::shared_ptr<Arm> arm, const Params &p) : arm_(std::move(arm)), p_(p) {
    INIT_ROS_PUBLISHER(pub_status_, Msg_GripperStatus, "reach/gripper_status", 10);
    INIT_ROS_PUBLISHER(pub_joints_, Msg_Float32MultiArray, "reach/joint_positions", 1);
    INIT_ROS_PUBLISHER(pub_states_, sensor_msgs::JointState, "joint_states", 1);

    INIT_ROS_SUBSCRIBER(sub_target_, "cmd/joint_target", 1, &Node::onTarget);

    INIT_ROS_SERVICE_SERVER(srv_position_, "cmd/set_position", &Node::onPosition);
    INIT_ROS_SERVICE_SERVER(srv_velocity_, "cmd/set_velocity", &Node::onVelocity);
    INIT_ROS_SERVICE_SERVER(srv_jaw_, "cmd/set_jaw", &Node::onJaw);
    INIT_ROS_SERVICE_SERVER(srv_open_, "cmd/open_jaw", &Node::onOpen);
    INIT_ROS_SERVICE_SERVER(srv_close_, "cmd/close_jaw", &Node::onClose);
    INIT_ROS_SERVICE_SERVER(srv_rest_, "cmd/rest", &Node::onRest);
    INIT_ROS_SERVICE_SERVER(srv_standby_, "cmd/standby", &Node::onStandby);

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        goal_[j] = 0.0f;
        ramp_[j] = 0;
    }

    log_at_ = nowSec();
}

void Node::tick() {
    std::lock_guard<std::mutex> lock(mtx_);

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        float pos = 0.0f;
        if (arm_->position(j, pos)) {
            pos_[j]     = pos;
            read_at_[j] = ROS_TIME_NOW();
        }
    }

    // Update one joint mode per tick and refresh climate when the interval expires.
    const uint32_t slot = ticks_++ % N_JOINTS;
    mode_[slot]         = static_cast<uint8_t>(arm_->mode(slot));

    const double now = nowSec();
    if (now - climate_at_ >= p_.climate_period_s) {
        climate_at_ = now;
        climate_seen_ |= arm_->climate(slot, climate_[slot]);
    }

    rampTick();
    jogWatchdog();
    publish();
    log();
}

void Node::rampTick() {
    bool moving = false;
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        if (!ramp_[j]) {
            continue;
        }

        const float step = static_cast<float>(p_.limits.max_vel[j] / p_.rate_hz);
        const float gap  = goal_[j] - pos_[j];
        if (std::fabs(gap) <= step) {
            arm_->move(j, goal_[j]);
            ramp_[j] = 0;
            continue;
        }

        arm_->move(j, pos_[j] + (gap > 0.0f ? step : -step));
        moving = true;
    }

    ramping_ = moving;
}

void Node::cancelRamp(uint32_t j) { ramp_[j] = 0; }

void Node::cancelRamp() {
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        ramp_[j] = 0;
    }
    ramping_ = false;
}

void Node::jogWatchdog() {
    if (!jogging_ || nowSec() - jog_at_ < p_.jog_timeout_s) {
        return;
    }
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        arm_->jog(j, 0.0f);
    }
    jogging_ = false;
    LOG_WARN("[arm] jog not refreshed for %.1f s, joints stopped", p_.jog_timeout_s);
}

void Node::publish() {
    Msg_GripperStatus status;
    status.gripper_position_array.resize(N_JOINTS);
    status.gripper_mode_array.resize(N_JOINTS);
    if (climate_seen_) {
        status.gripper_status_array.resize(N_JOINTS);
    }

    const uint64_t stamp = static_cast<uint64_t>(ROS_TIME_NOW_TO_SEC() * 1000.0);
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        status.gripper_position_array[j] = pos_[j];
        status.gripper_mode_array[j]     = mode_[j];
        if (climate_seen_) {
            status.gripper_status_array[j].time_stamp_ms = stamp;
            status.gripper_status_array[j].temperature   = climate_[j].temp;
            status.gripper_status_array[j].pressure      = climate_[j].pressure;
            status.gripper_status_array[j].humidity      = climate_[j].humidity;
        }
    }
    PUBLISH_ROS(pub_status_, status);

    // Stamped with the oldest reading it carries, not with now: a joint whose
    // serial read timed out is still in here, and consumers time out on this.
    ros::Time oldest = read_at_[0];
    for (uint32_t j = 1; j < N_JOINTS; ++j) {
        oldest = std::min(oldest, read_at_[j]);
    }
    if (oldest.isZero()) {
        if (!warned_no_pose_) {
            warned_no_pose_ = true;
            for (uint32_t j = 0; j < N_JOINTS; ++j) {
                if (read_at_[j].isZero()) {
                    LOG_WARN("[arm] %s has not answered yet, so no joint_states are published "
                             "and every move will be refused", NAME[j]);
                }
            }
        }
        return;
    }
    warned_no_pose_ = false;

    // Wire units, vendor convention: radians, and metres for the jaw.
    Msg_Float32MultiArray joints;
    sensor_msgs::JointState states;
    joints.data.resize(N_JOINTS);
    states.name.resize(N_JOINTS);
    states.position.resize(N_JOINTS);
    states.header.stamp = oldest;

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        const float wire = toWireRad(j, pos_[j]) * (j == JAW ? 0.001f : 1.0f);
        joints.data[j]     = wire;
        states.name[j]     = URDF_NAME[j];
        states.position[j] = wire;
    }
    PUBLISH_ROS(pub_joints_, joints);
    PUBLISH_ROS(pub_states_, states);
}

void Node::log() {
    bool changed = false;
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        changed |= mode_[j] != logged_mode_[j];
    }

    const double now = nowSec();
    if (!changed && now - log_at_ < p_.log_period_s) {
        return;
    }

    std::string line;
    char        buf[64];
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        std::snprintf(buf, sizeof(buf), "%s=%.2f  ", NAME[j], pos_[j]);
        line += buf;
    }
    line += "modes=";
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        std::snprintf(buf, sizeof(buf), "%02X ", mode_[j]);
        line += buf;
    }
    logged_mode_ = mode_;
    log_at_      = now;
}

bool Node::moveTo(uint32_t j, float pos, std::string &msg) {
    char buf[96];
    if (!p_.limits.inRange(j, pos)) {
        std::snprintf(buf, sizeof(buf), "%s target %.2f outside [%.2f, %.2f]",
                      NAME[j], pos, p_.limits.min_pos[j], p_.limits.max_pos[j]);
        msg = buf;
        return false;
    }

    cancelRamp(j);
    arm_->move(j, pos);
    std::snprintf(buf, sizeof(buf), "%s -> %.2f (from %.2f)", NAME[j], pos, pos_[j]);
    msg = buf;
    return true;
}

// wrist, elbow, shoulder, base_rot. One bad value drops the whole message.
void Node::onTarget(const Msg_Float32MultiArray_ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (msg->data.size() != N_JOINTS - 1) {
        LOG_WARN("[arm] joint_target wants %u values, got %u",
                 static_cast<uint32_t>(N_JOINTS - 1), static_cast<uint32_t>(msg->data.size()));
        return;
    }

    for (uint32_t j = 1; j < N_JOINTS; ++j) {
        if (!p_.limits.inRange(j, msg->data[j - 1])) {
            LOG_WARN("[arm] %s target %.2f outside [%.2f, %.2f]",
                     NAME[j], msg->data[j - 1], p_.limits.min_pos[j], p_.limits.max_pos[j]);
            return;
        }
    }

    cancelRamp();
    for (uint32_t j = 1; j < N_JOINTS; ++j) {
        arm_->move(j, msg->data[j - 1]);
    }
    jogging_ = false;
}

// Every joint at once, in published units. Nothing moves unless all five are legal.
bool Node::onPosition(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    res.success = false;
    if (req.data.size() != N_JOINTS) {
        LOG_WARN("[arm] set_position wants %u values, got %u",
                 N_JOINTS, static_cast<uint32_t>(req.data.size()));
        return true;
    }

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        if (!p_.limits.inRange(j, req.data[j])) {
            LOG_WARN("[arm] %s target %.2f outside [%.2f, %.2f]",
                     NAME[j], req.data[j], p_.limits.min_pos[j], p_.limits.max_pos[j]);
            return true;
        }
    }

    cancelRamp();
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        arm_->move(j, req.data[j]);
    }
    jogging_    = false;  // a position target leaves velocity mode
    res.success = true;
    return true;
}

// Keeps running until re-sent, zeroed, or the watchdog fires.
bool Node::onVelocity(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    res.success = false;
    if (req.data.size() != N_JOINTS) {
        LOG_WARN("[arm] set_velocity wants %u values, got %u",
                 static_cast<uint32_t>(N_JOINTS), static_cast<uint32_t>(req.data.size()));
        return true;
    }

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        if (!p_.limits.velOk(j, req.data[j])) {
            LOG_WARN("[arm] %s velocity %.2f over the %.2f/s cap",
                     NAME[j], req.data[j], p_.limits.max_vel[j]);
            return true;
        }
    }

    cancelRamp();
    bool moving = false;
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        arm_->jog(j, req.data[j]);
        moving |= req.data[j] != 0.0f;
    }

    jog_at_     = nowSec();
    jogging_    = moving;
    res.success = true;
    return true;
}

bool Node::onJaw(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    if (req.data.size() != 1) {
        LOG_WARN("[arm] set_jaw wants 1 value, got %u", static_cast<uint32_t>(req.data.size()));
        res.success = false;
        return true;
    }

    std::string msg;
    res.success = moveTo(JAW, req.data[0], msg);
    return true;
}

bool Node::onOpen(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    res.success = moveTo(JAW, static_cast<float>(p_.jaw_open_mm), res.message);
    return true;
}

bool Node::onClose(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    res.success = moveTo(JAW, p_.limits.min_pos[JAW], res.message);
    return true;
}

bool Node::onRest(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    // The ramp steps out from the measured position, so a joint that has never
    // reported one would be ramped from a standing zero.
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        if (read_at_[j].isZero()) {
            LOG_WARN("[arm] rest refused: %s has not reported a position", NAME[j]);
            res.success = false;
            res.message = "not every joint has reported a position yet";
            return true;
        }
    }

    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        goal_[j] = p_.limits.rest_pos[j];
        ramp_[j] = 1;
    }
    ramping_    = true;
    jogging_    = false;
    res.success = true;
    res.message = "resting";
    return true;
}

bool Node::onStandby(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> lock(mtx_);

    cancelRamp();
    for (uint32_t j = 0; j < N_JOINTS; ++j) {
        arm_->standby(j);
    }
    jogging_ = false;
    LOG_WARN("[arm] released to standby");
    res.success = true;
    res.message = "standby";
    return true;
}

}  // namespace reach
