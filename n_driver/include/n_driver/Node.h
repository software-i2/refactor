// Copyright by BeeX [2026]

#ifndef N_DRIVER_NODE_H
#define N_DRIVER_NODE_H

#include <n_driver/Arm.h>
#include <n_driver/Params.h>
#include <bx_msgs/RosBindings.hpp>
#include <sensor_msgs/JointState.h>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

namespace reach {

class Node {
public:
    Node(std::shared_ptr<Arm> arm, const Params &p);

    void tick();

private:
    DECLARE_ROS_PUBLISHER(pub_status_, Msg_GripperStatus)
    DECLARE_ROS_PUBLISHER(pub_joints_, Msg_Float32MultiArray)
    DECLARE_ROS_PUBLISHER(pub_states_, sensor_msgs::JointState)

    DECLARE_ROS_SUBSCRIBER(sub_target_, Msg_Float32MultiArray)

    DECLARE_ROS_SERVICE_SERVER(srv_position_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_velocity_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_jaw_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_open_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_close_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_rest_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_standby_, Srv_Trigger)

    // Streamed target for the four rotary joints, jaw untouched.
    void onTarget(const Msg_Float32MultiArray_ConstPtr &msg);

    bool onPosition(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onVelocity(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onJaw(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onOpen(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onClose(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onRest(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStandby(Srv_Trigger_Request &req, Srv_Trigger_Response &res);

    bool moveTo(uint32_t j, float pos, std::string &msg);
    void publish();
    void log();

    // Velocity commands run until stopped, so an unrefreshed jog is zeroed.
    void jogWatchdog();

    void rampTick();
    void cancelRamp(uint32_t j);
    void cancelRamp();

    std::shared_ptr<Arm> arm_;
    Params               p_;

    // Callbacks run on the spinner threads, tick() on the main loop. Everything
    // below is shared between them.
    std::mutex mtx_;

    SafeArray<ros::Time, N_JOINTS> read_at_;
    SafeArray<float, N_JOINTS>     pos_;
    SafeArray<uint8_t, N_JOINTS>   mode_;
    SafeArray<uint8_t, N_JOINTS>   logged_mode_;
    SafeArray<Climate, N_JOINTS>   climate_;
    bool                         climate_seen_   = false;
    bool                         warned_no_pose_ = false;

    uint32_t ticks_      = 0;
    double   climate_at_ = 0.0;
    double   log_at_     = 0.0;

    SafeArray<float, N_JOINTS>   goal_;
    SafeArray<uint8_t, N_JOINTS> ramp_;
    bool                         ramping_ = false;

    std::atomic<bool>   jogging_{false};
    std::atomic<double> jog_at_{0.0};
};

}  // namespace reach

#endif  // N_DRIVER_NODE_H
