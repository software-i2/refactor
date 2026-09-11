// Copyright by BeeX [2026]

#ifndef N_TASK_NODE_H
#define N_TASK_NODE_H

#include <bx_msgs/RosBindings.hpp>
#include <n_ctrl/Bridge.h>
#include <n_ctrl/Exec.h>
#include <n_task/Params.h>
#include <n_task/Pick.h>
#include <sensor_msgs/JointState.h>

#include <memory>
#include <mutex>
#include <string>

namespace task {

enum class Step : uint8_t {
    IDLE = 0,
    TO_STANDOFF,
    AT_STANDOFF,
    ADVANCING,
    HOLDING,
    FAILED
};

const char *name(Step s);

class Node {
public:
    Node(const Params &p, const ctrl::Params &motion, const kine::Params &arm,
         const check::Jaws &jaws, const reach::Limits &limits, const std::string &field_path);

    void tick();

private:
    void onStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onCtrlState(const Msg_UInt8::ConstPtr &msg);

    bool onPlan(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onPreview(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStart(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onPick(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onAdvance(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onClose(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onOpen(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onHome(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStop(Srv_Trigger_Request &req, Srv_Trigger_Response &res);

    bool plan(const std::vector<float> &data, std::string &why);
    void report();
    void publishChosen();

    bool goStandoff(std::string &why);
    bool goGrasp(std::string &why);

    bool moveToPose(const kine::Joints &goal, std::string &why);
    bool moveAlong(const kine::Vec3 &to, std::string &why);
    bool jaw(bool shut, std::string &why);
    void enter(Step s);
    bool snapshot(kine::Joints &q);
    void loadField(const std::string &path);

    DECLARE_ROS_SUBSCRIBER(sub_states_, sensor_msgs::JointState)
    DECLARE_ROS_SUBSCRIBER(sub_ctrl_, Msg_UInt8)
    DECLARE_ROS_PUBLISHER(pub_step_, Msg_UInt8)
    DECLARE_ROS_PUBLISHER(pub_chosen_, Msg_PoseArray)

    DECLARE_ROS_SERVICE_SERVER(srv_plan_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_preview_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_start_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_pick_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_advance_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_close_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_open_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_home_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_stop_, Srv_Trigger)

    DECLARE_ROS_SERVICE_CLIENT(cli_move_q_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_CLIENT(cli_move_grasp_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_CLIENT(cli_return_, Srv_Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_ctrl_stop_, Srv_Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_close_jaw_, Srv_Trigger)
    DECLARE_ROS_SERVICE_CLIENT(cli_open_jaw_, Srv_Trigger)

    Params        p_;
    ctrl::Params  motion_;
    kine::Geom    g_;
    check::Jaws   jaws_;
    reach::Limits limits_;

    check::Field                 field_;
    std::unique_ptr<check::Body> body_;
    std::vector<kine::Vec3>      scratch_;

    // Service callbacks run on the spinner threads, tick() on the main loop.
    // work_mtx_ covers the plan and the step machine; state_mtx_ only the
    // feedback the subscribers write. Always taken in that order.
    std::mutex   work_mtx_;
    std::mutex   state_mtx_;
    kine::Joints q_{};
    bool         seen_ = false;

    check::Hold hold_;
    bool        planned_ = false;

    Choice                 last_;
    std::vector<Candidate> candidates_;

    Step   step_   = Step::IDLE;
    double leg_at_ = 0.0;

    ctrl::State ctrl_state_ = ctrl::State::IDLE;
    bool        ctrl_seen_  = false;
    double      ctrl_at_    = 0.0;
    bool        ctrl_busy_  = false;
};

}  // namespace task

#endif  // N_TASK_NODE_H
