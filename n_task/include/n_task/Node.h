// Copyright by BeeX [2026]

#ifndef N_TASK_NODE_H
#define N_TASK_NODE_H

#include <bx_msgs/RosBindings.hpp>
#include <n_ctrl/Bridge.h>
#include <n_ctrl/Exec.h>
#include <n_task/Params.h>
#include <n_task/FSM.h>
#include <n_task/Pick.h>
#include <n_task/PickRrt.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include <memory>
#include <mutex>
#include <string>

namespace task {

class Node {
public:
    Node(const Params &p, const ctrl::Params &motion, const kine::Params &arm,
         const check::Jaws &jaws, const reach::Limits &limits, const std::string &field_path);

    void tick();
    void useRrt(const rrt::Settings &s);

private:
    void onStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onCtrlState(const Msg_UInt8::ConstPtr &msg);
    void onFrame(const std_msgs::String::ConstPtr &msg);
    void onCtrlField(const Msg_UInt64::ConstPtr &msg);

    bool onPlan(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onPlanLive(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onPreview(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStart(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onPick(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onAdvance(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onClose(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onOpen(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onHome(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStop(Srv_Trigger_Request &req, Srv_Trigger_Response &res);

    bool plan(const std::vector<float> &data, std::string &why);
    bool planStep(const std::vector<float> &data, std::string &why);
    bool planLive(const std::string &frame, std::string &msg);
    void report();
    void publishChosen();

    bool startPick(std::string &why);
    bool perform(Action a, Step arm, std::string &why);
    bool sameWorld(std::string &why);

    bool moveToPose(const kine::Joints &goal, std::string &why);
    bool moveAlong(const kine::Vec3 &to, std::string &why);
    bool jaw(bool shut, std::string &why);
    void watchGrip();
    Sense sense();
    void enter(Step s);
    bool snapshot(kine::Joints &q);
    void loadField(const std::string &path);
    bool openField(const std::string &path, check::Field &field,
                   std::unique_ptr<check::Body> &body);

    DECLARE_ROS_SUBSCRIBER(sub_states_, sensor_msgs::JointState)
    DECLARE_ROS_SUBSCRIBER(sub_ctrl_, Msg_UInt8)
    DECLARE_ROS_SUBSCRIBER(sub_frame_, std_msgs::String)
    DECLARE_ROS_SUBSCRIBER(sub_ctrl_field_, Msg_UInt64)
    DECLARE_ROS_PUBLISHER(pub_step_, std_msgs::String)
    DECLARE_ROS_PUBLISHER(pub_chosen_, Msg_PoseArray)
    DECLARE_ROS_PUBLISHER(pub_grip_, Msg_UInt8)
    DECLARE_ROS_PUBLISHER(pub_planned_, std_msgs::String)

    DECLARE_ROS_SERVICE_SERVER(srv_plan_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_plan_live_, Srv_Trigger)
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
    DECLARE_ROS_SERVICE_CLIENT(cli_load_field_, Srv_SetString)

    Params        p_;
    ctrl::Params  motion_;
    kine::Geom    g_;
    check::Jaws   jaws_;
    reach::Limits limits_;

    check::Field                 field_;
    std::unique_ptr<check::Body> body_;
    std::vector<kine::Vec3>      scratch_;

    rrt::Settings rrt_;
    bool          rrt_on_ = false;

    // Service callbacks run on the spinner threads, tick() on the main loop.
    // work_mtx_ covers the plan and the step machine; state_mtx_ only the
    // feedback the subscribers write. Always taken in that order.
    std::mutex   work_mtx_;
    std::mutex   state_mtx_;
    kine::Joints q_{};
    bool         seen_ = false;
    double       jaw_mm_ = 0.0;
    ros::Time    jaw_at_;
    std::string  frame_;
    uint64_t     ctrl_field_      = 0;
    bool         ctrl_field_seen_ = false;

    check::Hold hold_;
    bool        planned_ = false;

    Choice                 last_;
    std::vector<Candidate> candidates_;

    Step   step_       = Step::IDLE;
    double entered_at_ = 0.0;
    bool   carrying_   = false;

    Grip      grip_ = Grip::NONE;
    ros::Time grip_after_;
    ros::Time jaw_cmd_at_;
    double    still_mm_ = 0.0;
    ros::Time still_at_;

    ctrl::State ctrl_state_ = ctrl::State::IDLE;
    bool        ctrl_seen_  = false;
    double      ctrl_at_    = 0.0;
    bool        ctrl_busy_  = false;
};

}  // namespace task

#endif  // N_TASK_NODE_H
