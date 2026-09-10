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

// Where the pick has got to. Only ever advanced by a leg finishing.
enum class Step : uint8_t {
    IDLE = 0,
    TO_STANDOFF,
    AT_STANDOFF,
    ADVANCING,
    HOLDING,
    RETREATING,
    DONE,
    FAILED
};

const char *name(Step s);

class Node {
public:
    // Everything already loaded and checked by main, from the same file and the
    // same loaders n_ctrl uses.
    Node(const Params &p, const kine::Params &arm, const check::Jaws &jaws,
         const reach::Limits &limits, const std::string &field_path);

    void tick();

private:
    void onStates(const sensor_msgs::JointState::ConstPtr &msg);
    void onCtrlState(const Msg_UInt8::ConstPtr &msg);

    bool onPlan(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onPreview(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStart(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onPick(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onAdvance(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onRetreat(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onClose(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onOpen(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onHome(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onStop(Srv_Trigger_Request &req, Srv_Trigger_Response &res);

    // Planning only; nothing moves.
    bool plan(const std::vector<float> &data, std::string &why);

    // Logs the last evaluation: every candidate, then the summary.
    void report();

    // Draws the hold that won, so the choice is visible next to the candidates
    // it was chosen from rather than only described in a log line.
    void publishChosen();

    // Each returns false with a reason rather than throwing the arm at it.
    bool goStandoff(std::string &why);
    bool goGrasp(std::string &why);
    bool goStandoffBack(std::string &why);

    // The standoff leg goes to a posture, not a point: the gate solved a
    // specific branch and roll, and any other solution for the same point puts
    // the jaws somewhere the obstacle field never checked.
    bool moveToPose(const kine::Joints &goal, std::string &why);

    // The advance and retreat legs fly a line on that same branch and roll.
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
    DECLARE_ROS_SERVICE_SERVER(srv_retreat_, Srv_Trigger)
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
    kine::Geom    g_;
    check::Jaws   jaws_;
    reach::Limits limits_;

    // Built after the field: how finely the blades are sampled is fixed by the
    // field's voxel size rather than set by hand.
    check::Field                 field_;
    std::unique_ptr<check::Body> body_;
    std::vector<kine::Vec3>      scratch_;

    std::mutex   mtx_;
    kine::Joints q_{};
    bool         seen_ = false;

    check::Hold hold_;
    bool        planned_ = false;

    // The last evaluation, kept whole so preview can report it without solving
    // again. This is what says which candidates were live and why one won.
    Choice                 last_;
    std::vector<Candidate> candidates_;

    Step   step_       = Step::IDLE;
    double leg_began_s_ = 0.0;

    // What n_ctrl last reported, so a leg knows when it has finished.
    ctrl::State ctrl_state_ = ctrl::State::IDLE;
    bool        ctrl_seen_  = false;
};

}  // namespace task

#endif  // N_TASK_NODE_H
