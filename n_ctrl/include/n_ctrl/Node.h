// Copyright by BeeX [2026]

#ifndef N_CTRL_NODE_H
#define N_CTRL_NODE_H

#include <bx_msgs/RosBindings.hpp>
#include <n_ctrl/Bridge.h>
#include <n_ctrl/Exec.h>
#include <n_ctrl/Trail.h>
#include <n_check_rrt/Rrt.h>
#include <sensor_msgs/JointState.h>

#include <memory>
#include <mutex>
#include <string>

namespace ctrl {

class Node {
public:
    Node(const Params &p, const kine::Params &arm, const check::Jaws &jaws,
         const reach::Limits &limits, const std::string &field_path);

    void tick();
    void useRrt(const rrt::Settings &s);

private:
    class TopicSink : public Sink {
    public:
        void send(const kine::Joints &q) override;
        void release() override;

        DECLARE_ROS_PUBLISHER(pub_target_, Msg_Float32MultiArray)
        DECLARE_ROS_SERVICE_CLIENT(srv_standby_, Srv_Trigger)
        kine::Params params;
    };

    void onStates(const sensor_msgs::JointState::ConstPtr &msg);

    bool onMoveJ(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onMoveL(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onMoveJRel(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onMoveLRel(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onMoveQ(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onMoveGrasp(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res);
    bool onStop(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onReturn(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onRest(Srv_Trigger_Request &req, Srv_Trigger_Response &res);
    bool onLoadField(Srv_SetString_Request &req, Srv_SetString_Response &res);

    bool handle(Srv_SetFloat32Array_Request &req,
                Srv_SetFloat32Array_Response &res,
                bool straight,
                bool relative,
                const char *what);

    Move move(const kine::Vec3 &v, bool straight, bool relative);
    Move moveJoints(const kine::Joints &goal, bool record = true);
    Move moveGrasp(const Leg &leg);

    bool snapshot(kine::Joints &q);
    void run(const Path &path, const kine::Joints &from, Move &out, bool grip, bool record = true);
    void report(const Move &m, const char *what);
    void publishPose(const kine::Joints &q);
    void publishBody(const kine::Joints &q);

    DECLARE_ROS_SUBSCRIBER(sub_states_, sensor_msgs::JointState)
    DECLARE_ROS_PUBLISHER(pub_state_, Msg_UInt8)
    DECLARE_ROS_PUBLISHER(pub_pose_, Msg_PoseArray)
    DECLARE_ROS_PUBLISHER(pub_body_, Msg_MarkerArray)
    DECLARE_ROS_PUBLISHER(pub_field_, Msg_UInt64)
    DECLARE_ROS_SERVICE_SERVER(srv_move_j_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_move_l_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_move_j_rel_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_move_l_rel_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_move_q_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_move_grasp_, Srv_SetFloat32Array)
    DECLARE_ROS_SERVICE_SERVER(srv_stop_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_return_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_rest_, Srv_Trigger)
    DECLARE_ROS_SERVICE_SERVER(srv_load_field_, Srv_SetString)

    void loadField(const std::string &path);
    bool openField(const std::string &path, check::Field &field,
                   std::unique_ptr<check::Body> &body);

    Params        p_;
    kine::Geom    g_;
    check::Jaws   jaws_;
    reach::Limits limits_;

    check::Field                 field_;
    std::unique_ptr<check::Body> body_;
    std::vector<kine::Vec3>      scratch_;
    std::vector<kine::Vec3>      viz_scratch_;

    rrt::Settings rrt_;
    bool          rrt_on_ = false;

    TopicSink  sink_;
    Exec       exec_;
    Trail      trail_;

    // Service callbacks run on the spinner threads, tick() on the main loop.
    // work_mtx_ covers everything a move touches; state_mtx_ only the feedback
    // the subscriber writes. Always taken in that order.
    std::mutex   work_mtx_;
    std::mutex   state_mtx_;
    kine::Joints q_{};
    bool         seen_        = false;
    double       last_state_s_ = 0.0;

    State logged_    = State::IDLE;
    bool  retracing_ = false;
};

}

#endif
