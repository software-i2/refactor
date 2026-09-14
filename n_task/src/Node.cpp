// Copyright by BeeX [2026]

#include <n_check/Scene.h>
#include <n_kine/Fk.h>
#include <n_ctrl/Path.h>
#include <n_task/Node.h>

#include <n_driver/Time.h>

#include <cmath>
#include <cstdio>

namespace task {

const char *name(Step s) {
    switch (s) {
    case Step::IDLE:
        return "idle";
    case Step::TO_STANDOFF:
        return "moving to the standoff";
    case Step::AT_STANDOFF:
        return "at the standoff";
    case Step::ADVANCING:
        return "closing in on the handle";
    case Step::HOLDING:
        return "holding";
    default:
        return "failed";
    }
}

Node::Node(const Params &p, const ctrl::Params &motion, const kine::Params &arm,
           const check::Jaws &jaws, const reach::Limits &limits, const std::string &field_path)
        : p_(p), motion_(motion), g_(arm), jaws_(jaws), limits_(limits) {

    INIT_ROS_PUBLISHER(pub_step_, Msg_UInt8, "task/step", 1);
    INIT_ROS_PUBLISHER(pub_chosen_, Msg_PoseArray, "task/chosen", 1);
    INIT_ROS_SUBSCRIBER(sub_states_, "joint_states", 1, &Node::onStates);
    INIT_ROS_SUBSCRIBER(sub_ctrl_, "ctrl/state", 10, &Node::onCtrlState);

    INIT_ROS_SERVICE_SERVER(srv_plan_, "task/plan", &Node::onPlan);
    INIT_ROS_SERVICE_SERVER(srv_preview_, "task/preview", &Node::onPreview);
    INIT_ROS_SERVICE_SERVER(srv_start_, "task/start", &Node::onStart);
    INIT_ROS_SERVICE_SERVER(srv_pick_, "task/pick", &Node::onPick);
    INIT_ROS_SERVICE_SERVER(srv_advance_, "task/advance", &Node::onAdvance);
    INIT_ROS_SERVICE_SERVER(srv_close_, "task/close_jaw", &Node::onClose);
    INIT_ROS_SERVICE_SERVER(srv_open_, "task/open_jaw", &Node::onOpen);
    INIT_ROS_SERVICE_SERVER(srv_home_, "task/home", &Node::onHome);
    INIT_ROS_SERVICE_SERVER(srv_stop_, "task/stop", &Node::onStop);

    INIT_ROS_SERVICE_CLIENT(cli_move_q_, Srv_SetFloat32Array, "ctrl/move_q");
    INIT_ROS_SERVICE_CLIENT(cli_move_grasp_, Srv_SetFloat32Array, "ctrl/move_grasp");
    INIT_ROS_SERVICE_CLIENT(cli_return_, Srv_Trigger, "ctrl/return");
    INIT_ROS_SERVICE_CLIENT(cli_ctrl_stop_, Srv_Trigger, "ctrl/stop");
    INIT_ROS_SERVICE_CLIENT(cli_close_jaw_, Srv_Trigger, "cmd/close_jaw");
    INIT_ROS_SERVICE_CLIENT(cli_open_jaw_, Srv_Trigger, "cmd/open_jaw");

    if (!g_.ok()) {
        LOG_ERROR("[task] the arm geometry is unusable (%s); no hold can be planned",
                  g_.fault());
    }

    loadField(field_path);
}

// The task and controller use the same field loader; mismatches show up in logs.
void Node::loadField(const std::string &path) {
    const std::vector<check::Note> notes =
            check::openScene(path, jaws_, g_, ctrl::restPose(g_.params(), limits_), field_, body_);

    for (size_t i = 0; i < notes.size(); ++i) {
        switch (notes[i].level) {
        case check::Note::ERROR:
            LOG_ERROR("[task] %s", notes[i].text.c_str());
            break;
        case check::Note::WARN:
            LOG_WARN("[task] %s", notes[i].text.c_str());
            break;
        default:
            break;
        }
    }
}

void Node::onStates(const sensor_msgs::JointState::ConstPtr &msg) {
    kine::Joints q;
    if (!ctrl::readJointState(g_.params(), *msg, q)) {
        return;  // not the driver's joint_states
    }

    std::lock_guard<std::mutex> lock(state_mtx_);
    q_ = q;
    seen_ = true;
}

void Node::onCtrlState(const Msg_UInt8::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    ctrl_state_ = static_cast<ctrl::State>(msg->data);
    ctrl_seen_  = true;
    ctrl_at_    = reach::nowSec();
    if (ctrl_state_ == ctrl::State::APPROACHING || ctrl_state_ == ctrl::State::SETTLING) {
        ctrl_busy_ = true;
    }
}

bool Node::snapshot(kine::Joints &q) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    q = q_;
    return seen_;
}

void Node::enter(Step s) {
    const bool changed = step_ != s;
    step_   = s;
    leg_at_ = reach::nowSec();
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        ctrl_busy_ = false;
    }
    if (changed) {
        LOG_INFO("[task] state -> %s", name(s));
    }
}

bool Node::plan(const std::vector<float> &data, std::string &why) {
    kine::Joints seed;
    if (!snapshot(seed)) {
        why = "no joint_states yet, so there is nothing to plan from";
        return false;
    }

    std::vector<Candidate> candidates;
    if (!readCandidates(data, candidates, why)) {
        return false;
    }
    if (!why.empty()) {
        LOG_WARN("[task] %s", why.c_str());  // the 6-or-9 ambiguity
        why.clear();
    }

    // Solve every candidate first; ranking is based on the resulting postures.
    last_       = choose(g_, *body_, field_, p_.ask, motion_, candidates, seed, scratch_);
    candidates_ = candidates;

    report();

    if (!last_.found) {
        why      = summarise(last_, candidates_);
        planned_ = false;
        return false;
    }

    hold_    = last_.hold;
    planned_ = true;
    why      = summarise(last_, candidates_);
    return true;
}

void Node::publishChosen() {
    Msg_PoseArray msg;
    msg.header.stamp    = ROS_TIME_NOW();
    msg.header.frame_id = "arm_base";
    if (!planned_) {
        PUBLISH_ROS(pub_chosen_, msg);  // empty clears the last one
        return;
    }

    // Rotate +x onto the approach to draw a direction arrow.
    const kine::Vec3 a = kine::unit(hold_.approach);
    const kine::Vec3 axis{0.0, -a.z, a.y};              // cross({1,0,0}, a)
    const double     s = kine::norm(axis);
    const double     c = a.x;

    double qx = 0.0, qy = 0.0, qz = 0.0, qw = 1.0;
    if (s > 1e-9) {
        const double half = std::atan2(s, c) * 0.5;
        const double k    = std::sin(half) / s;
        qx = axis.x * k;
        qy = axis.y * k;
        qz = axis.z * k;
        qw = std::cos(half);
    } else if (c < 0.0) {
        qy = 1.0;                                        // a points at -x
        qw = 0.0;
    }

    const kine::Vec3 at[2] = {hold_.standoff_point, hold_.point};
    msg.poses.resize(2);
    for (int i = 0; i < 2; ++i) {
        msg.poses[i].position.x    = at[i].x;
        msg.poses[i].position.y    = at[i].y;
        msg.poses[i].position.z    = at[i].z;
        msg.poses[i].orientation.x = qx;
        msg.poses[i].orientation.y = qy;
        msg.poses[i].orientation.z = qz;
        msg.poses[i].orientation.w = qw;
    }
    PUBLISH_ROS(pub_chosen_, msg);
}

void Node::report() {
    publishChosen();

    if (last_.found && !field_.ok()) {
        LOG_WARN("[task] no obstacle field loaded, so nothing here was tested against the "
                 "world -- these verdicts are reach, limits and the floor only.");
    }
}

bool Node::moveToPose(const kine::Joints &goal, std::string &why) {
    Srv_SetFloat32Array srv;
    srv.request.data = ctrl::encodeJoints(goal);

    if (!CALL_SRV_ROS(cli_move_q_, srv)) {
        why = "ctrl/move_q did not answer";
        return false;
    }
    if (!srv.response.success) {
        why = "the move was refused; n_ctrl's log says why";
        return false;
    }
    return true;
}

// Everything but the target is the same for every leg of one grasp: the branch
// and the roll are what the gate solved, and holding them is the point.
bool Node::moveAlong(const kine::Vec3 &to, std::string &why) {
    ctrl::Leg leg;
    leg.start    = hold_.standoff_point;
    leg.target   = to;
    leg.q_wrist  = hold_.joints[kine::WRIST];
    leg.elbow_up = hold_.elbow_up;

    Srv_SetFloat32Array srv;
    srv.request.data = ctrl::encodeLeg(leg);

    if (!CALL_SRV_ROS(cli_move_grasp_, srv)) {
        why = "ctrl/move_grasp did not answer";
        return false;
    }
    if (!srv.response.success) {
        why = "the move was refused; n_ctrl's log says why";
        return false;
    }
    return true;
}

bool Node::jaw(bool shut, std::string &why) {
    Srv_Trigger         srv;
    ROS_SERVICE_CLIENT &cli = shut ? cli_close_jaw_ : cli_open_jaw_;
    if (!CALL_SRV_ROS(cli, srv)) {
        why = shut ? "cmd/close_jaw did not answer" : "cmd/open_jaw did not answer";
        return false;
    }
    if (!srv.response.success) {
        why = srv.response.message.empty() ? "the jaw command was refused"
                                           : srv.response.message;
        return false;
    }
    return true;
}

bool Node::goStandoff(std::string &why) {
    if (!planned_) {
        why = "nothing planned; call task/plan first";
        return false;
    }
    if (!jaw(false, why)) {
        return false;
    }
    // Armed before the call, or a leg n_ctrl finishes quickly reports REACHED
    // before this step starts watching and the arrival is never seen.
    enter(Step::TO_STANDOFF);
    // The posture, not the point: this is the pose the gate cleared.
    if (!moveToPose(hold_.standoff, why)) {
        enter(Step::FAILED);
        return false;
    }
    return true;
}

bool Node::goGrasp(std::string &why) {
    if (!planned_) {
        why = "nothing planned; call task/plan first";
        return false;
    }
    // Straight in, so the jaws travel down the approach instead of swinging
    // through the handle on an arc, and on the branch and roll the gate solved.
    enter(Step::ADVANCING);
    if (!moveAlong(hold_.point, why)) {
        enter(Step::FAILED);
        return false;
    }
    return true;
}

void Node::tick() {
    std::lock_guard<std::mutex> work(work_mtx_);

    Msg_UInt8 msg;
    msg.data = static_cast<uint8_t>(step_);
    PUBLISH_ROS(pub_step_, msg);

    const bool running = step_ == Step::TO_STANDOFF || step_ == Step::ADVANCING;

    ctrl::State ctrl_state = ctrl::State::IDLE;
    bool        ctrl_seen  = false;
    bool        ctrl_busy  = false;
    double      ctrl_at    = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        ctrl_state = ctrl_state_;
        ctrl_seen  = ctrl_seen_;
        ctrl_busy  = ctrl_busy_;
        ctrl_at    = ctrl_at_;
    }

    if (!running || !ctrl_seen) {
        return;
    }

    const double now = reach::nowSec();

    if (now - ctrl_at > p_.ctrl_silence_s) {
        LOG_ERROR("[task] n_ctrl has not reported for %.1f s; giving up on the leg",
                  p_.ctrl_silence_s);
        enter(Step::FAILED);
        return;
    }

    // ctrl/state is only published from n_ctrl's tick, so until it says it is
    // moving, what it reports still describes the leg before this one.
    if (!ctrl_busy) {
        if (now - leg_at_ > p_.ctrl_silence_s) {
            LOG_ERROR("[task] n_ctrl never took the leg; it still reports %s",
                      ctrl::name(ctrl_state));
            enter(Step::FAILED);
        }
        return;
    }

    // n_ctrl owns whether a leg finished; this only reacts to what it reports.
    if (ctrl_state == ctrl::State::STALLED || ctrl_state == ctrl::State::PILLOW
        || ctrl_state == ctrl::State::ABORTED || ctrl_state == ctrl::State::IDLE) {
        LOG_ERROR("[task] the arm stopped mid-leg (%s), so the pick is off. The outbound path is "
                  "kept: task/home will back out.", ctrl::name(ctrl_state));
        enter(Step::FAILED);
        return;
    }
    if (ctrl_state != ctrl::State::REACHED) {
        return;
    }

    std::string why;
    switch (step_) {
    case Step::TO_STANDOFF:
        enter(Step::AT_STANDOFF);
        if (p_.auto_sequence && !goGrasp(why)) {
            LOG_ERROR("[task] advance: %s", why.c_str());
            enter(Step::FAILED);
        }
        break;

    case Step::ADVANCING:
        enter(Step::HOLDING);
        if (p_.auto_sequence && !jaw(true, why)) {
            LOG_ERROR("[task] close: %s", why.c_str());
            enter(Step::FAILED);
        }
        break;

    default:
        break;
    }
}

bool Node::onPlan(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = plan(req.data, why);
    if (!res.success) {
        LOG_WARN("[task] plan: %s", why.c_str());
    }
    return true;
}

// The evaluation without the commitment. SetFloat32Array carries no message
// back, so the summary comes through here instead of out of task/plan.
bool Node::onPreview(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    if (candidates_.empty()) {
        res.success = false;
        res.message = "nothing to preview; send candidates to task/plan first";
        return true;
    }

    report();
    res.success = last_.found;
    res.message = summarise(last_, candidates_);
    return true;
}

bool Node::onPick(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    if (!plan(req.data, why)) {
        LOG_WARN("[task] pick: %s", why.c_str());
        res.success = false;
        return true;
    }
    res.success = goStandoff(why);
    if (!res.success) {
        LOG_WARN("[task] pick: %s", why.c_str());
    }
    return true;
}

// Everything after a preview. task/pick still exists for the one-shot, but once
// you have looked at a plan you should not have to hand it the candidates again
// just to say yes.
bool Node::onStart(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = goStandoff(why);
    res.message = res.success ? "moving to the standoff" : why;
    if (!res.success) {
        LOG_WARN("[task] start: %s", why.c_str());
    }
    return true;
}

bool Node::onAdvance(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = goGrasp(why);
    res.message = res.success ? "closing in on the handle" : why;
    if (!res.success) {
        LOG_WARN("[task] advance: %s", why.c_str());
    }
    return true;
}

bool Node::onClose(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = jaw(true, why);
    res.message = res.success ? "jaw closing" : why;
    if (res.success) {
        enter(Step::HOLDING);
    }
    return true;
}

bool Node::onOpen(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = jaw(false, why);
    res.message = res.success ? "jaw opening" : why;
    return true;
}

bool Node::onHome(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    Srv_Trigger srv;
    res.success = CALL_SRV_ROS(cli_return_, srv) && srv.response.success;
    res.message = res.success ? "retracing the way out" : "ctrl/return refused";
    if (res.success) {
        enter(Step::IDLE);
        planned_ = false;
    }
    return true;
}

bool Node::onStop(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    Srv_Trigger srv;
    CALL_SRV_ROS(cli_ctrl_stop_, srv);
    enter(Step::IDLE);
    planned_    = false;
    res.success = true;
    res.message = "stopped";
    LOG_WARN("[task] stopped; the plan is dropped");
    return true;
}

}  // namespace task
