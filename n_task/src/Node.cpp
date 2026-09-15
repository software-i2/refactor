// Copyright by BeeX [2026]

#include <n_check/Scene.h>
#include <n_kine/Fk.h>
#include <n_ctrl/Path.h>
#include <n_task/Node.h>

#include <n_driver/Time.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <utility>

namespace task {
namespace {

bool readFloats(const std::string &path, std::vector<float> &out, std::string &why) {
    std::FILE *f = std::fopen(path.c_str(), "r");
    if (f == NULL) {
        why = "cannot open " + path;
        return false;
    }
    std::string text;
    char        buf[4096];
    size_t      n = 0;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) {
        text.append(buf, n);
    }
    std::fclose(f);

    for (size_t i = 0; i < text.size(); ++i) {
        if (text[i] == '[' || text[i] == ']' || text[i] == ',') {
            text[i] = ' ';
        }
    }

    out.clear();
    const char *p = text.c_str();
    for (;;) {
        char        *end = NULL;
        const double v   = std::strtod(p, &end);
        if (end == p) {
            break;
        }
        out.push_back(static_cast<float>(v));
        p = end;
    }
    while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') {
        ++p;
    }
    if (*p != '\0') {
        why = path + " is not a list of numbers";
        return false;
    }
    return true;
}

}  // namespace

Node::Node(const Params &p, const ctrl::Params &motion, const kine::Params &arm,
           const check::Jaws &jaws, const reach::Limits &limits, const std::string &field_path)
        : p_(p), motion_(motion), g_(arm), jaws_(jaws), limits_(limits) {

    INIT_ROS_PUBLISHER(pub_step_, std_msgs::String, "task/step", 1);
    INIT_ROS_PUBLISHER(pub_chosen_, Msg_PoseArray, "task/chosen", 1);
    INIT_ROS_PUBLISHER(pub_grip_, Msg_UInt8, "task/grip", 1);
    INIT_ROS_PUBLISHER(pub_planned_, std_msgs::String, "task/planned_frame", 1);
    INIT_ROS_SUBSCRIBER(sub_states_, "joint_states", 1, &Node::onStates);
    INIT_ROS_SUBSCRIBER(sub_ctrl_, "ctrl/state", 10, &Node::onCtrlState);
    INIT_ROS_SUBSCRIBER(sub_frame_, "live/frame", 1, &Node::onFrame);
    INIT_ROS_SUBSCRIBER(sub_ctrl_field_, "ctrl/field", 1, &Node::onCtrlField);

    INIT_ROS_SERVICE_SERVER(srv_plan_, "task/plan", &Node::onPlan);
    INIT_ROS_SERVICE_SERVER(srv_plan_live_, "task/plan_live", &Node::onPlanLive);
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
    INIT_ROS_SERVICE_CLIENT(cli_load_field_, Srv_SetString, "ctrl/load_field");

    if (!g_.ok()) {
        LOG_ERROR("[task] the arm geometry is unusable (%s); no hold can be planned",
                  g_.fault());
    }

    loadField(field_path);
}

// The task and controller use the same field loader; mismatches show up in logs.
void Node::loadField(const std::string &path) {
    openField(path, field_, body_);
}

void Node::useRrt(const rrt::Settings &s) {
    std::lock_guard<std::mutex> work(work_mtx_);
    rrt_    = s;
    rrt_on_ = true;
}

bool Node::openField(const std::string &path, check::Field &field,
                     std::unique_ptr<check::Body> &body) {
    const std::vector<check::Note> notes =
            check::openScene(path, jaws_, g_, ctrl::restPose(g_.params(), limits_), field, body);

    bool usable = field.ok();
    for (size_t i = 0; i < notes.size(); ++i) {
        switch (notes[i].level) {
        case check::Note::ERROR:
            LOG_ERROR("[task] %s", notes[i].text.c_str());
            usable = false;
            break;
        case check::Note::WARN:
            LOG_WARN("[task] %s", notes[i].text.c_str());
            break;
        default:
            break;
        }
    }
    return usable;
}

void Node::onFrame(const std_msgs::String::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    frame_ = msg->data;
}

void Node::onCtrlField(const Msg_UInt64::ConstPtr &msg) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    ctrl_field_      = msg->data;
    ctrl_field_seen_ = true;
}

void Node::onStates(const sensor_msgs::JointState::ConstPtr &msg) {
    kine::Joints q;
    if (!ctrl::readJointState(g_.params(), *msg, q)) {
        return;  // not the driver's joint_states
    }

    std::lock_guard<std::mutex> lock(state_mtx_);
    q_ = q;
    seen_ = true;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        if (msg->name[i] == reach::URDF_NAME[reach::JAW]) {
            jaw_mm_ = msg->position[i] * 1000.0;
            jaw_at_ = msg->header.stamp;
            break;
        }
    }
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
    step_       = s;
    entered_at_ = reach::nowSec();
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        ctrl_busy_ = false;
    }
    if (changed) {
        std_msgs::String msg;
        msg.data = name(s);
        PUBLISH_ROS(pub_step_, msg);
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
    last_       = rrt_on_ ? chooseRrt(g_, *body_, field_, p_.ask, motion_, rrt_, candidates, seed,
                                      scratch_)
                          : choose(g_, *body_, field_, p_.ask, motion_, candidates, seed, scratch_);
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

bool Node::planStep(const std::vector<float> &data, std::string &why) {
    if (!accepts(step_, Command::PLAN, why)) {
        return false;
    }
    const Step from = step_;
    enter(afterCommand(step_, Command::PLAN));
    const bool planned = plan(data, why);
    if (!planned) {
        planned_ = false;
        publishChosen();
    }
    enter(afterPlan(from, planned));
    return planned;
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
    grip_       = shut ? Grip::CLOSING : Grip::NONE;
    grip_after_ = ROS_TIME_NOW();
    jaw_cmd_at_ = grip_after_;
    still_at_   = ros::Time();
    return true;
}

void Node::watchGrip() {
    double    mm = 0.0;
    ros::Time at;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        mm = jaw_mm_;
        at = jaw_at_;
    }
    if ((grip_ != Grip::CLOSING && grip_ != Grip::HELD) || at <= grip_after_) {
        return;
    }
    grip_after_ = at;

    const double shut = limits_.min_pos[reach::JAW];
    const bool   held = mm > shut + p_.jaw_held_mm;

    if (grip_ == Grip::HELD) {
        if (!held) {
            grip_ = Grip::EMPTY;
            LOG_WARN("[task] grip: LOST, the jaw closed to %.2f mm after holding", mm);
        }
        return;
    }

    if (still_at_.isZero() || std::fabs(mm - still_mm_) > p_.jaw_still_mm) {
        still_mm_ = mm;
        still_at_ = at;
        return;
    }
    if ((at - still_at_).toSec() < p_.jaw_settle_s) {
        return;
    }

    grip_ = held ? Grip::HELD : Grip::EMPTY;
    if (held) {
        LOG_INFO("[task] grip: holding, the jaw stopped at %.2f mm, %.2f mm short of closed",
                 mm, mm - shut);
    } else {
        LOG_WARN("[task] grip: EMPTY, the jaw closed to %.2f mm with nothing between the blades",
                 mm);
    }
}

Sense Node::sense() {
    Sense     in;
    ros::Time jaw_at;
    in.now_s     = reach::nowSec();
    in.entered_s = entered_at_;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        in.ctrl      = ctrl_state_;
        in.ctrl_seen = ctrl_seen_;
        in.ctrl_busy = ctrl_busy_;
        in.ctrl_at_s = ctrl_at_;
        in.jaw_mm    = jaw_mm_;
        jaw_at       = jaw_at_;
    }
    in.jaw_fresh     = jaw_at > jaw_cmd_at_;
    in.jaw_still     = !still_at_.isZero() && grip_after_ > still_at_;
    in.grip          = grip_;
    in.carrying      = carrying_;
    in.auto_sequence = p_.auto_sequence;
    return in;
}

bool Node::startPick(std::string &why) {
    if (!accepts(step_, Command::START, why) || !sameWorld(why) || !jaw(false, why)) {
        return false;
    }
    enter(afterCommand(step_, Command::START));
    return true;
}

bool Node::perform(Action a, Step arm, std::string &why) {
    switch (a) {
    case Action::MOVE_TO_STANDOFF:
        if (!sameWorld(why)) {
            return false;
        }
        // Armed before the call, or a leg n_ctrl finishes quickly reports REACHED
        // before this step starts watching and the arrival is never seen.
        enter(arm);
        // The posture, not the point: this is the pose the gate cleared.
        if (!moveToPose(hold_.standoff, why)) {
            enter(Step::FAILED);
            return false;
        }
        return true;

    case Action::MOVE_TO_HANDLE:
        if (!sameWorld(why)) {
            return false;
        }
        // Straight in, so the jaws travel down the approach instead of swinging
        // through the handle on an arc, and on the branch and roll the gate solved.
        enter(arm);
        if (!moveAlong(hold_.point, why)) {
            enter(Step::FAILED);
            return false;
        }
        return true;

    case Action::CLOSE_JAW:
        if (!jaw(true, why)) {
            return false;
        }
        enter(arm);
        return true;

    case Action::HOME: {
        const Step  from = step_;
        Srv_Trigger srv;
        enter(arm);
        if (!CALL_SRV_ROS(cli_return_, srv) || !srv.response.success) {
            why = srv.response.message.empty() ? "ctrl/return refused"
                                               : "ctrl/return refused: " + srv.response.message;
            enter(from);
            return false;
        }
        carrying_ = from == Step::HOLDING;
        planned_  = false;
        return true;
    }

    default:
        return true;
    }
}

void Node::tick() {
    std::lock_guard<std::mutex> work(work_mtx_);

    std_msgs::String msg;
    msg.data = name(step_);
    PUBLISH_ROS(pub_step_, msg);

    watchGrip();
    Msg_UInt8 grip;
    grip.data = static_cast<uint8_t>(grip_);
    PUBLISH_ROS(pub_grip_, grip);

    if (step_ == Step::IDLE || step_ == Step::CHOSEN) {
        return;
    }

    const Next n = next(step_, sense(), p_);

    if (n.action != Action::NONE) {
        std::string why;
        if (!perform(n.action, n.step, why)) {
            LOG_ERROR("[task] %s: %s", name(n.step), why.c_str());
            enter(Step::FAILED);
        }
        return;
    }
    if (n.step == step_) {
        return;
    }
    if (n.step == Step::FAILED) {
        LOG_ERROR("[task] %s", n.why.c_str());
    } else if (!n.why.empty()) {
        LOG_WARN("[task] %s", n.why.c_str());
    }
    enter(n.step);
}

bool Node::onPlan(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = planStep(req.data, why);
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
    res.success = planStep(req.data, why) && startPick(why);
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
    res.success = startPick(why);
    res.message = res.success ? "opening the jaw, then to the standoff" : why;
    if (!res.success) {
        LOG_WARN("[task] start: %s", why.c_str());
    }
    return true;
}

bool Node::onAdvance(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = accepts(step_, Command::ADVANCE, why)
                  && perform(Action::MOVE_TO_HANDLE, afterCommand(step_, Command::ADVANCE), why);
    res.message = res.success ? "closing in on the handle" : why;
    if (!res.success) {
        LOG_WARN("[task] advance: %s", why.c_str());
    }
    return true;
}

bool Node::onClose(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = accepts(step_, Command::CLOSE, why) && jaw(true, why);
    res.message = res.success ? "jaw closing" : why;
    if (res.success) {
        enter(afterCommand(step_, Command::CLOSE));
    }
    return true;
}

bool Node::onOpen(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = accepts(step_, Command::OPEN, why) && jaw(false, why);
    res.message = res.success ? "jaw opening" : why;
    if (res.success) {
        enter(afterCommand(step_, Command::OPEN));
    }
    return true;
}

bool Node::onHome(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    std::string why;
    res.success = accepts(step_, Command::HOME, why)
                  && perform(Action::HOME, afterCommand(step_, Command::HOME), why);
    res.message = res.success ? "retracing the way out" : why;
    return true;
}

bool Node::onStop(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    Srv_Trigger srv;
    const bool  halted = CALL_SRV_ROS(cli_ctrl_stop_, srv);
    enter(afterCommand(step_, Command::STOP));
    planned_    = false;
    res.success = halted;
    if (halted) {
        res.message = "stopped";
        LOG_WARN("[task] stopped; the plan is dropped");
    } else {
        res.message = "ctrl/stop did not answer; the arm may still be moving";
        LOG_ERROR("[task] %s", res.message.c_str());
    }
    return true;
}

bool Node::sameWorld(std::string &why) {
    uint64_t theirs = 0;
    bool     seen   = false;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        theirs = ctrl_field_;
        seen   = ctrl_field_seen_;
    }
    const uint64_t mine = field_.ok() ? field_.digest() : 0;
    if (!seen) {
        why = "n_ctrl has not said which field it checks against on ctrl/field";
        return false;
    }
    if (theirs != mine) {
        char buf[200];
        std::snprintf(buf, sizeof(buf),
                      "n_ctrl checks field %016llx but this plan was made against %016llx; "
                      "plan again",
                      static_cast<unsigned long long>(theirs), static_cast<unsigned long long>(mine));
        why = buf;
        return false;
    }
    return true;
}

bool Node::planLive(const std::string &frame, std::string &msg) {
    const Step        from = step_;
    const std::string id   = frame.substr(frame.find_last_of('/') + 1);
    const std::string path = frame + "/field.bin";

    std::string        why;
    std::vector<float> data;
    bool               loaded = readFloats(frame + "/candidates.txt", data, why);
    if (loaded && data.empty()) {
        why    = "n_live found no candidates within reach in this frame; nothing was loaded";
        loaded = false;
    }

    check::Field                 field;
    std::unique_ptr<check::Body> body;
    if (loaded && !openField(path, field, body)) {
        why    = "the field is not usable; the task log says why";
        loaded = false;
    }

    if (from == Step::CHOSEN) {
        kine::Joints           seed;
        std::vector<Candidate> candidates;
        if (loaded && !snapshot(seed)) {
            why    = "no joint_states yet, so there is nothing to plan from";
            loaded = false;
        }
        if (loaded && !readCandidates(data, candidates, why)) {
            loaded = false;
        }
        if (loaded) {
            const Choice trial =
                    rrt_on_ ? chooseRrt(g_, *body, field, p_.ask, motion_, rrt_, candidates, seed,
                                        scratch_)
                            : choose(g_, *body, field, p_.ask, motion_, candidates, seed, scratch_);
            loaded = trial.found;
            why    = summarise(trial, candidates);
        }
        if (!loaded) {
            msg = id + ": " + why + " Kept the hold already planned.";
            return false;
        }
    }

    if (loaded) {
        Srv_SetString srv;
        srv.request.data = path;
        if (!CALL_SRV_ROS(cli_load_field_, srv)) {
            why    = "ctrl/load_field did not answer";
            loaded = false;
        } else if (!srv.response.success) {
            why    = "n_ctrl kept its field, because a move is running or the way home is still "
                     "recorded (task/home first); its log says which";
            loaded = false;
        }
    }

    planned_ = false;
    last_    = Choice();
    candidates_.clear();
    publishChosen();

    bool planned = false;
    if (loaded) {
        enter(afterCommand(step_, Command::PLAN));
        std::swap(field_, field);
        body_.swap(body);
        planned = plan(data, why);
    }
    if (planned) {
        std_msgs::String kept;
        kept.data = frame;
        PUBLISH_ROS(pub_planned_, kept);
    }
    enter(afterPlan(from, planned));
    msg = id + ": " + why;
    return planned;
}

bool Node::onPlanLive(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    res.success = false;
    if (!accepts(step_, Command::PLAN, res.message)) {
        return true;
    }

    std::string frame;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        frame = frame_;
    }
    if (frame.empty()) {
        res.message = "nothing on live/frame yet; is n_live running?";
        return true;
    }

    res.success = planLive(frame, res.message) || (step_ == Step::CHOSEN && planned_);
    if (res.success) {
        LOG_INFO("[task] plan_live: %s", res.message.c_str());
    } else {
        LOG_WARN("[task] plan_live: %s", res.message.c_str());
    }
    return true;
}

}  // namespace task
