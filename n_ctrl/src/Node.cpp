// Copyright by BeeX [2026]

#include <n_check/Scene.h>
#include <n_ctrl/Node.h>
#include <n_kine/Fk.h>

#include <n_driver/Time.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace ctrl {
namespace {

const char *kFrame = "arm_base";

const char *kVolNs[check::N_VOLS] = {"upper_arm",  "forearm",    "wrist_mount",
                                     "palm",       "blade_left", "blade_right"};

const int kVolLink[check::N_VOLS] = {check::UPPER_ARM, check::FOREARM, check::WRIST_MOUNT,
                                     check::PALM,      check::JAW,     check::JAW};

void aimZ(const kine::Vec3 &d, Msg_Marker &m) {
    const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    if (len < 1e-9) {
        m.pose.orientation.w = 1.0;
        return;
    }
    const double ux = d.x / len, uy = d.y / len, uz = d.z / len;
    if (uz < -0.999999) {
        m.pose.orientation.x = 1.0;
        return;
    }
    const double s = std::sqrt(2.0 * (1.0 + uz));
    m.pose.orientation.x = -uy / s;
    m.pose.orientation.y = ux / s;
    m.pose.orientation.w = 0.5 * s;
}

void paint(Msg_Marker &m, bool hit) {
    m.color.r = hit ? 0.90f : 0.20f;
    m.color.g = hit ? 0.10f : 0.80f;
    m.color.b = 0.20f;
    m.color.a = 0.35f;
}

void addCapsule(Msg_MarkerArray &out, const char *ns, const kine::Vec3 &a, const kine::Vec3 &b,
                double r, bool hit) {
    const kine::Vec3 d   = b - a;
    const double     len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);

    Msg_Marker cyl;
    cyl.ns                = ns;
    cyl.id                = 0;
    cyl.type              = Msg_Marker::CYLINDER;
    cyl.action            = Msg_Marker::ADD;
    cyl.pose.position.x   = 0.5 * (a.x + b.x);
    cyl.pose.position.y   = 0.5 * (a.y + b.y);
    cyl.pose.position.z   = 0.5 * (a.z + b.z);
    cyl.scale.x           = 2.0 * r;
    cyl.scale.y           = 2.0 * r;
    cyl.scale.z           = len;
    aimZ(d, cyl);
    paint(cyl, hit);
    out.markers.push_back(cyl);

    for (int e = 0; e < 2; ++e) {
        const kine::Vec3 &at = e == 0 ? a : b;

        Msg_Marker cap;
        cap.ns                 = ns;
        cap.id                 = 1 + e;
        cap.type               = Msg_Marker::SPHERE;
        cap.action             = Msg_Marker::ADD;
        cap.pose.position.x    = at.x;
        cap.pose.position.y    = at.y;
        cap.pose.position.z    = at.z;
        cap.pose.orientation.w = 1.0;
        cap.scale.x = cap.scale.y = cap.scale.z = 2.0 * r;
        paint(cap, hit);
        out.markers.push_back(cap);
    }
}

void addBlade(Msg_MarkerArray &out, const char *ns, const kine::Vec3 *pts, size_t from, size_t to,
              double r, bool hit) {
    Msg_Marker m;
    m.ns                 = ns;
    m.id                 = 0;
    m.type               = Msg_Marker::SPHERE_LIST;
    m.action             = Msg_Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = m.scale.y = m.scale.z = 2.0 * r;
    paint(m, hit);

    m.points.resize(to - from);
    for (size_t i = from; i < to; ++i) {
        m.points[i - from].x = pts[i].x;
        m.points[i - from].y = pts[i].y;
        m.points[i - from].z = pts[i].z;
    }
    out.markers.push_back(m);
}

}  // namespace

void Node::TopicSink::send(const kine::Joints &q) {
    Msg_Float32MultiArray msg;
    msg.data.resize(reach::N_JOINTS - 1);
    for (int j = 0; j < kine::DOF; ++j) {
        msg.data[WIRE_SLOT[j] - 1] = static_cast<float>(kine::toPubDeg(params, j, q[j]));
    }
    PUBLISH_ROS(pub_target_, msg);
}

void Node::TopicSink::release() {
    Srv_Trigger srv;
    CALL_SRV_ROS(srv_standby_, srv);
}

Node::Node(const Params &p, const kine::Params &arm, const check::Jaws &jaws,
           const reach::Limits &limits, const std::string &field_path)
        : p_(p), g_(arm), jaws_(jaws), limits_(limits), exec_(p_, sink_) {
    sink_.params = g_.params();

    INIT_ROS_PUBLISHER(sink_.pub_target_, Msg_Float32MultiArray, "cmd/joint_target", 1);
    INIT_ROS_SERVICE_CLIENT(sink_.srv_standby_, Srv_Trigger, "cmd/standby");

    INIT_ROS_PUBLISHER(pub_state_, Msg_UInt8, "ctrl/state", 1);
    INIT_ROS_PUBLISHER(pub_pose_, Msg_PoseArray, "ctrl/pose", 1);
    INIT_ROS_PUBLISHER(pub_body_, Msg_MarkerArray, "ctrl/collision_body", 1);
    INIT_ROS_PUBLISHER(pub_field_, Msg_UInt64, "ctrl/field", 1);
    INIT_ROS_SUBSCRIBER(sub_states_, "joint_states", 1, &Node::onStates);

    INIT_ROS_SERVICE_SERVER(srv_move_j_, "ctrl/move_j", &Node::onMoveJ);
    INIT_ROS_SERVICE_SERVER(srv_move_l_, "ctrl/move_l", &Node::onMoveL);
    INIT_ROS_SERVICE_SERVER(srv_move_j_rel_, "ctrl/move_j_rel", &Node::onMoveJRel);
    INIT_ROS_SERVICE_SERVER(srv_move_l_rel_, "ctrl/move_l_rel", &Node::onMoveLRel);
    INIT_ROS_SERVICE_SERVER(srv_move_q_, "ctrl/move_q", &Node::onMoveQ);
    INIT_ROS_SERVICE_SERVER(srv_move_grasp_, "ctrl/move_grasp", &Node::onMoveGrasp);
    INIT_ROS_SERVICE_SERVER(srv_stop_, "ctrl/stop", &Node::onStop);
    INIT_ROS_SERVICE_SERVER(srv_return_, "ctrl/return", &Node::onReturn);
    INIT_ROS_SERVICE_SERVER(srv_rest_, "ctrl/rest", &Node::onRest);
    INIT_ROS_SERVICE_SERVER(srv_load_field_, "ctrl/load_field", &Node::onLoadField);

    if (!g_.ok()) {
        LOG_ERROR("[ctrl] the arm geometry is unusable (%s); every move will be refused",
                  g_.fault());
    }

    loadField(field_path);
}

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
            check::openScene(path, jaws_, g_, restPose(g_.params(), limits_), field, body);

    bool usable = field.ok();
    for (size_t i = 0; i < notes.size(); ++i) {
        switch (notes[i].level) {
        case check::Note::ERROR:
            LOG_ERROR("[ctrl] %s", notes[i].text.c_str());
            usable = false;
            break;
        case check::Note::WARN:
            LOG_WARN("[ctrl] %s", notes[i].text.c_str());
            break;
        default:
            break;
        }
    }
    return usable;
}

void Node::onStates(const sensor_msgs::JointState::ConstPtr &msg) {
    kine::Joints q;
    if (!readJointState(g_.params(), *msg, q)) {
        return;
    }

    // Age the state from the message timestamp, not from the moment it arrived.
    const double lag = (ROS_TIME_NOW() - msg->header.stamp).toSec();

    std::lock_guard<std::mutex> lock(state_mtx_);
    q_ = q;
    last_state_s_ = reach::nowSec() - (lag > 0.0 ? lag : 0.0);
    seen_ = true;
}

void Node::tick() {
    std::lock_guard<std::mutex> work(work_mtx_);

    Msg_UInt64 field_msg;
    field_msg.data = field_.ok() ? field_.digest() : 0;
    PUBLISH_ROS(pub_field_, field_msg);

    const double now = reach::nowSec();

    kine::Joints q;
    double       age = 0.0;
    {
        std::lock_guard<std::mutex> lock(state_mtx_);
        if (!seen_) {
            return;
        }
        q   = q_;
        age = now - last_state_s_;
    }

    if (age > p_.feedback_timeout_s) {
        exec_.blind();
    } else {
        exec_.measure(q);
    }
    const State s = exec_.tick(now);

    Msg_UInt8 state_msg;
    state_msg.data = static_cast<uint8_t>(s);
    PUBLISH_ROS(pub_state_, state_msg);
    publishPose(q);
    publishBody(q);

    if (retracing_ && s == State::REACHED) {
        trail_.clear();
        retracing_ = false;
    }

    if (s == logged_) {
        return;
    }
    logged_ = s;

    const kine::Vec3 at = kine::forward(g_, q).throat;
    switch (s) {
    case State::REACHED:
        LOG_INFO("[ctrl] reached (%.3f, %.3f, %.3f)", at.x, at.y, at.z);
        break;
    case State::STALLED:
        LOG_ERROR("[ctrl] STALLED — did not arrive within %.1f s, released to standby at "
                  "(%.3f, %.3f, %.3f)", p_.arrival_timeout_s, at.x, at.y, at.z);
        break;
    case State::PILLOW: {
        const int j = exec_.pillowJoint();
        trail_.stoppedAfter(exec_.issued(), snapToWindow(g_, q, p_.goal_tolerance_deg));
        LOG_ERROR("[ctrl] PILLOW — %s stopped following while still being driven, so the arm "
                  "has hit something unmapped. Released at (%.3f, %.3f, %.3f); ctrl/return "
                  "will back out the way it came.",
                  j >= 0 ? reach::NAME[WIRE_SLOT[j]] : "a joint", at.x, at.y, at.z);
        break;
    }
    default:
        break;
    }
}

void Node::publishPose(const kine::Joints &q) {
    if (!g_.ok()) {
        return;
    }
    const kine::Pose  p = kine::forward(g_, q);
    const kine::Vec3 pts[6] = {p.shoulder, p.elbow, p.wrist, p.mount, p.throat, p.tip};

    Msg_PoseArray msg;
    msg.header.stamp    = ROS_TIME_NOW();
    msg.header.frame_id = kFrame;
    msg.poses.resize(6);
    for (int i = 0; i < 6; ++i) {
        msg.poses[i].position.x    = pts[i].x;
        msg.poses[i].position.y    = pts[i].y;
        msg.poses[i].position.z    = pts[i].z;
        msg.poses[i].orientation.w = 1.0;
    }
    PUBLISH_ROS(pub_pose_, msg);
}

void Node::publishBody(const kine::Joints &q) {
    if (!g_.ok() || !body_) {
        return;
    }

    check::Body::Volume v;
    body_->volume(g_, q, viz_scratch_, v);

    const int hit = field_.empty() ? -1 : check::firstBlocked(field_, v, false);

    Msg_MarkerArray  msg;
    const kine::Vec3 *axis[4][2] = {{&v.shoulder, &v.elbow},
                                    {&v.elbow, &v.wrist},
                                    {&v.wrist, &v.mount},
                                    {&v.mount, &v.palm_end}};
    for (int i = 0; i < 4; ++i) {
        addCapsule(msg, kVolNs[i], *axis[i][0], *axis[i][1],
                   std::max(field_.radius(kVolLink[i]), 0.002), hit == i);
    }

    if (v.blades != NULL && v.count > 0) {
        const double r = std::max(field_.radius(check::JAW), 0.002);
        addBlade(msg, kVolNs[check::V_BLADE_LEFT], v.blades, 0, v.left, r,
                 hit == check::V_BLADE_LEFT);
        addBlade(msg, kVolNs[check::V_BLADE_RIGHT], v.blades, v.left, v.count, r,
                 hit == check::V_BLADE_RIGHT);
    }

    for (size_t i = 0; i < msg.markers.size(); ++i) {
        msg.markers[i].header.stamp    = ROS_TIME_NOW();
        msg.markers[i].header.frame_id = kFrame;
    }
    PUBLISH_ROS(pub_body_, msg);
}

bool Node::snapshot(kine::Joints &q) {
    std::lock_guard<std::mutex> lock(state_mtx_);
    q = snapToWindow(g_, q_, p_.goal_tolerance_deg);
    return seen_;
}

void Node::run(const Path &path, const kine::Joints &from, Move &out, bool grip, bool record) {
    if (exec_.busy()) {
        out.status = Status::BUSY;
        out.note   = reason(Status::BUSY);
        return;
    }

    std::string  why;
    const Status s = admit(g_, p_, path, field_, *body_, grip, scratch_, why);
    if (s != Status::OK) {
        out.status = s;
        out.note   = why;
        return;
    }

    if (!exec_.load(path)) {
        out.status = Status::EMPTY;
        out.note   = reason(Status::EMPTY);
        return;
    }

    if (record) {
        if (!trail_.recording()) {
            trail_.start(from);
        }
        trail_.add(path);
    }

    out.status     = Status::OK;
    out.waypoints  = path.size();
    out.duration_s = static_cast<double>(path.size()) / p_.rate_hz;
}

Move Node::moveJoints(const kine::Joints &goal, bool record) {
    Move         out;
    kine::Joints q;
    if (!snapshot(q)) {
        out.note = "no joint_states yet, so the arm's position is unknown";
        return out;
    }
    if (!g_.ok()) {
        out.note = g_.fault();
        return out;
    }

    out.from = kine::forward(g_, q).throat;
    out.to   = kine::forward(g_, goal).throat;

    Path path;
    if (rrt_on_ && record && body_) {
        rrt::Stats        st;
        const rrt::Result r = rrt::plan(g_, *body_, field_, rrt_, p_.floor_z_m, q, goal, scratch_,
                                        path, st);
        if (r != rrt::Result::OK) {
            out.status = r == rrt::Result::GOAL_FLOOR ? Status::FLOOR
                         : r == rrt::Result::GOAL_OUTSIDE || r == rrt::Result::START_OUTSIDE
                                 ? Status::LIMIT
                                 : Status::OBSTACLE;
            out.note = std::string("rrt* found no route: ") + rrt::name(r);
            return out;
        }
        LOG_INFO("[ctrl] rrt* %s: %u corners, %d iterations, %u nodes, %u waypoints, %.3f s",
                 st.direct ? "straight" : "tree", static_cast<uint32_t>(st.corners),
                 st.iterations, static_cast<uint32_t>(st.nodes),
                 static_cast<uint32_t>(path.size()), st.total_s);
    } else {
        planJoint(p_, q, goal, path);
    }
    run(path, q, out, false, record);
    return out;
}

Move Node::moveGrasp(const Leg &leg) {
    Move         out;
    kine::Joints q;
    if (!snapshot(q)) {
        out.note = "no joint_states yet, so the arm's position is unknown";
        return out;
    }
    if (!g_.ok()) {
        out.note = g_.fault();
        return out;
    }

    out.from = kine::forward(g_, q).throat;
    out.to   = leg.target;

    Path         path;
    double       dev = 0.0;
    const Status s   = planLine(g_, p_, q, leg, path, dev);
    if (s != Status::OK) {
        out.status = s;
        out.note   = reason(s);
        return out;
    }

    char buf[112];
    std::snprintf(buf, sizeof(buf), "within %.2e m of the line, wrist held at %.2f deg", dev,
                  kine::rad2deg(leg.q_wrist));
    out.note = buf;

    run(path, q, out, false);
    return out;
}

Move Node::move(const kine::Vec3 &v, bool straight, bool relative) {
    Move         out;
    kine::Joints q;
    if (!snapshot(q)) {
        out.note = "no joint_states yet, so the arm's position is unknown";
        return out;
    }
    if (!g_.ok()) {
        out.note = "the arm geometry is not solvable";
        return out;
    }

    out.from = kine::forward(g_, q).throat;
    out.to   = relative ? kine::Vec3{out.from.x + v.x, out.from.y + v.y, out.from.z + v.z} : v;

    Path path;
    if (straight) {
        double       dev = 0.0;
        const Status s   = planLine(g_, p_, q, out.to, path, dev);
        if (s != Status::OK) {
            out.status = s;
            out.note   = reason(s);
            return out;
        }
        char buf[96];
        std::snprintf(buf, sizeof(buf), "within %.2e m of the line", dev);
        out.note = buf;
    } else {
        kine::Joints goal;
        const Status s = solveTarget(g_, q, out.to, goal);
        if (s != Status::OK) {
            out.status = s;
            out.note   = reason(s);
            return out;
        }
        planJoint(p_, q, goal, path);
    }

    run(path, q, out, false);
    return out;
}

void Node::report(const Move &m, const char *what) {
    if (!m.ok()) {
        LOG_WARN("[ctrl] %s REFUSED: (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f). %s. "
                 "Throat reach is %.3f .. %.3f m.",
                 what, m.from.x, m.from.y, m.from.z, m.to.x, m.to.y, m.to.z, m.note.c_str(),
                 g_.reachMin(g_.throatAlong()), g_.reachMax(g_.throatAlong()));
        return;
    }
}

bool Node::handle(Srv_SetFloat32Array_Request &req,
                  Srv_SetFloat32Array_Response &res,
                  bool straight,
                  bool relative,
                  const char *what) {
    std::lock_guard<std::mutex> work(work_mtx_);

    if (req.data.size() != 3) {
        LOG_WARN("[ctrl] %s wants 3 values (%s, metres), got %u", what,
                 relative ? "dx dy dz" : "x y z", static_cast<uint32_t>(req.data.size()));
        res.success = false;
        return true;
    }

    const Move m = move({req.data[0], req.data[1], req.data[2]}, straight, relative);
    report(m, what);
    res.success = m.ok();
    return true;
}

bool Node::onMoveJ(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    return handle(req, res, false, false, "move_j");
}

bool Node::onMoveL(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    return handle(req, res, true, false, "move_l");
}

bool Node::onMoveJRel(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    return handle(req, res, false, true, "move_j_rel");
}

bool Node::onMoveLRel(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    return handle(req, res, true, true, "move_l_rel");
}

bool Node::onMoveQ(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    kine::Joints goal;
    std::string  why;
    if (!decodeJoints(req.data, goal, why)) {
        LOG_WARN("[ctrl] move_q: %s", why.c_str());
        res.success = false;
        return true;
    }

    const Move m = moveJoints(goal);
    report(m, "move_q");
    res.success = m.ok();
    return true;
}

bool Node::onMoveGrasp(Srv_SetFloat32Array_Request &req, Srv_SetFloat32Array_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    Leg         leg;
    std::string why;
    if (!decodeLeg(req.data, leg, why)) {
        LOG_WARN("[ctrl] move_grasp: %s", why.c_str());
        res.success = false;
        return true;
    }

    const Move m = moveGrasp(leg);
    report(m, "move_grasp");
    res.success = m.ok();
    return true;
}

bool Node::onStop(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    const bool was = exec_.busy();
    exec_.abort();
    trail_.clear();
    retracing_ = false;
    LOG_WARN("[ctrl] stop — %s", was ? "trajectory abandoned" : "nothing was running");
    res.success = true;
    res.message = was ? "aborted" : "nothing running";
    return true;
}

bool Node::onReturn(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    Move out;
    if (trail_.size() < 2) {
        LOG_WARN("[ctrl] return: no outbound path is recorded");
        res.success = false;
        res.message = "nothing recorded";
        return true;
    }

    kine::Joints q;
    if (!snapshot(q)) {
        res.success = false;
        res.message = "no joint_states yet";
        return true;
    }
    out.from        = kine::forward(g_, q).throat;
    const Path home = trail_.back();
    out.to          = kine::forward(g_, home.back()).throat;

    run(home, q, out, true, false);
    report(out, "return");
    retracing_ = out.ok();

    res.success = out.ok();
    res.message = out.ok() ? "retracing" : out.note;
    return true;
}

bool Node::onRest(Srv_Trigger_Request & /*req*/, Srv_Trigger_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    const Move m = moveJoints(restPose(g_.params(), limits_), false);
    report(m, "rest");
    retracing_ = m.ok();

    res.success = m.ok();
    res.message = m.ok() ? "returning to rest" : m.note;
    return true;
}

bool Node::onLoadField(Srv_SetString_Request &req, Srv_SetString_Response &res) {
    std::lock_guard<std::mutex> work(work_mtx_);

    res.success = false;
    if (exec_.busy() || !trail_.empty()) {
        LOG_WARN("[ctrl] load_field REFUSED: %s, and it was checked against the field already "
                 "loaded. Finish with ctrl/return or ctrl/stop first.",
                 exec_.busy() ? "a move is running" : "the way home is still recorded");
        return true;
    }

    check::Field                 field;
    std::unique_ptr<check::Body> body;
    if (!openField(req.data, field, body)) {
        LOG_WARN("[ctrl] load_field REFUSED: %s is not usable, so %s stays loaded",
                 req.data.c_str(), field_.ok() ? field_.source().c_str() : "no field");
        return true;
    }

    std::swap(field_, field);
    body_.swap(body);
    LOG_INFO("[ctrl] field now %s, digest %016llx", field_.source().c_str(),
             static_cast<unsigned long long>(field_.digest()));
    res.success = true;
    return true;
}

}
