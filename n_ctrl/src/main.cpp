// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <n_conf/Doc.h>
#include <n_ctrl/Node.h>

#include <cstdio>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);

    INIT_ROS_NODE("n_ctrl", 0, "n_ctrl/alive")

    std::string config;
    GET_ROS_PARAM("~config", config, config);
    if (config.empty()) {
        LOG_ERROR("[ctrl] no ~config given. Point it at n_conf/config/arm.yaml.");
        return -1;
    }

    conf::Doc doc;
    doc.load(config, {"world", "arm", "jaws", "ctrl"}, {"driver"});

    ctrl::Params  p;
    kine::Params  arm;
    check::Jaws   jaws;
    reach::Limits limits;
    p.load(doc);
    arm.load(doc);
    jaws.load(doc);
    limits.load(doc);
    const std::string field_path = doc.text("world.obstacle_field");

    if (!doc.ok()) {
        LOG_ERROR("[ctrl] %s is not usable:\n%s", config.c_str(), doc.report().c_str());
        return -1;
    }
    if (p.missing() != NULL) {
        LOG_ERROR("[ctrl] %s: %s is missing or not a usable value", config.c_str(), p.missing());
        return -1;
    }

    std::string rrt_config;
    GET_ROS_PARAM("~rrt_config", rrt_config, rrt_config);
    rrt::Settings rrt_settings;
    if (!rrt_config.empty()) {
        conf::Doc rrt_doc;
        rrt_doc.load(rrt_config, {"rrt"});
        rrt_settings.load(rrt_doc);
        if (!rrt_doc.ok() || rrt_settings.missing() != NULL) {
            LOG_ERROR("[ctrl] %s is not usable (%s):\n%s", rrt_config.c_str(),
                      rrt_settings.missing() != NULL ? rrt_settings.missing() : "see below",
                      rrt_doc.report().c_str());
            return -1;
        }
        rrt_settings.check_step_deg = p.max_joint_step_deg;
    }

    LOG_INFO("[ctrl] config loaded: %s", config.c_str());

    ctrl::Node node(p, arm, jaws, limits, field_path);
    if (!rrt_config.empty()) {
        node.useRrt(rrt_settings);
        LOG_WARN("[ctrl] RRT* trial on: move_q plans with RRT*%s (%s)",
                 rrt_settings.standoff ? "" : ", no standoff",
                 rrt_config.c_str());
    }
    LOG_INFO("[ctrl] node started");

    // Run ROS callbacks and the control loop concurrently.
    ROS_ASYNC_SPIN(2)

    ros::Rate loop(p.rate_hz);
    while (IS_ROS_NODE_OK()) {
        node.tick();
        loop.sleep();
    }

    ROS_SHUTDOWN();
    return 0;
}
