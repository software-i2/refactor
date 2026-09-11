// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <n_conf/Doc.h>
#include <n_task/Node.h>

#include <cstdio>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);

    INIT_ROS_NODE("n_task", 0, "n_task/alive")

    std::string config;
    GET_ROS_PARAM("~config", config, config);
    if (config.empty()) {
        LOG_ERROR("[task] no ~config given. Point it at n_conf/config/arm.yaml.");
        return -1;
    }

    // The same file, the same `arm` and `jaws` sections, the same loader as
    // n_ctrl. That is what stops the two describing different arms.
    conf::Doc doc;
    doc.load(config, {"world", "arm", "jaws", "task"}, {"driver", "ctrl"});

    task::Params  p;
    ctrl::Params  motion;
    kine::Params  arm;
    check::Jaws   jaws;
    reach::Limits limits;
    p.load(doc);
    motion.load(doc);
    arm.load(doc);
    jaws.load(doc);
    limits.load(doc);
    const std::string field_path = doc.text("world.obstacle_field");

    if (!doc.ok()) {
        LOG_ERROR("[task] %s is not usable:\n%s", config.c_str(), doc.report().c_str());
        return -1;
    }
    if (p.missing() != NULL) {
        LOG_ERROR("[task] %s: %s is missing or not a usable value", config.c_str(), p.missing());
        return -1;
    }
    if (motion.missing() != NULL) {
        LOG_ERROR("[task] %s: %s is missing or not a usable value", config.c_str(),
                  motion.missing());
        return -1;
    }

    LOG_INFO("[task] config loaded: %s", config.c_str());

    task::Node node(p, motion, arm, jaws, limits, field_path);
    LOG_INFO("[task] node started");

    ROS_ASYNC_SPIN(2)

    ros::Rate loop(p.rate_hz);
    while (IS_ROS_NODE_OK()) {
        node.tick();
        loop.sleep();
    }

    ROS_SHUTDOWN();
    return 0;
}
