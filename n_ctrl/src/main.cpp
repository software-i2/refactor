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

    LOG_INFO("[ctrl] config %s\n%s", config.c_str(), doc.effective().c_str());

    ctrl::Node node(p, arm, jaws, limits, field_path);
    LOG_INFO("[ctrl] ready");

    ROS_ASYNC_SPIN(2)

    ros::Rate loop(p.rate_hz);
    while (IS_ROS_NODE_OK()) {
        node.tick();
        loop.sleep();
    }

    ROS_SHUTDOWN();
    return 0;
}
