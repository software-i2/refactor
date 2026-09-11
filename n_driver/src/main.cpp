// Copyright by BeeX [2026]

#include <bx_msgs/RosBindings.hpp>
#include <n_conf/Doc.h>
#include <n_driver/Node.h>
#include <n_driver/Serial.h>
#include <n_driver/Sim.h>

#include <cstdio>
#include <memory>
#include <string>

DECLARE_ROS_NODE_HANDLE

int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, 0);

    INIT_ROS_NODE("n_driver", 0, "n_driver/alive")

    std::string config;
    GET_ROS_PARAM("~config", config, config);
    if (config.empty()) {
        LOG_ERROR("[arm] no ~config given. Point it at n_conf/config/arm.yaml.");
        return -1;
    }

    conf::Doc doc;
    doc.load(config, {"driver"}, {"jaws"});

    reach::Params p;
    p.load(doc);

    if (!doc.ok()) {
        LOG_ERROR("[arm] %s is not usable:\n%s", config.c_str(), doc.report().c_str());
        return -1;
    }
    if (p.missing() != NULL) {
        LOG_ERROR("[arm] %s: %s is missing or not a usable value", config.c_str(), p.missing());
        return -1;
    }

    LOG_INFO("[arm] config loaded: %s", config.c_str());

    std::string port_override;
    GET_ROS_PARAM("~port", port_override, port_override);
    if (!port_override.empty() && port_override != p.port) {
        LOG_WARN("[arm] ~port overrides driver.port: %s -> %s", p.port.c_str(),
                 port_override.c_str());
        p.port = port_override;
    }

    std::shared_ptr<reach::Port> port;
    if (p.port == "sim") {
        port = std::make_shared<reach::Sim>(p.sim_joint_speed, p.sim_jaw_speed, p.limits);
    } else {
        LOG_INFO("[arm] opening %s at %d baud", p.port.c_str(), p.baud);
        auto serial = std::make_shared<reach::Serial>(p.port, static_cast<unsigned int>(p.baud));
        if (!serial->ok()) {
            LOG_ERROR("[arm] cannot open %s", p.port.c_str());
            return -1;
        }
        port = serial;
    }

    auto arm = std::make_shared<reach::Arm>(port, p.reply_timeout_s, p.climate_timeout_s,
                                            p.climate_max_miss);
    if (!arm->ping()) {
        LOG_ERROR("[arm] no arm on %s", p.port.c_str());
        return -1;
    }
    for (uint32_t j = 0; j < reach::N_JOINTS; ++j) {
        arm->standby(j);
    }

    reach::Node node(arm, p);
    LOG_INFO("[arm] driver initialized at %.1f Hz", p.rate_hz);

    ROS_ASYNC_SPIN(2)

    ros::Rate loop(p.rate_hz);
    while (IS_ROS_NODE_OK()) {
        node.tick();
        loop.sleep();
    }

    for (uint32_t j = 0; j < reach::N_JOINTS; ++j) {
        arm->standby(j);
    }

    ROS_SHUTDOWN();
    return 0;
}
