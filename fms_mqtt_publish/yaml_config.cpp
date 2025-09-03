#include "yaml_config.h"
#include <yaml-cpp/yaml.h>
#include <iostream>

AmrConfig YamlConfig::load(const std::string& filename) 
{
    YAML::Node config = YAML::LoadFile(filename);
    AmrConfig cfg;
    cfg.amr_count = config["amr_count"].as<int>();
    cfg.base_port = config["base_port"].as<int>();

    // Load protocol_type
    if (config["protocol_type"]) 
    {
        cfg.protocol_type = config["protocol_type"].as<std::string>();
    } 
    else 
    {
        cfg.protocol_type = "vda5050";
        std::cerr << "Warning: 'protocol_type' not found in config. Defaulting to 'vda5050'." << std::endl;
    }    

    cfg.amr_params.wheel_radius = config["amr_params"]["wheel_radius"].as<double>();
    cfg.amr_params.wheel_base = config["amr_params"]["wheel_base"].as<double>();
    cfg.amr_params.max_speed = config["amr_params"]["max_speed"].as<double>();
    cfg.amr_params.max_angular_speed = config["amr_params"]["max_angular_speed"].as<double>();
    cfg.amr_params.max_acceleration = config["amr_params"]["max_acceleration"].as<double>();
    cfg.amr_params.max_deceleration = config["amr_params"]["max_deceleration"].as<double>();
    cfg.amr_params.max_angular_acceleration = config["amr_params"]["max_angular_acceleration"].as<double>();
    cfg.amr_params.max_angular_deceleration = config["amr_params"]["max_angular_deceleration"].as<double>();
    cfg.amr_params.mass_vehicle = config["amr_params"]["mass_vehicle"].as<double>();
    cfg.amr_params.load_weight = config["amr_params"]["load_weight"].as<double>();
    cfg.amr_params.max_torque = config["amr_params"]["max_torque"].as<double>();
    cfg.amr_params.friction_coeff = config["amr_params"]["friction_coeff"].as<double>();
    cfg.amr_params.max_rpm_deviation = config["amr_params"]["max_rpm_deviation"].as<double>();

    if(config["speedup_ratio"])
    {
        cfg.speedup_ratio = config["speedup_ratio"].as<double>();
    }
    else
    {
        cfg.speedup_ratio  = 1.0;
    }

    cfg.visualization_publish_period = config["visualization_publish_period"].as<double>();
    cfg.state_publish_period = config["state_publish_period"].as<double>();
    cfg.control_period = config["control_period"].as<double>();
    cfg.dead_reckoning_model = config["dead_reckoning_model"].as<std::string>();

    return cfg;
}