#include "vehicle.h"
#include "visualization.h"
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>

static Vec3 parseVec3(const YAML::Node& node)
{
    Vec3 v{0.0, 0.0, 0.0};
    if (!node) return v;
    v.x = node["x"].as<double>(0.0);
    v.y = node["y"].as<double>(0.0);
    v.z = node["z"].as<double>(0.0);
    return v;
}

static Vec3* parseVec3Array(const YAML::Node& node, int size)
{
    if (!node || size <= 0)
        return nullptr;

    Vec3* values = new Vec3[size];
    if (node.IsSequence())
    {
        const int nodeCount = static_cast<int>(node.size());
        for (int i = 0; i < size; ++i)
        {
            values[i] = i < nodeCount ? parseVec3(node[i]) : Vec3{0.0, 0.0, 0.0};
        }
    }
    else
    {
        const Vec3 value = parseVec3(node);
        for (int i = 0; i < size; ++i)
            values[i] = value;
    }

    return values;
}

static void loadVec3Array(Vec3*& target, const YAML::Node& node, int size)
{
    if (!node)
        return;

    delete[] target;
    target = parseVec3Array(node, size);
}

static position parsePosition(const YAML::Node& node)
{
    position p;
    if (!node) return p;
    p.frame = node["frame"].as<std::string>(p.frame);
    p.childframe = node["childframe"].as<std::string>(p.childframe);
    p.translation = parseVec3(node["translation"]);
    p.orientation = parseVec3(node["orientation"]);
    return p;
}

static std::string configPath(const std::string& filename)
{
    if (filename.size() >= 5 && filename.substr(filename.size() - 5) == ".yaml")
        return filename;
    return filename + ".yaml";
}

static void initializeStateFromSpec(Vehicle& vehicle)
{
    const int totalWheels = vehicle.Spec.TotalWheels;
    if (totalWheels <= 0)
        return;

    delete[] vehicle.State.ContactPoint;
    delete[] vehicle.State.WheelMountMotion;
    delete[] vehicle.State.WheelMotion;
    delete[] vehicle.State.WheelForce;
    delete[] vehicle.State.SuspensionMotion;
    delete[] vehicle.State.SuspensionForce;
    delete[] vehicle.State.GlobalWheelMotion;
    delete[] vehicle.State.GlobalWheelMountMotion;
    delete[] vehicle.State.GlobalSuspensionMotion;

    vehicle.State.ContactPoint = new position[totalWheels];
    vehicle.State.ContactWrench = new Wrench[totalWheels];
    vehicle.State.WheelMountMotion = new acceleration[totalWheels];
    vehicle.State.WheelMotion = new acceleration[totalWheels];
    vehicle.State.WheelForce = new Wrench[totalWheels];
    vehicle.State.SuspensionMotion = new acceleration[totalWheels];
    vehicle.State.SuspensionForce = new Wrench[totalWheels];
    vehicle.State.GlobalContactPoint = new position[totalWheels];
    vehicle.State.GlobalWheelMotion = new acceleration[totalWheels];
    vehicle.State.GlobalWheelMountMotion = new acceleration[totalWheels];
    vehicle.State.GlobalSuspensionMotion = new acceleration[totalWheels];
    vehicle.State.GravityWheelForce = new Wrench[totalWheels];

    for (int wheel = 0; wheel < totalWheels; ++wheel)
    {
        if (vehicle.Spec.WheelMount != nullptr)
        {
            vehicle.State.WheelMountMotion[wheel].frame_velocity.frame_position = vehicle.Spec.WheelMount[wheel];
            vehicle.State.GlobalWheelMountMotion[wheel].frame_velocity.frame_position =
                vehicle.State.LocalVehicleMotion.frame_velocity.frame_position.ForwardKinematics(vehicle.Spec.WheelMount[wheel]);
            vehicle.State.GlobalContactPoint[wheel] = vehicle.State.GlobalWheelMountMotion[wheel].frame_velocity.frame_position;
        }

        if (vehicle.Spec.SuspensionPosition != nullptr)
        {
            vehicle.State.SuspensionMotion[wheel].frame_velocity.frame_position = vehicle.Spec.SuspensionPosition[wheel];
            vehicle.State.WheelMotion[wheel].frame_velocity.frame_position = vehicle.Spec.SuspensionPosition[wheel];

            const position mountPose = vehicle.Spec.WheelMount != nullptr
                ? vehicle.State.GlobalWheelMountMotion[wheel].frame_velocity.frame_position
                : vehicle.State.LocalVehicleMotion.frame_velocity.frame_position;
            const position wheelPose = mountPose.ForwardKinematics(vehicle.Spec.SuspensionPosition[wheel]);
            vehicle.State.GlobalSuspensionMotion[wheel].frame_velocity.frame_position = wheelPose;
            vehicle.State.GlobalWheelMotion[wheel].frame_velocity.frame_position = wheelPose;
        }
        else if (vehicle.Spec.WheelMount != nullptr)
        {
            vehicle.State.GlobalWheelMotion[wheel].frame_velocity.frame_position = vehicle.State.GlobalWheelMountMotion[wheel].frame_velocity.frame_position;
        }
    }
}

Vehicle::Vehicle(std::string filename)
    : odeSolver("Euler", 0.001)
{
    const std::string path = configPath(filename);
    try
    {
        YAML::Node config = YAML::LoadFile(path);
        if (config["Mass"]) Spec.Mass = config["Mass"].as<double>();
        if (config["Inertia"]) Spec.Inertia = parseVec3(config["Inertia"]);
        if (config["CenterOfMass"]) Spec.CenterOfMass = parseVec3(config["CenterOfMass"]);
        if (config["WheelMass"]) Spec.WheelMass = config["WheelMass"].as<double>();
        if (config["WheelInertia"]) Spec.WheelInertia = parseVec3(config["WheelInertia"]);

        if (config["Wheel"])
        {
            auto wheel = config["Wheel"];
            double radius = wheel["radius"].as<double>(0.0);
            double height = wheel["height"].as<double>(0.0);
            Position center{};
            center.translation = parseVec3(wheel["center"]["translation"]);
            center.rotation = parseVec3(wheel["center"]["rotation"]);
            Spec.Wheel = new Cylinder(radius, height, center);
        }

        if (config["Chassis"])
        {
            auto chassis = config["Chassis"];
            double length = chassis["length"].as<double>(0.0);
            double width = chassis["width"].as<double>(0.0);
            double height = chassis["height"].as<double>(0.0);
            Position center{};
            center.translation = parseVec3(chassis["center"]["translation"]);
            center.rotation = parseVec3(chassis["center"]["rotation"]);
            Spec.Chassis = new Box(length, width, height, center);
        }

        if (config["WheelMount"] && config["WheelMount"].IsSequence())
        {
            const YAML::Node& nodes = config["WheelMount"];
            Spec.WheelMount = new position[nodes.size()];
            Spec.TotalWheels = static_cast<int>(nodes.size());
            for (std::size_t i = 0; i < nodes.size(); ++i)
            {
                Spec.WheelMount[i] = parsePosition(nodes[i]);
            }
                
                
        }

        if (config["SuspensionPosition"] && config["SuspensionPosition"].IsSequence())
        {
            const YAML::Node& nodes = config["SuspensionPosition"];

            Spec.SuspensionPosition = new position[nodes.size()];
            
            if (Spec.TotalWheels == 0)
                Spec.TotalWheels = static_cast<int>(nodes.size());
            for (std::size_t i = 0; i < nodes.size(); ++i)
                {
                    Spec.SuspensionPosition[i] = parsePosition(nodes[i]);
                }
        }

        const YAML::Node suspension = config["Suspension"];
        if (suspension)
        {
            loadVec3Array(Spec.SuspensionSpring, suspension["stiffness"], Spec.TotalWheels);
            loadVec3Array(Spec.SuspensionDamper, suspension["damping"], Spec.TotalWheels);
        }
        loadVec3Array(Spec.SuspensionSpring, config["SuspensionSpring"], Spec.TotalWheels);
        loadVec3Array(Spec.SuspensionDamper, config["SuspensionDamper"], Spec.TotalWheels);

        if (config["Wheel"])
        {
            auto wheel = config["Wheel"];
            loadVec3Array(Spec.WheelSpring, wheel["stiffness"], Spec.TotalWheels);
            loadVec3Array(Spec.WheelDamper, wheel["damping"], Spec.TotalWheels);
        }
        loadVec3Array(Spec.WheelSpring, config["WheelSpring"], Spec.TotalWheels);
        loadVec3Array(Spec.WheelDamper, config["WheelDamper"], Spec.TotalWheels);

        initializeStateFromSpec(*this);
    }
    catch (const YAML::BadFile& e)
    {
        std::cerr << "Vehicle: failed to open YAML file '" << path << "': " << e.what() << "\n";
    }
    catch (const YAML::Exception& e)
    {
        std::cerr << "Vehicle: YAML parse error in '" << path << "': " << e.what() << "\n";
    }
}

Vehicle::~Vehicle()
{
    delete[] State.ContactPoint;
    delete[] State.WheelMountMotion;
    delete[] State.WheelMotion;
    delete[] State.WheelForce;
    delete[] State.SuspensionMotion;
    delete[] State.SuspensionForce;
    delete[] State.GlobalWheelMotion;
    delete[] State.GlobalWheelMountMotion;
    delete[] State.GlobalSuspensionMotion;
    delete[] Spec.WheelMount;
    delete[] Spec.SuspensionPosition;
    delete[] Spec.SuspensionSpring;
    delete[] Spec.SuspensionDamper;
    delete[] Spec.WheelSpring;
    delete[] Spec.WheelDamper;
    delete Spec.Wheel;
    delete Spec.Chassis;
}


