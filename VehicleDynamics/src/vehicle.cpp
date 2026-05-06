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
                Spec.WheelMount[i] = parsePosition(nodes[i]);
        }

        if (config["SuspensionPosition"] && config["SuspensionPosition"].IsSequence())
        {
            const YAML::Node& nodes = config["SuspensionPosition"];
            Spec.SuspensionPosition = new position[nodes.size()];
            if (Spec.TotalWheels == 0)
                Spec.TotalWheels = static_cast<int>(nodes.size());
            for (std::size_t i = 0; i < nodes.size(); ++i)
                Spec.SuspensionPosition[i] = parsePosition(nodes[i]);
        }
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
    delete[] Spec.WheelMount;
    delete[] Spec.SuspensionPosition;
    delete Spec.Wheel;
    delete Spec.Chassis;
}

void Vehicle::Update()
{
    
    this->State.LocalVehicleMotion = this->State.GlobalVehicleMotion.InverseKinematics(this->State.GlobalVehicleMotion);
    
    
    for(int wheelcnt = 0 ; wheelcnt < this->Spec.TotalWheels ; wheelcnt++)
    {
        this->State.SuspensionMotion[wheelcnt] = this->State.GlobalVehicleMotion.InverseKinematics(this->State.GlobalSuspensionMotion[wheelcnt]);
        
        /// applied force from suspension to wheel mount
        odeSolver.solve( this->State.WheelMountMotion[wheelcnt]);
        odeSolver.solve( this->State.WheelMotion[wheelcnt]);
        this->State.GlobalSuspensionMotion[wheelcnt] = this->State.LocalVehicleMotion.ForwardKinematics(this->State.SuspensionMotion[wheelcnt]);
    }

    this->State.GlobalVehicleMotion = this->State.GlobalVehicleMotion.ForwardKinematics(this->State.LocalVehicleMotion);
    // Example: update wheel mount position based on vehicle position
    // Vehicle update logic can be implemented here.
}

bool Vehicle::Visualize(const std::string& filename) const
{
    std::ofstream file(filename);
    if (!file)
        return false;

    Visualization::ObjWriter writer(file);

    const position& vehiclePose = State.LocalVehicleMotion.frame_position;
    if (Spec.Chassis != nullptr)
    {
        Box chassis = *Spec.Chassis;
        position chassisLocal;
        chassisLocal.translation = chassis.Center.translation;
        chassisLocal.orientation = chassis.Center.rotation;

        const position chassisGlobal = vehiclePose.ForwardKinematics(chassisLocal);
        chassis.Center.translation = chassisGlobal.translation;
        chassis.Center.rotation = chassisGlobal.orientation;
        writer.writeBox("chassis", chassis);
    }

    if (Spec.Wheel == nullptr)
        return true;

    for (int wheel = 0; wheel < Spec.TotalWheels; ++wheel)
    {
        Cylinder wheelShape = *Spec.Wheel;
        position wheelPose;

        if (State.GlobalWheelMotion != nullptr)
        {
            wheelPose = State.GlobalWheelMotion[wheel].frame_position;
        }
        else if (Spec.WheelMount != nullptr && Spec.SuspensionPosition != nullptr)
        {
            wheelPose = vehiclePose.ForwardKinematics(Spec.WheelMount[wheel]).ForwardKinematics(Spec.SuspensionPosition[wheel]);
        }
        else if (Spec.WheelMount != nullptr)
        {
            wheelPose = vehiclePose.ForwardKinematics(Spec.WheelMount[wheel]);
        }
        else
        {
            wheelPose = vehiclePose;
        }

        position wheelCenter;
        wheelCenter.translation = wheelShape.Center.translation;
        wheelCenter.orientation = wheelShape.Center.rotation;
        wheelPose = wheelPose.ForwardKinematics(wheelCenter);

        wheelShape.Center.translation = wheelPose.translation;
        wheelShape.Center.rotation = wheelPose.orientation;
        writer.writeCylinder("wheel_" + std::to_string(wheel), wheelShape);
    }

    return true;
}
