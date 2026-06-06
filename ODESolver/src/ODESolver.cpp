#include "ODESolver.h"


ODESolver::ODESolver(std::string method, double dt)
    : ODEmethod(std::move(method)), dt(dt), timeStamp(0.0)
{
}

ODESolver::~ODESolver()
{
}


void ODESolver::solve(velocity& velocity, position& position)
{
    if (ODEmethod == "Euler")
    {
        position.translation = position.translation + velocity.translation * dt;
        position.orientation = position.orientation + velocity.orientation * dt;

    }
    // Implement other methods like Runge-Kutta here
}


void ODESolver::solve(acceleration& acceleration, velocity& velocity)
{
    if (ODEmethod == "Euler")
    {
        velocity.translation = velocity.translation + acceleration.translation * dt;
        velocity.orientation = velocity.orientation + acceleration.orientation * dt;
    }
    // Implement other methods like Runge-Kutta here
}


void ODESolver::solve(acceleration& acceleration)
{
    if (ODEmethod == "Euler")
    {
         
        acceleration.frame_velocity.translation = acceleration.frame_velocity.translation + acceleration.translation * dt;
        acceleration.frame_velocity.orientation = acceleration.frame_velocity.orientation + acceleration.orientation * dt;
        acceleration.frame_velocity.frame_position.translation = acceleration.frame_velocity.frame_position.translation + acceleration.frame_velocity.translation * dt;
        acceleration.frame_velocity.frame_position.orientation = acceleration.frame_velocity.frame_position.orientation + acceleration.frame_velocity.orientation * dt;   
    }
    // Implement other methods like Runge-Kutta here
}

void ODESolver::solve(motion& motion)
{
    if (ODEmethod == "Euler")
    {
        
        solve(motion.frame_acceleration, motion.frame_velocity);
        solve(motion.frame_velocity, motion.frame_position);
        //std::cout << motion.frame_position.translation.x << ", " << motion.frame_position.translation.y << ", " << motion.frame_position.translation.z << std::endl;
    }
    // Implement other methods like Runge-Kutta here
}