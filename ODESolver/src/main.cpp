#include "ODESolver.h"
int main() {
    // Example usage of ODESolver
    ODESolver solver("Euler", 0.01);
    
    // Create instances of Velocity, Position, and Acceleration
    velocity velocity_{{"1", "2", Vec3{1, 0, 0}, Vec3{0, 0, 0}}, Vec3{1, 0, 0}, Vec3{0, 0, 0}};
    position position_{"1", "2", Vec3{1, 0, 0}, Vec3{0, 0, 0}};
    acceleration acceleration_{{"1", "2", Vec3{0, 0, 0}, Vec3{0, 0, 0}}, Vec3{1, 0, 0}, Vec3{0, 0, 0}}; // Example acceleration values
    
    // Initialize velocity, position, and acceleration as needed
    // ...

    // Solve for new position based on velocity
    solver.solve(acceleration_, velocity_);
    solver.solve(velocity_, position_);
    std::cout << velocity_.translation.x << std::endl;;

    // Solve for new velocity based on acceleration
    

    return 0;
}