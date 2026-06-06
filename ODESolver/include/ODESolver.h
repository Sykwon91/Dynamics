#ifndef ODESOLVER_H
#define ODESOLVER_H
#include <iostream>

#include <string>
#include "kinematics.h"

class ODESolver {
    public:
        ODESolver(std::string method = "Euler", double dt = 0.001);
        ~ODESolver();

        void solve(velocity& velocity, position& position);
        void solve(acceleration& acceleration, velocity& velocity);
        void solve(motion& motion);
        void solve(acceleration& acceleration);

    private:
        std::string ODEmethod;
        double dt;
        double timeStamp;
};

#endif // ODESOLVER_H