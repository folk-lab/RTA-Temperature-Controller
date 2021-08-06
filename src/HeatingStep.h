#ifndef HEATINGSTEP_H
#define HEATINGSTEP_H

struct HeatingStep
{
    HeatingStep(double setpoint,
                double proportional,
                double integral,
                double derivative,
                double hold_time,
                double delta_t) : setpoint(setpoint), proportional(proportional), integral(integral), derivative(derivative), hold_time(hold_time), delta_t(delta_t){};
                
    double setpoint;
    double proportional;
    double integral;
    double derivative;
    double hold_time; // hold time in seconds
    double delta_t;
};

#endif