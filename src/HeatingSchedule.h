#ifndef HEATINGSCHEDULE_H
#define HEATINGSCHEDULE_H

struct HeatingSchedule
{
    HeatingSchedule(double setpoint,
                    double proportional,
                    double integral,
                    double derivative,
                    double hold_time) : setpoint(setpoint), proportional(proportional), integral(integral), derivative(derivative), hold_time(hold_time){};

    double setpoint;
    double proportional;
    double integral;
    double derivative;
    double hold_time; // hold time in seconds 
};

#endif