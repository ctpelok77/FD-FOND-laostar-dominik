#ifndef TIMER_H
#define TIMER_H

#include <iosfwd>

class Timer {
    double last_start_clock;
    double collected_time;
    bool stopped;

public:
    double current_clock() const;
    Timer();
    ~Timer();
    double operator()() const;
    double stop();
    void resume();
    double reset();
};

std::ostream &operator<<(std::ostream &os, const Timer &timer);

#endif
