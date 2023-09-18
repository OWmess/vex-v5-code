#include "EZ-Template/PID.hpp"
class Gps_PID:public PID{
    public:
    Gps_PID();
    Gps_PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");
    ~Gps_PID();

    double compute(double x,double y);
    double compute(double current)=delete;
};