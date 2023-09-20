#pragma once
#include "EZ-Template/PID.hpp"

struct Point2d{
    double x;
    double y;
};
class Gps_PID:public PID{
    public:
    Gps_PID();
    Gps_PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");
    ~Gps_PID();

    double compute(double x,double y){
      error = hypot(target_position.x-x,target_position.y-y);
      error *= cal_overflow(x, y);
      derivative = error - prev_error;

      if (constants.ki != 0) {
        if (fabs(error) < constants.start_i)
          integral += error;

        if (ez::util::sgn(error) != ez::util::sgn(prev_error))
          integral = 0;
      }

      output = (error * constants.kp) + (integral * constants.ki) + (derivative * constants.kd);

      prev_error = error;

      return output;
    }

    double compute(double current)=delete;
    void set_target(double x,double y){
        target_position={x,y};


    }

    void initlize(double start_x,double start_y,double target_x,double target_y){
      inital_position={start_x,start_y};
      target_position={target_x,target_y};

      k=(target_y-start_y)/(target_x-start_x);
      k1=-1/k;

    }
    void set_target(double current)=delete;

   private:

    inline int8_t cal_overflow(double x,double y) {
      double y1=k1*(x-target_position.x)+target_position.y;
      if(k>0){//可以删掉？
        if(target_position.y-inital_position.y>0){
          if(y1>y){
            return 1;
          }else{
            return -1;
          }
        }else{
          if(y1>y){
            return -1;
          }else{
            return 1;
          }
        }
      }else{
        if(target_position.y-inital_position.y>0){
          if(y1>y){
            return 1;
          }else{
            return -1;
          }
        }
      }
      return 0;
    }

    double k;
    double k1;

    Point2d target_position;
    Point2d inital_position;
};