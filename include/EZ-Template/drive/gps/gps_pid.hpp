#pragma once
#include "EZ-Template/PID.hpp"

class Point2d{
    public:
    Point2d(double x,double y){
      update_quadrant(x,y);
    }

    Point2d(){
        x=0;
        y=0;
        quadrant=1;
    }

    void set(double x,double y){
      update_quadrant(x,y);
      this->x=x;
      this->y=y;
    }

    double get_x(){
      return x;
    }

    double get_y(){
      return y;
    }

   private:
    double x;
    double y;
    int8_t quadrant;


    void update_quadrant(double x,double y){
      if(x>=0&&y>=0)
        quadrant=1;
      else if(x<0&&y>0)
        quadrant=2;
      else if(x<0&&y<0)
        quadrant=3;
      else if(x>0&&y<0)
        quadrant=4;
    }
};
class Gps_PID:public PID{
    public:
    Gps_PID();
    Gps_PID(double p, double i = 0, double d = 0, double start_i = 0, std::string name = "");
    ~Gps_PID();

    double compute(double x,double y){
      error = hypot(target2d.get_x()-x,target2d.get_y()-y);
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
        target2d={x,y};


    }
    void set_target(double current)=delete;

    auto get_target(){
        return target;
    }
   private:

    inline int8_t cal_overflow(double x,double y) {
        k=(target2d.get_y()-y)/(target2d.get_x()-x);
        double k1=-1/k;
        double y1=k1*(x-target2d.get_x())+target2d.get_y();

        if(k1>0){

        }
        if(k1*(x-target2d.get_x())+target2d.get_y()-y>0){
          return -1;
        }else{
          return 1;
        }
    }
    double k;
    Point2d target2d;

};