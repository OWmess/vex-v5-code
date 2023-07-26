#pragma once
#include <utility>

class Odom{
public:
    template<typename T1>
    class Odom_Point{
    public:
        T1 x;
        T1 y;
        Odom_Point(T1 x, T1 y):x(x),y(y){}
        Odom_Point():x(0),y(0){}
        ~Odom_Point(){}
    };
    typedef typename Odom::Odom_Point<double> Point;
    Odom(){
        this->initlize();
    }
    ~Odom(){
    }

    Point get_position();
    float get_X_position();
    float get_Y_position();
    void set_position(float X_position, float Y_position, float orientation_deg, float ForwardTracker_position, float SidewaysTracker_position);
    void update_position(Point point);
    void initlize(float start_x=0.f,float start_y=0.f,float moveThreshold=0.f,float turnThreshold=0.f);
    void set_threshold(float moveThreshold,float turnThreshold);
    
    /**
     * \param point: the point to compute distance and angle to
     * \return std::pair<float,float>: first is distance, second is angle
    */
    inline std::pair<float,float> computeDistanceAndAngleToPoint(Point point){
        ///TODO:: 这里的计算可能有问题，angle可能需要-90
        float distance = hypot(point.x-this->X_position,point.y-this->Y_position);

        float angle =acos((point.x-this->X_position)/distance)*180/M_PI;
        return std::make_pair(distance,angle);
    }

private:
    float X_position;
    float Y_position;
    float orientation_deg;
    bool isbackwards;
    float ForwardTracker_center_distance;
    float SidewaysTracker_center_distance;
    float ForwardTracker_position;
    float SideWaysTracker_position;
    float moveThreshold;
    float turnThreshold;
};