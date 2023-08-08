#pragma once
#include <string>
#include "EZ-Template/PID.hpp"

struct DriveMode{
    static constexpr const char* FORWARD="forward";
    static constexpr const char* BACKWARD="backward";
    static constexpr const char* TURN="turn";
    static constexpr const char* SWING="swing";
    static constexpr const char* TRUN_GYRO_FREE="turn_gyro_free";
};
class PIDLogger{
public:
    PIDLogger();
    ~PIDLogger();
    bool create_log(std::string mode, PID::Constants constants,double left_sensor_target,double right_sensor_target);
    bool create_log(std::string mode, PID::Constants constants,double target);
    void set_root_path(std::string root_path);
private:
    bool create_log(std::string mode, PID::Constants constants);
private:
    //sd卡是否已插入
    bool sd_card_state;
    //文件路径
    std::string file_path;
    //文件根目录
    std::string root_path;
    double left_sensor_target;
    double right_sensor_target;
    double target;
};