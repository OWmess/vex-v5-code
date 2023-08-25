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
    /**
     * @brief 创建pid日志文件
     * @param mode 底盘模式
     * @param constants pid参数
     * @param left_sensor_target 左侧电机目标值
     * @param right_sensor_target 右侧电机目标值
    */
    bool create_log(std::string mode, PID::Constants constants,double left_sensor_target,double right_sensor_target);
    /**
     * @brief 创建pid日志文件
     * @param mode 底盘模式
     * @param constants pid参数
     * @param target 陀螺仪目标值
    */
    bool create_log(std::string mode, PID::Constants constants,double target);
    /**
     * @brief 设置文件根目录
     * @param root_path 文件根目录
    */
    void set_root_path(std::string root_path);
    /**
     * @brief 保存数据到文件
     * @param gyro_vec 陀螺仪数据
    */
    bool save_data_to_file(const std::vector<double> &gyro_vec);
    /**
     * @brief 保存数据到文件
     * @param left_sensor_vec 左侧电机数据
     * @param right_sensor_vec 右侧电机数据
     * @param gyro_vec 陀螺仪数据
    */
    bool save_data_to_file(const std::vector<double> &left_sensor_vec,const std::vector<double> &right_sensor_vec,const std::vector<double> &gyro_vec);
    
private:
    /**
     * @brief 创建pid日志文件
     * @param mode 底盘模式
     * @param constants pid参数
    */
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