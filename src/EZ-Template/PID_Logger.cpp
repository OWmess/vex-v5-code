#include "main.h"
#include <filesystem>
namespace fs=std::filesystem;
PIDLogger::PIDLogger(){
    //初始化根目录，判断sd卡是否存在
    root_path="/usd/pid_log/";
    sd_card_state=pros::usd::is_installed();
}
PIDLogger::~PIDLogger(){

}

bool PIDLogger::create_log(std::string mode, PID::Constants constants){
    if(!sd_card_state){
        std::cout<<"sd card not found"<<std::endl;
        return false;
    }
    //创建文件，文件已存在则清空文件
    file_path=root_path+mode+"_"+std::to_string(constants.kp)+"_"+
        std::to_string(constants.ki)+"_"+std::to_string(constants.kd)+"_"
        +std::to_string(constants.start_i)+".txt";

    if (fs::exists(file_path)) {
        std::ofstream file(file_path, std::ios::out | std::ios::trunc);
        if (file.is_open()) {
            file.close();
            return true;
        } else {
            return false;
        }
    } else {
        std::ofstream file(file_path);
        if (file.is_open()) {
            file.close();
            return true;
        } else {
            return false;
        }
    }
}

bool PIDLogger::create_log(std::string mode, PID::Constants constants,double target){
    this->target=target;
    return create_log(mode,constants);
}

bool PIDLogger::create_log(std::string mode, PID::Constants constants,double left_sensor_target,double right_sensor_target){
    this->left_sensor_target=left_sensor_target;
    this->right_sensor_target=right_sensor_target;
    return create_log(mode,constants);
}

void PIDLogger::set_root_path(std::string root_path){
    this->root_path=root_path;
}

bool PIDLogger::save_data_to_file(const std::vector<double> &gyro_vec){
    if(!sd_card_state){
        std::cout<<"sd card not found"<<std::endl;
        return false;
    }
    std::ofstream file(file_path,ios::app);
    if(!file.is_open()){
        std::cout<<"failed to open pid data file"<<std::endl;
        return false;
    }
    file<<"gyro:"<<std::endl;
    for(const auto &i:gyro_vec){
        file<<i<<",";
    }
    file<<"##"<<std::endl;
    file.close();
    return true;
}

bool PIDLogger::save_data_to_file(const std::vector<double> &left_sensor_vec,const std::vector<double> &right_sensor_vec,const std::vector<double> &gyro_vec){
    if(!sd_card_state){
        std::cout<<"sd card not found"<<std::endl;
        return false;
    }
    std::ofstream file(file_path,ios::app);
    if(!file.is_open()){
        std::cout<<"failed to open pid data file"<<std::endl;
        return false;
    }

    file<<"gyro:"<<std::endl;
    for(const auto &i:gyro_vec){
        file<<i<<",";
    }
    file<<"##"<<std::endl;

    file<<"left_sensor:"<<std::endl;
    for(const auto &i:left_sensor_vec){
        file<<i<<",";
    }
    file<<"##"<<std::endl;

    file<<"right_sensor:"<<std::endl;
    for(const auto &i:right_sensor_vec){
        file<<i<<",";
    }
    file<<"##"<<std::endl;

    file.close();
    return true;
}