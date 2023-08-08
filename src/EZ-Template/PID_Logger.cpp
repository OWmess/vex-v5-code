#include "main.h"
#include <filesystem>
using namespace std;
namespace fs=std::filesystem;
PIDLogger::PIDLogger(){
    root_path="/usd/pid_log/";
    sd_card_state=pros::usd::is_installed();
}
PIDLogger::~PIDLogger(){

}

bool PIDLogger::create_log(std::string mode, PID::Constants constants){
    if(!sd_card_state){
        return false;
    }
    file_path=root_path+mode+"_"+std::to_string(constants.kp)+"_"+
        std::to_string(constants.ki)+"_"+std::to_string(constants.kd)+"_"
        +std::to_string(constants.start_i);

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