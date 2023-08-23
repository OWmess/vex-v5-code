#include "main.h"

template <typename T,typename Func>
inline bool checkAndOutput(std::ofstream& stream, T x, Func&& f) {
    if (!std::isnan(x)&&!std::isinf(x)) {
        f(stream,x);
        return true;
    }
    return false;   
}



PIDLogger::PIDLogger(){
    //初始化根目录，判断sd卡是否存在
    root_path="/usd/";
    sd_card_state=pros::usd::is_installed();
}
PIDLogger::~PIDLogger(){

}

bool PIDLogger::create_log(std::string mode, PID::Constants constants){
    if(!sd_card_state){
        std::cout<<"sd card not found"<<std::endl;
        return false;
    }


    file_path=root_path+mode+"_"+std::to_string(constants.kp)+"_"+
        std::to_string(constants.ki)+"_"+std::to_string(constants.kd)+"_"
        +std::to_string(constants.start_i)+".txt";


    std::ofstream file(file_path, std::ios::out | std::ios::trunc);
    if (file.is_open()) {
        file.close();
        return true;
    } else {
        return false;
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

    file<<"gyro_target:"<<std::endl;
    checkAndOutput(file,target,[](std::ofstream& stream, auto x) {stream << x << std::endl;});

    file<<"gyro:"<<std::endl;
    for(const auto &i:gyro_vec){
        checkAndOutput(file,i,[](std::ofstream& stream, auto x) {stream << x << ",";});
    }
    file<<std::endl;
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
    file<<"left_sensor_target:"<<std::endl;
    checkAndOutput(file,left_sensor_target,[](std::ofstream& stream, auto x) {stream << x << std::endl;});
    file<<"right_sensor_target:"<<std::endl;
    checkAndOutput(file,right_sensor_target,[](std::ofstream& stream, auto x) {stream << x << std::endl;});
    file<<"gyro_target:"<<std::endl;
    checkAndOutput(file,gyro_vec.front(),[](std::ofstream& stream, auto x) {stream << x << std::endl;});

    file<<"gyro:"<<std::endl;
    for(const auto &i:gyro_vec){
        checkAndOutput(file,i,[](std::ofstream& stream, auto x) {stream << x << ",";});
    }
    file<<std::endl;
    file<<"left_sensor:"<<std::endl;
    for(const auto &i:left_sensor_vec){
        checkAndOutput(file,i,[](std::ofstream& stream, auto x) {stream << x << ",";});
    }
    file<<std::endl;
    file<<"right_sensor:"<<std::endl;
    for(const auto &i:right_sensor_vec){
        checkAndOutput(file,i,[](std::ofstream& stream, auto x) {stream << x << ",";});
    }
    file<<std::endl;
    file.close();
    return true;
}