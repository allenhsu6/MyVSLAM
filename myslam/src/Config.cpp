//
// Created by xu on 2019/12/12.
//

#include "Config.h"

// 我自己的方法可以new，通过自己函数，限制new多个config
void myslam::Config::setParameterFile(const string &filename) {
    if (config_== nullptr){
        config_ = shared_ptr<Config>(new Config);
    }
    // 装入配置文件
    config_->file_ = cv::FileStorage(filename.c_str(), cv::FileStorage::READ);
    if (config_->file_.isOpened() == false){
        cerr << "parameter file "<< filename << " does not exist!" << endl;
        config_->file_.release();
        return;
    }
}

myslam::Config::~Config() {
    if(file_.isOpened()){
        file_.release();
    }
}

// 单例模式的全局指针定义
shared_ptr<myslam::Config> myslam::Config::config_ = nullptr;