//
// Created by xu on 2019/12/12.
//

#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include <common_include.h>

namespace myslam
{
class Config
{
private:
    static shared_ptr<Config> config_;
    cv::FileStorage file_;
    Config(){}      // 私有构造函数，实现单例模式

public:
    ~Config();      // close the file when deconstructing

    static void setParameterFile(const string& filename);

    template< typename T >
    static T get(const string& key){
        return T(Config::config_->file_[key]);
    }

};

}

#endif //MYSLAM_CONFIG_H
