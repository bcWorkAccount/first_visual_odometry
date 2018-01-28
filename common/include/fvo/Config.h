/**
* This file is part of FVO.
* Config.h:  read the configuration information from yaml setting file
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/


#ifndef FIRSTVO_CONFIG_H
#define FIRSTVO_CONFIG_H

#include "fvo/common_headers.h"

using namespace std;

namespace fvo{

class Config {
private:
    cv::FileStorage settings;
public:

    Config(const string& settingFile );
    virtual ~Config();

    // read all settings from setting file and store them into
    //   GlboalConfig vars
    void readAllSettings();
};



} // end of namespace

#endif //FIRSTVO_CONFIG_H
