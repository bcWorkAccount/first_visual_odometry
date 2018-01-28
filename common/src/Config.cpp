/**
* This file is part of FVO.
* Config.h:  read the configuration information from yaml setting file
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/


#include "fvo/Config.h"

namespace fvo{

    Config::Config(const string& settingFile )
    {
        //Read rectification parameters
        settings = cv::FileStorage ( settingFile, cv::FileStorage::READ);
        if( !settings.isOpened() ) {
            LOG(FATAL) << "ERROR: Wrong path to settings file :" << settingFile <<  endl;
            return;
        }
    }

    Config::~Config() {}

    void Config::readAllSettings(){
        // Store the rows and cols at global config
        G::imageWidth = settings["Camera.height"];
        G::imageHeight= settings["Camera.width"];

        // Get the camera parameters
        G::fx = settings["Camera.fx"];
        G::fy = settings["Camera.fy"];
        G::cx = settings["Camera.cx"];
        G::cy = settings["Camera.cy"];
        G::bf = settings["Camera.bf"];

        // Get the View parameters
        G::viewerHeight = settings["View.height"];
        G::viewerWidth  = settings["View.width"];
        G::UI_WIDTH = settings["View.UI_WIDTH"];
    }


} // end of namespace