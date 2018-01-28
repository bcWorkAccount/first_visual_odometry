/**
* This file is part of FVO.
* FileState:  the helper class of file system.
 * judge the file or directory state
 * all is static functions
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FISTVO_FILESTATE_H
#define FISTVO_FILESTATE_H

#include "fvo/common_headers.h"

namespace fvo {

class FileState {
public:
    // judge whether the directory exists or not
    static bool isDirExist(std::string path) ;
    // judge whether the file exists or not
    static bool isFileExist(std::string path);
};

} // end of namespace

#endif //FISTVO_FILESTATE_H

