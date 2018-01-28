/**
* This file is part of FVO.
*  common_headers.h:  the common headers for other c++ files
 *
 * Author: Arhtur.Chen
 * Email: shihezichen@live.cn
 * Created: 27th Jan, 2018
*/

#ifndef FISTVO_COMMON_HEADERS_H
#define FISTVO_COMMON_HEADERS_H

// stl
#include <map>
#include <list>
#include <mutex>
#include <chrono>
#include <vector>
#include <memory>
#include <thread>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>

// for cv:: namespace
#include <opencv2/opencv.hpp>

// for LOG() function
#include <glog/logging.h>

// for file and directory judgement
#include "fvo/FileState.h"

// for Eigen data type definition
#include "fvo/NumTypes.h"
#include <Eigen/Core>
#include <Eigen/Dense>  // linear algebra
#include <Eigen/StdVector>

// for Glboal Configure Parameters
#include "fvo/GlobalConfig.h"

#endif //FISTVO_COMMON_HEADERS_H
