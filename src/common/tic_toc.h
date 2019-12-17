// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk
//           Keke Liu               kekliu@qq.com

#pragma once

#include <glog/logging.h>
#include <chrono>
#include <cstdlib>
#include <ctime>

class TicToc {
  using TimePoint = std::chrono::time_point<std::chrono::system_clock>;

 public:
  TicToc() { tic(); }

  void tic() { start_ = std::chrono::system_clock::now(); }

  double toc() {
    end_ = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed_seconds = end_ - start_;
    return elapsed_seconds.count() * 1000;
  }

 private:
  TimePoint start_;
  TimePoint end_;
};

#define LOG_STEP_TIME(module, describe, msecs) \
  LOG(INFO) << "[" << module << "] " describe << ": " << msecs << " ms"
