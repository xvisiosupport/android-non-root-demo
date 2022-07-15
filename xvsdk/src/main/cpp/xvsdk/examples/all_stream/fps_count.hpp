#pragma once

#include <chrono>
#include <vector>

struct FpsCount
{
  std::vector<std::chrono::system_clock::time_point> frames_ts;
  long long total = 0;

  void tic()
  {
    frames_ts.push_back( std::chrono::system_clock::now() );
    ++total;
    while( std::chrono::duration_cast<std::chrono::milliseconds>( frames_ts.back() - frames_ts.front() ).count() > 1100 ){
      frames_ts.erase( frames_ts.begin() );
    }
  }

  double fps()
  {
    double fps = 0;
    const size_t &size = frames_ts.size();
    if( size > 2 ){
      auto microseconds = std::chrono::duration_cast<std::chrono::microseconds>( frames_ts.back() - frames_ts.front() ).count();
      fps = 1000000.0 * static_cast<double>(size - 1) / microseconds;
    }
    return fps;
  }

  void reset()
  {
      frames_ts.clear();
      total = 0;
  }
};

