#pragma once

#include <chrono>
#include <vector>

class FrequencyCounter
{
  std::vector<std::chrono::steady_clock::time_point> m_framesTps;
  unsigned long m_count = 0;
  
public:
  void tic()
  {
    ++m_count;
    m_framesTps.push_back( std::chrono::steady_clock::now() );
    while( m_framesTps.size() > 100 ){
      m_framesTps.erase( m_framesTps.begin() );
    }
  }
  unsigned long count() {return m_count; }
  double fps()
  {
    double fps = 0;
    const size_t &size = m_framesTps.size();
    if( size > 2 ){
      auto milliseconds = std::chrono::duration_cast<std::chrono::microseconds>( m_framesTps.back() - m_framesTps.front() ).count();
      fps = 1000000.0 * static_cast<double>(size) / milliseconds;
    }
    return fps;
  }
};

