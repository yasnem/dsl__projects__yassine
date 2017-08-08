#include <asrl/timing/SimpleTimer.hpp>
#include <cstddef>

namespace asrl { namespace common { namespace library {

  SimpleTimer::SimpleTimer()
  {
    reset();
  }

  SimpleTimer::~SimpleTimer()
  {
    // nothing
  }

  void SimpleTimer::reset()
  {
    start_time_ = get_wall_time();
  }

  double SimpleTimer::elapsed()
  {
    return get_wall_time() - start_time_;
  }

  double SimpleTimer::get_wall_time()
  {
    struct timeval time;
    if (gettimeofday(&time,NULL)){
      //  Handle error
      return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
  }

} } }; // asrl::common::library