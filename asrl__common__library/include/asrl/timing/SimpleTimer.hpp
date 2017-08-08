// system timer
#include <sys/time.h>

namespace asrl { namespace common { namespace library {

  class SimpleTimer
  {
  public:
    SimpleTimer();
    ~SimpleTimer();
    void reset();
    double elapsed();

  private:
    double get_wall_time();

    double start_time_;
  };

} } } // asrl::common::library