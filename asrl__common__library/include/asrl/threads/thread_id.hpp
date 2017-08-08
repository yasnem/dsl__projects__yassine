#ifndef ASRL_THREAD_ID_HPP
#define ASRL_THREAD_ID_HPP

namespace asrl {
  namespace threads {

    // Warning: this code should never be use.d
    inline long int thread_id()
    {
      return (long int)syscall(224);
    }

  }} 


#endif /* ASRL_THREAD_ID_HPP */
