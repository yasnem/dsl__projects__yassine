#ifndef ASRL_FENCEPOST_HPP
#define ASRL_FENCEPOST_HPP

#include "../assert_macros.hpp"
#include "../string_routines.hpp"

namespace asrl { namespace debug {

    template<int SIZE, int SEED = 1>
    class FencePost
    {
    public:
      ASRL_DEFINE_EXCEPTION(Exception,std::runtime_error);
      FencePost()
      {
	srand(SEED);
	for(int i = 0; i < SIZE; i++)
	  {
	    post_[i] = rand();
	  }
      }
      ~FencePost()
      {
	
      }

      void validate(const asrl::source_file_pos & sfp)
      {
	srand(SEED);
	int desiredPost[SIZE];
	for(int i = 0; i < SIZE; i++)
	  {
	    desiredPost[i] = rand();
	  }
	for(int i = 0; i < SIZE; i++)
	  {

	    if(post_[i] != desiredPost[i])
	      {
		asrl::detail::throw_exception<Exception>("", sfp,std::string() +
							 "FencePost exception. Expected Post " + asrl::arrToString(desiredPost,SIZE) +
							 ", got Post " + asrl::arrToString(post_,SIZE));
		
	      }
           }
      } 

      int post_[SIZE];
    };

  }}


#endif /* ASRL_FENCEPOST_HPP */
