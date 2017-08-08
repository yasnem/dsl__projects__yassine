#ifndef ASRL_RAND1_HPP
#define ASRL_RAND1_HPP

#include "assert_macros.hpp"
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/normal_distribution.hpp>
#include <vector>

namespace asrl {
  typedef boost::minstd_rand base_generator_type;
  // Returns a float r in the range 0.0 < f < 1.0.
   inline double rand1() {
    static base_generator_type generator;

    // Define a uniform random number distribution which produces "double"
    // values between 0 and 1 (0 inclusive, 1 exclusive).
    static boost::uniform_real<> uni_dist(0,1);
    static boost::variate_generator<base_generator_type&, boost::uniform_real<> > uni(generator, uni_dist);

    return uni();
  }

   inline double randLU(float lowerBound, float upperBound) {
    return rand1() * (upperBound - lowerBound) + lowerBound;
  }	

   inline int randLUi(int lowerBound, int upperBound) {
    return (int)floor(rand1() * (upperBound - lowerBound + 1)) + lowerBound;
  }

  template<typename T>
  inline void randomSample(std::vector<T> & vec, unsigned N) {
    ASRL_ASSERT_LT_DBG(std::runtime_error,N, vec.size(), "N must be less than the size of the vector. N: " << N << ", vec.size: " << vec.size());
    for(unsigned i = 0; i < N; i++) {
      int v = asrl::randLUi(i,(int)vec.size()-1);
      std::swap(vec[i],vec[v]);
    }
  }

   inline double randn() {
    static base_generator_type generator;

    // Define a uniform random number distribution which produces "double"
    // values between 0 and 1 (0 inclusive, 1 exclusive).
    static boost::normal_distribution<> normal_dist;
    static boost::variate_generator<base_generator_type&, boost::normal_distribution<> > normal(generator, normal_dist);

    return normal();
  }
}


#endif /* ASRL_RAND1_HPP */
