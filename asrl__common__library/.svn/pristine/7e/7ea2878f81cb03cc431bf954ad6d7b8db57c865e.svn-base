#ifndef ASRL_ENUM_UTILITIES_HPP
#define ASRL_ENUM_UTILITIES_HPP
#include "assert_macros.hpp"
#include "string_routines.hpp"

namespace asrl {
  template<typename E, typename EXCEPTION_T, typename STRING_ITERATOR>
  E stringToEnum(std::string const & enumName, std::string const & enumSearchString, STRING_ITERATOR stringsBegin, STRING_ITERATOR stringsEnd)
  {
    // This function assumes that the enum starts at 0 and goes to stringsEnd - stringsBegin.
    int N = stringsEnd - stringsBegin;
    ASRL_ASSERT_GT(EXCEPTION_T, N, 0, "A positive number of strings to search is required.");
    STRING_ITERATOR s = stringsBegin;
    for ( ; s != stringsEnd; s++)
      {
	if(enumSearchString == *s)
	  {
	    break;
	  }
      }


    if(s == stringsEnd)
      {
	ASRL_THROW(EXCEPTION_T, "Unknown " << enumName << " \"" << enumSearchString 
		   << "\". Valid values are " 
		   << asrl::arrToString(stringsBegin, stringsEnd)); 
      }
    return (E) (s - stringsBegin);
		 
  }


    
} // namespace asrl
#endif // ASRL_ENUM_UTILITIES_HPP
