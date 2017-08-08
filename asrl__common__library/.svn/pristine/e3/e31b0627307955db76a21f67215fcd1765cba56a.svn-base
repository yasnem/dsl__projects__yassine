#ifndef ASRL_FILES_FILESEQUENCE_HPP
#define ASRL_FILES_FILESEQUENCE_HPP

#include <boost/filesystem/path.hpp>
#include<vector>
#include <asrl/assert_macros.hpp>

namespace asrl { namespace files {

    ASRL_DEFINE_EXCEPTION(Exception,std::runtime_error);

    class FileSequence {
    public:
      enum MissingPolicy {
	DoNothing,
	WarnInclude,
	WarnExclude,
	ThrowError,
	Skip
      };
      typedef boost::filesystem::path path_t;
      typedef std::vector<path_t> path_list_t;
      static path_list_t getFileSequence(path_t basePath, std::string const & tag, std::string const & extension, unsigned start, unsigned end, int step, unsigned pad, MissingPolicy policy = DoNothing);
      static path_t getFileName(path_t basePath, std::string const & tag, std::string const & extension, unsigned idx, unsigned pad);
    };

  }} // end namespace asrl::files


#endif // ASRL_FILES_FILESEQUENCE_HPP
