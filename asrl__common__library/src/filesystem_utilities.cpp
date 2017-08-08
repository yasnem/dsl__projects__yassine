#include <asrl/filesystem_utilities.hpp>

namespace asrl {
  boost::filesystem::path getAsrlDirectory()
  {
    std::string home = getenv("HOME");

    if(home.size() == 0)
      {
	home = "/tmp";
      }
    boost::filesystem::path asrlDir = home + "/.asrl";
    if(!boost::filesystem::exists(asrlDir))
      {
	boost::filesystem::create_directory(asrlDir);
      }

    return asrlDir;

  }

} // namespace asrl
