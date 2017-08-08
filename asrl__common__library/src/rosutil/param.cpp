#include <asrl/rosutil/param.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <boost/algorithm/string/trim.hpp>
#include <asrl/string_routines.hpp>

namespace asrl { namespace rosutil {

    ParameterContainer::ParameterContainer()
    {

    }

    ParameterContainer::~ParameterContainer()
    {

    }


    RosParameterContainer::RosParameterContainer(const ros::NodeHandle & nh) : nh_(nh)
    {
    }
    
    RosParameterContainer::~RosParameterContainer()
    {
    }

      
    int RosParameterContainer::getInt(const std::string & key, int defaultValue)
    {
      return get<int>(key,defaultValue);
    }
 
    int RosParameterContainer::getInt(const std::string & key)
    {
      return get<int>(key);
    }
 

    double RosParameterContainer::getDouble(const std::string & key, double defaultValue)
    {
      return get<double>(key,defaultValue);
    }
 
    double RosParameterContainer::getDouble(const std::string & key)
    {
      return get<double>(key);
    }
 

    std::string RosParameterContainer::getString(const std::string & key, const std::string & defaultValue)
    {
      return get<std::string>(key,defaultValue);
    }
 
    std::string RosParameterContainer::getString(const std::string & key)
    {
      return get<std::string>(key);
    }
 

    bool RosParameterContainer::getBool(const std::string & key, bool defaultValue)
    {
      return get<bool>(key,defaultValue);
    }
 
    bool RosParameterContainer::getBool(const std::string & key)
    {
      return get<bool>(key);
    }


    // ---

    MapParameterContainer::MapParameterContainer()
    {

    }
    MapParameterContainer::~MapParameterContainer()
    {

    }

    int MapParameterContainer::getInt(const std::string & key, int defaultValue)
    {
      return get<int>(key, defaultValue);
    }
 
    int MapParameterContainer::getInt(const std::string & key)
    {
      return get<int>(key);
    }
 

    double MapParameterContainer::getDouble(const std::string & key, double defaultValue)
    {
      return get<double>(key, defaultValue);
    }
 
    double MapParameterContainer::getDouble(const std::string & key)
    {
      return get<double>(key);
    }
 

    std::string MapParameterContainer::getString(const std::string & key, const std::string & defaultValue)
    {
      return get<std::string>(key, defaultValue);
    }
 
    std::string MapParameterContainer::getString(const std::string & key)
    {
      return get<std::string>(key);
    }
 

    bool MapParameterContainer::getBool(const std::string & key, bool defaultValue)
    {
      return get<bool>(key, defaultValue);
    }
 
    bool MapParameterContainer::getBool(const std::string & key)
    {
      return get<bool>(key);
    }
 

    void MapParameterContainer::clear()
    {
      map_.clear();
    }
    
    void MapParameterContainer::load(const std::string & configFile)
    {
      using namespace boost::filesystem;
      boost::filesystem::path thePath(configFile);
      if(!thePath.has_root_path())
        {
          thePath = current_path() / thePath; 
        }

      std::deque<path> fileQueue;
      fileQueue.push_back(thePath);
      std::set<path> filesUsed;
      filesUsed.insert(thePath);
      path currFile;
		
      const unsigned b_size = 2048;
      char buffer[b_size];

      while(!fileQueue.empty()) {
        currFile = fileQueue.front();
        fileQueue.pop_front();
        std::ifstream fin(currFile.string().c_str());

        ASRL_ASSERT(std::runtime_error, fin.good(),"Unable to open config file: \"" << currFile << "\"");

        while(!fin.eof()) {
          std::string key;
          std::string val;
          fin >> key;
          if(key.size() == 0)
            continue;
				
          if(key[0] == '#') {
            if(key == "#include") {
              //std::cout << "Found an include file.\n";
              fin.getline(buffer, b_size);
	      std::string fname_tsub = boost::algorithm::trim_copy(asrl::substituteEnvVars(buffer));
              path fname = path(fname_tsub);
              if(!fname.has_root_path())
                fname = currFile.parent_path() / fname;

              if(filesUsed.find(fname) == filesUsed.end()) {
                // queue up the file.
                fileQueue.push_back(fname);
                // Track the file usage so the same file isn't parsed twice.
                filesUsed.insert(fname);
              }
              continue;
            } else {
              fin.getline(buffer, b_size);
              continue;
            }
          }
				
          ASRL_ASSERT(std::runtime_error, !fin.eof(),"Unbalanced key/value in config file \"" << currFile << "\", key: " << key << ", key size: " << key.size());
          fin.getline(buffer, b_size);
          val = asrl::substituteEnvVars(boost::algorithm::trim_copy(std::string(buffer)));
          map_[key] = val;
        }
      }
    }

    void MapParameterContainer::setPrefix(const std::string & prefix)
    {
      if(prefix.size() == 0)
        {
          prefix_ = prefix;
        }
      else if( prefix[prefix.size() - 1] != '/')
        {
          prefix_ = prefix + "/";
        }
      else
        {
          prefix_ = prefix;
        }
    }
    

      std::string MapParameterContainer::addPrefix(const std::string & key)
      {
        std::string result;
        if(key.size() > 0 && key[0] == '/')
          {
            result = prefix_ + key.substr(1);
          }
        else
          {
            result = prefix_ + key;
          }
        return result;
      }



  }}
