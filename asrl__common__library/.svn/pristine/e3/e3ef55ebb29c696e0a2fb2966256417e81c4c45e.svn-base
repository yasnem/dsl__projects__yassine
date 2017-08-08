#include <asrl/files/FileSequence.hpp>
#include <boost/filesystem/convenience.hpp>
#include <asrl/string_routines.hpp>
#include <asrl/assert_macros.hpp>
#include <boost/lexical_cast.hpp>

namespace asrl { namespace files {

FileSequence::path_list_t FileSequence::getFileSequence(path_t basePath, std::string const & tag, std::string const & extension_in, unsigned start, unsigned end, int step, unsigned pad, MissingPolicy policy) {
	path_list_t sequence;
	
	ASRL_ASSERT(Exception, start > end ? step < 0 : step > 0, "Step needs to bring the index from start to end. Start: " 
		<< start << ", step: " << step << ", end: " << end);

	std::string extension = extension_in;
	if(extension.size() > 0 && extension[0] == '.')
	  extension = extension.substr(1);


	for(unsigned i = start; (start > end && i >= end) || (start < end && i <= end); i+=step) {
		path_t path = getFileName(basePath, tag, extension, i, pad);
		bool exists = true;
		if(policy != DoNothing)
			exists = boost::filesystem::exists(path);

		if(exists) {
			sequence.push_back(path);
		} else {
			switch(policy) {
				case DoNothing:
					sequence.push_back(path);
					break;
				case WarnInclude:
					std::cout << "Warning: file \"" << path << "\" does not exist. Including anyways.\n";
					sequence.push_back(path);
					break;
				case WarnExclude:
					std::cout << "Warning: file \"" << path << "\" does not exist. Excluding.\n";
					break;
				case ThrowError:
				  ASRL_THROW(Exception,"file \"" << path << "\" does not exist");
					break;
				case Skip:
					// Do nothing
					break;
			} 
		} // end if exists, else
		
	}
	
	return sequence;
}	

FileSequence::path_t FileSequence::getFileName(path_t basePath, std::string const & tag, std::string const & extension, unsigned idx, unsigned pad){
	return basePath / (tag + asrl::leading_pad(boost::lexical_cast<std::string>(idx),pad,'0') + "." + extension);
}}

}// end namespace utr
