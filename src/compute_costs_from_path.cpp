#include <ompl/mod/objectives/DTCOptimizationObjective.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <string>

namespace po = boost::program_options;

std::vector<std::string> getFilesMatchingPattern(
    const std::string& target_path, const std::string& pattern = "*.path") {
  // const boost::regex my_filter(pattern);

  std::vector<std::string> all_matching_files;

  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator i(target_path); i != end_itr;
       ++i) {
    if (!boost::filesystem::is_regular_file(i->status())) continue;

    boost::smatch what;
    if (i->path().filename().extension() != pattern) continue;

    all_matching_files.push_back(i->path().string());
  }
  return all_matching_files;
}

int main(int argn, char* args[]) {
  po::options_description desc("allowed options");
  desc.add_options()("help", "produce help message")(
      "folder", po::value<std::string>(), "Folder where the paths are stored.");

  po::variables_map vm;
  po::store(po::parse_command_line(argn, args, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << "\n";
    return 1;
  }

  if (vm.count("folder")) {
    std::cout << "Folder for paths was set to: "
              << vm["folder"].as<std::string>() << ".\n";
  } else {
    std::cout << "Folder was not set.\n";
    std::cout << desc << "\n";
    return 1;
  }

  std::string folder_path = vm["folder"].as<std::string>();
  char* scenarios[] = {"corridor1", "corridor2", "corridor3", "corridor4"};
  char* times[] = {"t1", "t2", "t3"};

  std::vector<std::string> all_path_files;

  return 0;
}