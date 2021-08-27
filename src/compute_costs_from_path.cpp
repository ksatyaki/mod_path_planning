#include <ompl/mod/objectives/DTCOptimizationObjective.h>
#include <ompl/mod/objectives/IntensityMapOptimizationObjective.h>
#include <ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h>
#include <ros/console.h>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/regex.hpp>
#include <cliffmap_ros/cliffmap.hpp>
#include <gmmtmap_ros/gmmtmap.hpp>
#include <ompl_planners_ros/car_state_space.hpp>
#include <stefmap_ros/stefmap.hpp>
#include <string>

namespace po = boost::program_options;
namespace ob = ompl::base;

std::vector<std::string> getFilesMatchingPattern(
    const std::string &target_path, const std::string &pattern = "*.path") {
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

void computeCostsFromCSV(const std::string &csv_file_name,
                         const ompl::mod::MoDOptimizationObjectivePtr &ptr,
                         const std::string &type) {
  std::string costs_file_name = csv_file_name;
  boost::regex_replace(costs_file_name, boost::regex(".path"), ".costs_file");

  std::fstream path_file(csv_file_name, std::ios::in);
  std::fstream costs_file(costs_file_name, std::ios::out);

  if (!path_file) {
    ROS_INFO_STREAM("Somehow couldn't open this file for reading "
                    << csv_file_name);
  }
  if (!costs_file) {
    ROS_INFO_STREAM("Somehow couldn't open this file for writing "
                    << costs_file_name);
  }

  ompl::mod::MoDOptimizationObjectivePtr concOptObj;

  std::vector<std::array<double, 3>> poses;
}

int main(int argn, char *args[]) {
  ros::init(argn, args, "paths_to_costs");

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
  const std::string scenarios[4] = {"corridor1", "corridor2", "corridor3",
                                    "corridor4"};
  const std::string times[3] = {"t1", "t2", "t3"};

  std::vector<std::string> all_path_files;

  for (const std::string &scenario : scenarios) {
    for (const std::string &time : times) {
      std::string this_folder = folder_path + "/" + scenario + "/" + time;
      auto this_files = getFilesMatchingPattern(this_folder);
      all_path_files.insert(all_path_files.end(), this_files.begin(),
                            this_files.end());
    }
  }

  std::shared_ptr<ob::CarStateSpace> cStateSpace =
      std::make_shared<ob::CarStateSpace>(0.5);
  std::shared_ptr<ob::SpaceInformation> spaceInfo =
      std::make_shared<ob::SpaceInformation>(cStateSpace);

  auto cliffmap_client = std::make_shared<cliffmap_ros::CLiFFMapClient>();
  auto stefmap_client = std::make_shared<stefmap_ros::STeFMapClient>();
  auto gmmtmap_client = std::make_shared<gmmtmap_ros::GMMTMapClient>();

  double planning_time = 0.0;

  constexpr double weight_d = 1.0, weight_q = 1.0, weight_c = 0.1;

  ompl::mod::DTCOptimizationObjective DTCCostObjective(
      spaceInfo, cliffmap_client->get(), weight_d, weight_q, 0.02, 1.0);

  ompl::mod::UpstreamCriterionOptimizationObjective STeFUpstreamCostObjective(
      spaceInfo, stefmap_client->get(planning_time, 2), weight_c, weight_q,
      weight_c);

  ompl::mod::UpstreamCriterionOptimizationObjective GMMTUpstreamCostObjective(
      spaceInfo, gmmtmap_client->get(), weight_d, weight_q, weight_c);

  ompl::mod::UpstreamCriterionOptimizationObjective CLiFFUpstreamCostObjective(
      spaceInfo, cliffmap_client->get(), weight_d, weight_q, weight_c);

  ompl::mod::IntensityMapOptimizationObjective IntensityCostObjective(
      spaceInfo, "/home/ksatyaki/intensity_map_1m.xml", weight_d, weight_q,
      weight_c * 2);

  ROS_INFO_STREAM("All MoD Optimization Objectives initialized.");

  return 0;
}