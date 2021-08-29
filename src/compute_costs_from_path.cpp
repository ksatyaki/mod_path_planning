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
    const std::string &target_path, const std::string &extension = ".path") {
  // const boost::regex my_filter(pattern);

  std::vector<std::string> all_matching_files;

  boost::filesystem::directory_iterator end_itr;
  for (boost::filesystem::directory_iterator i(target_path); i != end_itr;
       ++i) {
    if (!boost::filesystem::is_regular_file(i->status())) continue;

    boost::smatch what;
    if (i->path().filename().extension() != extension) continue;

    all_matching_files.push_back(i->path().string());
  }
  return all_matching_files;
}

ob::ScopedState<ob::CarStateSpace> poseFromStr(
    const std::vector<std::string> &str,
    const std::shared_ptr<ob::CarStateSpace> &cStateSpacePtr) {
  ob::ScopedState<ob::CarStateSpace> scopedState(cStateSpacePtr);
  scopedState[0] = std::stod(str[0]);
  scopedState[1] = std::stod(str[1]);
  scopedState[2] = std::stod(str[2]);
  return scopedState;
}

void computeCostsFromCSV(
    const std::string &csv_file_name,
    const std::array<ompl::mod::MoDOptimizationObjectivePtr, 7> &ptrs,
    const std::shared_ptr<ob::CarStateSpace> &cStateSpace) {
  std::string costs_file_name = csv_file_name;
  boost::replace_all(costs_file_name, ".path", ".costs_file");

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

  std::vector<ob::ScopedState<ob::CarStateSpace>> poses;
  std::string single_line;
  while (std::getline(path_file, single_line)) {
    std::vector<std::string> pose_str;
    single_line.erase(
        std::remove_if(single_line.begin(), single_line.end(),
                       [](unsigned char x) { return std::isspace(x); }));
    boost::split(pose_str, single_line, boost::is_any_of(","));
    if (pose_str.size() != 3) {
      ROS_ERROR_STREAM(
          "Pose size isn't 3 somehow! It is instead: " << pose_str.size());
    }
    poses.push_back(poseFromStr(pose_str, cStateSpace));
  }

  costs_file << std::fixed << std::setprecision(4);
  for (size_t i = 1; i < poses.size(); i++) {
    auto cost0 = ptrs[0]->motionCost(poses[i - 1].get(), poses[i].get());
    auto cost1 = ptrs[1]->motionCost(poses[i - 1].get(), poses[i].get());
    auto cost2 = ptrs[2]->motionCost(poses[i - 1].get(), poses[i].get());
    auto cost3 = ptrs[3]->motionCost(poses[i - 1].get(), poses[i].get());
    auto cost4 = ptrs[4]->motionCost(poses[i - 1].get(), poses[i].get());
    auto cost5 = ptrs[5]->motionCost(poses[i - 1].get(), poses[i].get());
    auto cost6 = ptrs[6]->motionCost(poses[i - 1].get(), poses[i].get());

    costs_file << ptrs[0]->getLastCostD() << ", " << ptrs[0]->getLastCostQ()
               << ", " << ptrs[0]->getLastCostC() << ", "
               << ptrs[1]->getLastCostC() << ", " << ptrs[2]->getLastCostC()
               << ", " << ptrs[3]->getLastCostC() << ", "
               << ptrs[4]->getLastCostC() << ", " << ptrs[5]->getLastCostC()
               << ", " << ptrs[6]->getLastCostC() << ", " << cost0 << ", "
               << cost1 << ", " << cost2 << ", " << cost3 << ", " << cost4
               << ", " << cost5 << ", " << cost6 << "\n";
  }
  costs_file.close();
  path_file.close();
  poses.clear();
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
  constexpr double time_point1 = 1352866100;
  constexpr double time_point2 = 1352883600;
  constexpr double time_point3 = 1352890000;

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
      std::make_shared<ob::CarStateSpace>(0.5, false);
  ob::SpaceInformationPtr spaceInfo =
      std::make_shared<ob::SpaceInformation>(cStateSpace);

  spaceInfo->setStateValidityCheckingResolution(
      0.01 / cStateSpace->getMaximumExtent());

  auto cliffmap_client = std::make_shared<cliffmap_ros::CLiFFMapClient>();
  auto stefmap_client = std::make_shared<stefmap_ros::STeFMapClient>();
  auto gmmtmap_client = std::make_shared<gmmtmap_ros::GMMTMapClient>();

  constexpr double weight_d = 1.0, weight_q = 1.0, weight_c = 0.1;

  ompl::mod::MoDOptimizationObjectivePtr DTCCostObjective(
      new ompl::mod::DTCOptimizationObjective(spaceInfo, cliffmap_client->get(),
                                              weight_d, weight_q, 0.02, 1.0));

  ompl::mod::MoDOptimizationObjectivePtr STeFUpstreamCostObjective1(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          spaceInfo, stefmap_client->get(time_point1, 2), weight_d, weight_q,
          weight_c));

  ompl::mod::MoDOptimizationObjectivePtr STeFUpstreamCostObjective2(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          spaceInfo, stefmap_client->get(time_point2, 2), weight_d, weight_q,
          weight_c));

  ompl::mod::MoDOptimizationObjectivePtr STeFUpstreamCostObjective3(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          spaceInfo, stefmap_client->get(time_point3, 2), weight_d, weight_q,
          weight_c));

  ompl::mod::MoDOptimizationObjectivePtr GMMTUpstreamCostObjective(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          spaceInfo, gmmtmap_client->get(), weight_d, weight_q, weight_c));

  ompl::mod::MoDOptimizationObjectivePtr CLiFFUpstreamCostObjective(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          spaceInfo, cliffmap_client->get(), weight_d, weight_q, weight_c));

  ompl::mod::MoDOptimizationObjectivePtr IntensityCostObjective(
      new ompl::mod::IntensityMapOptimizationObjective(
          spaceInfo, "/home/ksatyaki/intensity_map_1m.xml", weight_d, weight_q,
          weight_c * 2));

  std::array<ompl::mod::MoDOptimizationObjectivePtr, 7> ptrs = {
      DTCCostObjective,           CLiFFUpstreamCostObjective,
      STeFUpstreamCostObjective1, STeFUpstreamCostObjective2,
      STeFUpstreamCostObjective3, GMMTUpstreamCostObjective,
      IntensityCostObjective};

  ROS_INFO_STREAM("All MoD Optimization Objectives initialized.");

  size_t files_completed = 0;
  for (const auto &fileName : all_path_files) {
    computeCostsFromCSV(fileName, ptrs, cStateSpace);
    if (files_completed % 20 == 0)
      std::printf("%ld cost files computed out of %ld...\r", files_completed,
                  all_path_files.size());
    files_completed++;
  }
  return 0;
}