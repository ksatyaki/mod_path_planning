#include <ompl_planners_ros/mc_reeds_shepp_car_planner.hpp>

#include <cliffmap_ros/cliffmap.hpp>
#include <costmap_2d/costmap_2d_ros.h>
#include <gmmtmap_ros/gmmtmap.hpp>
#include <nav_msgs/GetMap.h>
#include <ompl/tools/benchmark/Benchmark.h>

#include "ompl/mod/objectives/DTCOptimizationObjective.h"
#include "ompl/mod/objectives/IntensityMapOptimizationObjective.h"
#include "ompl/mod/objectives/MoDOptimizationObjective.h"
#include "ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h"
#include "ompl_planners_ros/visualization.hpp"

#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

ompl::base::PlannerPtr
getConfiguredRRTstar(const ompl::base::SpaceInformationPtr &si) {

  auto *planner = new ompl::geometric::RRTstar(si);
  planner->setKNearest(false);
  planner->setRange(si->getStateSpace()->getMaximumExtent());
  return ompl::base::PlannerPtr(planner);
}

ompl::base::PlannerPtr
getConfiguredInformedRRTstar(const ompl::base::SpaceInformationPtr &si) {

  auto *planner = new ompl::geometric::InformedRRTstar(si);
  planner->setKNearest(false);
  planner->setRange(si->getStateSpace()->getMaximumExtent());
  return ompl::base::PlannerPtr(planner);
}

ompl::base::PlannerPtr
getConfiguredPRMstar(const ompl::base::SpaceInformationPtr &si) {

  auto *planner = new ompl::geometric::PRMstar(si);
  return ompl::base::PlannerPtr(planner);
}

int main(int argn, char *args[]) {

  ros::init(argn, args, "mod_path_planning_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // Weight parameters
  // weight_upstream: 0.1
  // weight_dtc: 0.02
  // weight_intensity: 0.2

  double weight_upstream = 0.1;
  double weight_dtc = 0.02;
  double weight_intensity = 0.2;

  // MoD Clients
  std::shared_ptr<stefmap_ros::STeFMapClient> stefmap_client;
  std::shared_ptr<cliffmap_ros::CLiFFMapClient> cliffmap_client;
  std::shared_ptr<gmmtmap_ros::GMMTMapClient> gmmtmap_client;

  ob::OptimizationObjectivePtr DTCCostObjective;
  ob::OptimizationObjectivePtr CLiFFUpstreamCostObjective;
  ob::OptimizationObjectivePtr STeFUpstreamCostObjective;
  ob::OptimizationObjectivePtr GMMTUpstreamCostObjective;
  ob::OptimizationObjectivePtr IntensityCostObjective;

  // Parameters of the vehicle and planning
  ompl_planners_ros::PlannerParameters pp;
  ompl_planners_ros::VehicleParameters vp;

  // Occupancy map client
  ros::ServiceClient map_client;
  map_client = private_nh.serviceClient<nav_msgs::GetMap>("static_map");
  map_client.waitForExistence();

  nav_msgs::GetMap get_map_;
  if (!map_client.call(get_map_)) {
    ROS_FATAL("Failed to call the map server for map!");
  }

  ROS_INFO_STREAM("[PLANNER]: Map received!");

  nh.getParam("planner/weight_upstream", weight_upstream);
  nh.getParam("planner/weight_dtc", weight_dtc);
  nh.getParam("planner/weight_intensity", weight_intensity);

  nh.getParam("planner/weight_d", pp.weight_d);
  nh.getParam("planner/weight_q", pp.weight_q);
  nh.getParam("planner/path_resolution", pp.path_resolution);
  nh.getParam("planner/publish_viz_markers", pp.publish_viz_markers);
  nh.getParam("vehile/inflation_radius", vp.inflation_radius);
  nh.getParam("vehicle/turning_radius", vp.turning_radius);

  std::vector<geometry_msgs::Point> ftprnt =
      costmap_2d::makeFootprintFromParams(nh);

  for (const auto &ftprntpt : ftprnt) {
    ROS_INFO_STREAM(std::fixed << std::setprecision(2) << "\x1b[34m("
                               << ftprntpt.x << "," << ftprntpt.y << ")");
    vp.footprint.AddVertex(ftprntpt.x, ftprntpt.y);
  }

  int shitwidth = 31;

  ROS_INFO_STREAM("*** *********************************** ***");
  ROS_INFO_STREAM("*** wd: " << std::setw(shitwidth) << pp.weight_d << " ***");
  ROS_INFO_STREAM("*** wq: " << std::setw(shitwidth) << pp.weight_q << " ***");
  ROS_INFO_STREAM("*** wc: " << std::setw(shitwidth) << pp.weight_c << " ***");
  ROS_INFO_STREAM("*** Publish viz markers? " << std::setw(shitwidth - 17)
                                              << pp.publish_viz_markers
                                              << " ***");
  ROS_INFO_STREAM("*** Planning time: " << std::setw(shitwidth - 11)
                                        << pp.planning_time << " ***");
  ROS_INFO_STREAM("*** Path resolution: " << std::setw(shitwidth - 13)
                                          << pp.path_resolution << " ***");
  ROS_INFO_STREAM("*** Turning radius: " << std::setw(shitwidth - 12)
                                         << vp.turning_radius << " ***");
  ROS_INFO_STREAM("*** Inflation radius: " << std::setw(shitwidth - 14)
                                           << vp.inflation_radius << " ***");
  ROS_INFO_STREAM("*** Check footprint manually." << std::setw(shitwidth - 17)
                                                  << " ***");
  ROS_INFO_STREAM("*** *********************************** ***");

  // Start all clients
  cliffmap_client = std::make_shared<cliffmap_ros::CLiFFMapClient>();
  // stefmap_client = std::make_shared<stefmap_ros::STeFMapClient>();
  gmmtmap_client = std::make_shared<gmmtmap_ros::GMMTMapClient>();

  // Get the occupancy map
  nav_msgs::OccupancyGridPtr occ_map =
      nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
  *occ_map = get_map_.response.map;

  auto ss_dtc = ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner::
      generateSimpleSetup(pp, vp, occ_map);
  auto ss_intensity = ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner::
      generateSimpleSetup(pp, vp, occ_map);
  auto ss_cliff = ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner::
      generateSimpleSetup(pp, vp, occ_map);
  auto ss_gmmt = ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner::
      generateSimpleSetup(pp, vp, occ_map);

  DTCCostObjective =
      ob::OptimizationObjectivePtr(new ompl::mod::DTCOptimizationObjective(
          ss_dtc->getSpaceInformation(), cliffmap_client->get(), pp.weight_d,
          pp.weight_q, weight_dtc, vp.max_vehicle_speed));

  GMMTUpstreamCostObjective = ob::OptimizationObjectivePtr(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          ss_gmmt->getSpaceInformation(), gmmtmap_client->get(), pp.weight_d,
          pp.weight_q, weight_upstream));

  CLiFFUpstreamCostObjective = ob::OptimizationObjectivePtr(
      new ompl::mod::UpstreamCriterionOptimizationObjective(
          ss_cliff->getSpaceInformation(), cliffmap_client->get(), pp.weight_d,
          pp.weight_q, weight_upstream));

  IntensityCostObjective = ob::OptimizationObjectivePtr(
      new ompl::mod::IntensityMapOptimizationObjective(
          ss_intensity->getSpaceInformation(),
          "/home/ksatyaki/intensity_map_1m.xml", pp.weight_d,
          pp.weight_q, weight_intensity));

  ompl::tools::Benchmark bm_cliff(*ss_cliff, "CLiFF-benchmark");
  ompl::tools::Benchmark bm_gmmt(*ss_gmmt, "GMMT-benchmark");
  ompl::tools::Benchmark bm_dtc(*ss_dtc, "DTC-benchmark");
  ompl::tools::Benchmark bm_intensity(*ss_intensity, "Intensity-benchmark");

  ss_cliff->setOptimizationObjective(CLiFFUpstreamCostObjective);
  ss_gmmt->setOptimizationObjective(GMMTUpstreamCostObjective);
  ss_dtc->setOptimizationObjective(DTCCostObjective);
  ss_intensity->setOptimizationObjective(IntensityCostObjective);

  bm_cliff.addPlannerAllocator(
      std::bind(&getConfiguredRRTstar, std::placeholders::_1));
  bm_cliff.addPlannerAllocator(
      std::bind(&getConfiguredPRMstar, std::placeholders::_1));
  bm_cliff.addPlannerAllocator(
      std::bind(&getConfiguredInformedRRTstar, std::placeholders::_1));

  bm_dtc.addPlannerAllocator(
      std::bind(&getConfiguredRRTstar, std::placeholders::_1));
  bm_dtc.addPlannerAllocator(
      std::bind(&getConfiguredPRMstar, std::placeholders::_1));
  bm_dtc.addPlannerAllocator(
      std::bind(&getConfiguredInformedRRTstar, std::placeholders::_1));

  bm_gmmt.addPlannerAllocator(
      std::bind(&getConfiguredRRTstar, std::placeholders::_1));
  bm_gmmt.addPlannerAllocator(
      std::bind(&getConfiguredPRMstar, std::placeholders::_1));
  bm_gmmt.addPlannerAllocator(
      std::bind(&getConfiguredInformedRRTstar, std::placeholders::_1));

  bm_intensity.addPlannerAllocator(
      std::bind(&getConfiguredRRTstar, std::placeholders::_1));
  bm_intensity.addPlannerAllocator(
      std::bind(&getConfiguredPRMstar, std::placeholders::_1));
  bm_intensity.addPlannerAllocator(
      std::bind(&getConfiguredInformedRRTstar, std::placeholders::_1));

  ompl::tools::Benchmark::Request req;
  req.maxTime = 60.0;
  req.maxMem = 6000.0;
  req.runCount = 15;
  req.displayProgress = true;
  req.simplify = false;
  req.timeBetweenUpdates = 0.001;

  std::shared_ptr<ob::ScopedState<>> start[3];
  std::shared_ptr<ob::ScopedState<>> goal[3];

  for (size_t i = 0; i < 3; i++) {
    start[i] = std::make_shared<ob::ScopedState<>>(ss_cliff->getStateSpace());
    goal[i] = std::make_shared<ob::ScopedState<>>(ss_cliff->getStateSpace());
  }

  // Start: 2,018, 2,162, 0,059;
  // Goal:  26,457, 17,616, 1,386;

  // Start1: 0.416, 27.676, -0.425;
  // Goal1: 30.404, 2.929, -1.571;

  // Start2: 23.804, 2.444, 3.086;
  // Goal2: 25.163, 27.482, 1.571;

  // start: {x: 47.690, y: -18.848, theta: -2.356}
  // goal: {x: -19.575, y: 12.390, theta: 2.313}

  // goal: {x: -3.85, y: -6.82, theta: -0.8203}
  // start: {x: 21.471, y: -17.647, theta: 0.733}

  // start: {x: 11.5, y: -5.0, theta: 2.604}
  // goal: {x: -25.00, y: 3.00, theta: -1.999}

  (*start[0])[0] = 47.690; (*start[0])[1] = -18.848; (*start[0])[2] = -2.356;
  (*goal[0])[0] = -19.575; (*goal[0])[1] = 12.390; (*goal[0])[2] = 2.313;


  (*start[1])[0] = -3.85; (*start[1])[1] = -6.82; (*start[1])[2] = -0.8203;
  (*goal[1])[0] = 21.471; (*goal[1])[1] = -17.647; (*goal[1])[2] = 0.733;


  (*start[2])[0] = 11.5; (*start[2])[1] = -5.0; (*start[2])[2] = 2.604;
  (*goal[2])[0] = -25.00; (*goal[2])[1] = 3.00; (*goal[2])[2] = -1.999;


  char fileName[256];

  ros::Time t = ros::Time::now();

  for (int number = 0; number < 3; number++) {
    ss_cliff->setStartAndGoalStates(*start[number], *goal[number]);
    bm_cliff.benchmark(req);
    sprintf(fileName, "/home/ksatyaki/prelim_results/%lf-CLiFF-%d-results.log",
            t.toSec(), number);
    bm_cliff.saveResultsToFile(fileName);

    ss_dtc->setStartAndGoalStates(*start[number], *goal[number]);
    bm_dtc.benchmark(req);
    sprintf(fileName, "/home/ksatyaki/prelim_results/%lf-DTC-%d-results.log",
            t.toSec(), number);
    bm_dtc.saveResultsToFile(fileName);

    ss_gmmt->setStartAndGoalStates(*start[number], *goal[number]);
    bm_gmmt.benchmark(req);
    sprintf(fileName, "/home/ksatyaki/prelim_results/%lf-GMMT-%d-results.log",
            t.toSec(), number);
    bm_gmmt.saveResultsToFile(fileName);

    ss_intensity->setStartAndGoalStates(*start[number], *goal[number]);
    bm_intensity.benchmark(req);
    sprintf(fileName,
            "/home/ksatyaki/prelim_results/%lf-intensity-%d-results.log",
            t.toSec(), number);
    bm_intensity.saveResultsToFile(fileName);
  }

  std::cout << "... DONE ..." << std::endl;
  return 0;
}