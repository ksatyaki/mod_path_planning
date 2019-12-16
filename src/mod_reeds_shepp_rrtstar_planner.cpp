/*
 *   Copyright (c) Chittaranjan Srinivas Swaminathan
 *   This file is part of mod_path_planning.
 *
 *   ompl_planners_ros is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU Lesser General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   ompl_planners_ros is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU Lesser General Public License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with ompl_planners_ros.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <ros/console.h>
#include <ros/ros.h>

#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseArray.h>
#include <mod_path_planning/MoDPlanningGoal.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>

#include <mrpt/math/CPolygon.h>

#include "ompl_planners_ros/mc_reeds_shepp_car_planner.hpp"
#include "ompl_planners_ros/visualization.hpp"

#include <cliffmap_ros/cliffmap.hpp>
#include <gmmtmap_ros/gmmtmap.hpp>
#include <stefmap_ros/stefmap.hpp>
#include <whytemap_ros/whytemap.hpp>

#include "ompl/mod/objectives/DTCOptimizationObjective.h"
#include "ompl/mod/objectives/DTWOptimizationObjective.h"
#include "ompl/mod/objectives/MoDOptimizationObjective.h"
#include "ompl/mod/objectives/UpstreamCriterionOptimizationObjective.h"

std::queue<mod_path_planning::MoDPlanningGoalConstPtr> goal_queue_;

class MoDReedsSheppRRTStarPlanner {
protected:
  // ros::NodeHandles
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // What MoD is the planning going to happen for?
  ompl::mod::MapType mod_type_;

  // Parameters of the vehicle and planning
  ompl_planners_ros::PlannerParameters pp;
  ompl_planners_ros::VehicleParameters vp;

  // MoD clients
  std::shared_ptr<stefmap_ros::STeFMapClient> stefmap_client;
  std::shared_ptr<cliffmap_ros::CLiFFMapClient> cliffmap_client;
  std::shared_ptr<gmmtmap_ros::GMMTMapClient> gmmtmap_client;
  std::shared_ptr<whytemap_ros::WHyTeMapClient> whytemap_client;

  // Occupancy map client
  ros::ServiceClient map_client;

  // Solution will be stored here after plan is computed.
  boost::shared_ptr<std::vector<geometry_msgs::Pose2D>> solution_poses_ptr;

  // Reeds-Shepp car planner
  boost::shared_ptr<ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>
      planner;

  // How long should we plan
  double planning_time_;

  ob::OptimizationObjectivePtr MoDUnawareCostObjective;
  ob::OptimizationObjectivePtr DTCCostObjective;
  ob::OptimizationObjectivePtr DTWCostObjective;
  ob::OptimizationObjectivePtr CLiFFUpstreamCostObjective;
  ob::OptimizationObjectivePtr STeFUpstreamCostObjective;
  ob::OptimizationObjectivePtr GMMTUpstreamCostObjective;
  ob::OptimizationObjectivePtr WHyTeUpstreamCostObjective;

public:
  MoDReedsSheppRRTStarPlanner(ompl::mod::MapType map_type, double time = 0.0,
                              double weight_c = 0.2, bool upstream = false)
      : planning_time_(time), nh("~"), mod_type_(map_type) {
    map_client = private_nh.serviceClient<nav_msgs::GetMap>("static_map");
    map_client.waitForExistence();

    nav_msgs::GetMap get_map_;
    if (!map_client.call(get_map_)) {
      ROS_FATAL("Failed to call the map server for map!");
    }

    ROS_INFO_STREAM("[PLANNER]: Map received!");

    nh.getParam("planner/weight_d", pp.weight_d);
    pp.weight_c = weight_c;
    nh.getParam("planner/weight_q", pp.weight_q);
    nh.getParam("planner/planning_time", pp.planning_time);
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
    ROS_INFO_STREAM("*** wd: " << std::setw(shitwidth) << pp.weight_d
                               << " ***");
    ROS_INFO_STREAM("*** wq: " << std::setw(shitwidth) << pp.weight_q
                               << " ***");
    ROS_INFO_STREAM("*** wc: " << std::setw(shitwidth) << pp.weight_c
                               << " ***");
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

    // Start all clients:
    cliffmap_client = std::make_shared<cliffmap_ros::CLiFFMapClient>();
    stefmap_client = std::make_shared<stefmap_ros::STeFMapClient>();
    gmmtmap_client = std::make_shared<gmmtmap_ros::GMMTMapClient>();
    whytemap_client = std::make_shared<whytemap_ros::WHyTeMapClient>();

    // Get the occupancy map
    nav_msgs::OccupancyGridPtr occ_map =
        nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    *occ_map = get_map_.response.map;
    planner = boost::make_shared<
        ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>(pp, vp,
                                                                occ_map);

    DTCCostObjective =
        ob::OptimizationObjectivePtr(new ompl::mod::DTCOptimizationObjective(
            planner->ss->getSpaceInformation(), cliffmap_client->get(),
            pp.weight_d, pp.weight_q, pp.weight_c, vp.max_vehicle_speed));

    DTWCostObjective =
        ob::OptimizationObjectivePtr(new ompl::mod::DTWOptimizationObjective(
            planner->ss->getSpaceInformation(), whytemap_client->get(),
            pp.weight_d, pp.weight_q, pp.weight_c, vp.max_vehicle_speed));
    std::dynamic_pointer_cast<ompl::mod::DTWOptimizationObjective>(
        DTWCostObjective)
        ->setTimeStamp(planning_time_);

    STeFUpstreamCostObjective = ob::OptimizationObjectivePtr(
        new ompl::mod::UpstreamCriterionOptimizationObjective(
            planner->ss->getSpaceInformation(),
            stefmap_client->get(planning_time_, 2, -10, 3, 0, 14, 0.5),
            pp.weight_d, pp.weight_q, pp.weight_c));

    GMMTUpstreamCostObjective = ob::OptimizationObjectivePtr(
        new ompl::mod::UpstreamCriterionOptimizationObjective(
            planner->ss->getSpaceInformation(), gmmtmap_client->get(),
            pp.weight_d, pp.weight_q, pp.weight_c));

    CLiFFUpstreamCostObjective = ob::OptimizationObjectivePtr(
        new ompl::mod::UpstreamCriterionOptimizationObjective(
            planner->ss->getSpaceInformation(), cliffmap_client->get(),
            pp.weight_d, pp.weight_q, pp.weight_c));

    // Init WHyTeUpstreamCostObjective

    MoDUnawareCostObjective = ob::OptimizationObjectivePtr(
        new ompl::mod::UpstreamCriterionOptimizationObjective(
            planner->ss->getSpaceInformation(), cliffmap_client->get(),
            pp.weight_d, pp.weight_q, 0.0));

    if (mod_type_ == ompl::mod::MapType::WHyTeMap && not upstream) {
      ROS_INFO_STREAM("\x1b[34mWHyTe-map planning is activated with DTW cost.");
      planner->ss->setOptimizationObjective(DTWCostObjective);
    } else if (mod_type_ == ompl::mod::MapType::CLiFFMap && not upstream) {
      ROS_INFO_STREAM("\x1b[34mCLiFF-map planning is activated with DTC cost.");
      planner->ss->setOptimizationObjective(DTCCostObjective);
    } else if (mod_type_ == ompl::mod::MapType::STeFMap) {
      ROS_INFO_STREAM("\x1b[34mSTeF-map planning is activated.");
      planner->ss->setOptimizationObjective(STeFUpstreamCostObjective);
    } else if (mod_type_ == ompl::mod::MapType::GMMTMap) {
      ROS_INFO_STREAM("\x1b[34mGMMT-map planning is activated.");
      planner->ss->setOptimizationObjective(GMMTUpstreamCostObjective);
    } else if (mod_type_ == ompl::mod::MapType::CLiFFMap && upstream) {
      ROS_INFO_STREAM(
          "\x1b[34mCLiFF-map planning is activated with upstream cost.");
      planner->ss->setOptimizationObjective(CLiFFUpstreamCostObjective);
    } else if (mod_type_ == ompl::mod::MapType::WHyTeMap && upstream) {
      ROS_INFO_STREAM(
          "\x1b[34mWHyTe-map planning is activated with upstream cost.");
      planner->ss->setOptimizationObjective(WHyTeUpstreamCostObjective);
    } else {
      ROS_INFO_STREAM("\x1b[34m MoD-unaware planning is activated.");
      planner->ss->setOptimizationObjective(MoDUnawareCostObjective);
    }
  }

  std::string getMapTypeStr() {
    return std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
               planner->ss->getOptimizationObjective())
        ->getMapTypeStr();
  }

  virtual ~MoDReedsSheppRRTStarPlanner() = default;

  void plan(const geometry_msgs::Pose2D &start,
            const geometry_msgs::Pose2D &goal) {
    planner->ss->getProblemDefinition()->setIntermediateSolutionCallback(
        boost::bind(&MoDReedsSheppRRTStarPlanner::solutionCallback, this, _1,
                    _2, _3, start, goal));

    solution_poses_ptr.reset();
    solution_poses_ptr =
        boost::make_shared<std::vector<geometry_msgs::Pose2D>>();
    planner->plan(start, goal, solution_poses_ptr.get());
  }

  double getDTCCost() {
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            DTCCostObjective);
    auto costs = getSolutionCostComponents(opt_obj);
    return costs.cost_c_;
  }

  double getCLiFFUpstreamCost() {
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            CLiFFUpstreamCostObjective);
    auto costs = getSolutionCostComponents(opt_obj);
    return costs.cost_c_;
  }

  double getSTeFUpstreamCost() {
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            STeFUpstreamCostObjective);
    auto costs = getSolutionCostComponents(opt_obj);
    return costs.cost_c_;
  }

  double getGMMTUpstreamCost() {
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            GMMTUpstreamCostObjective);
    auto costs = getSolutionCostComponents(opt_obj);
    return costs.cost_c_;
  }

  double getWHyTeUpstreamCost() {
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            WHyTeUpstreamCostObjective);
    auto costs = getSolutionCostComponents(opt_obj);
    return costs.cost_c_;
  }

  double getDTWCost() {
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            DTWCostObjective);
    auto costs = getSolutionCostComponents(opt_obj);
    return costs.cost_c_;
  }

  double getSolutionCost(ompl::mod::MoDOptimizationObjectivePtr opt_obj) {
    auto space_info = this->planner->ss->getSpaceInformation();

    std::vector<ompl::base::State *> states =
        planner->ss->getSolutionPath().getStates();

    double total_cost = 0.0;
    for (size_t i = 0; i < (states.size() - 1); i++) {
      auto this_cost = opt_obj->motionCost(states[i], states[i + 1]);
      total_cost = total_cost + this_cost.value();
    }

    return total_cost;
  }

  ompl::mod::Cost
  getSolutionCostComponents(ompl::mod::MoDOptimizationObjectivePtr opt_obj) {
    auto space_info = this->planner->ss->getSpaceInformation();

    std::vector<ompl::base::State *> states =
        planner->ss->getSolutionPath().getStates();

    ompl::mod::Cost all_costs;
    for (size_t i = 0; i < (states.size() - 1); i++) {
      auto this_cost = opt_obj->motionCost(states[i], states[i + 1]);
      all_costs = all_costs + opt_obj->getLastCost();
    }

    return all_costs;
  }

  void savePathCallback(const std_msgs::String &msg) {
    ROS_INFO_STREAM("\x1b[34mSaving path to file: " << msg.data.c_str());
    std::ofstream ofile(msg.data.c_str(), std::ios::out);
    if (!ofile) {
      ROS_ERROR_STREAM("Couldn't open paths file!");
      return;
    }

    for (const auto &pose : *solution_poses_ptr) {
      ofile << pose.x << ", " << pose.y << ", " << pose.theta << std::endl;
    }

    ofile.close();
    ROS_INFO_STREAM("Successfully saved path to file: " << msg.data.c_str());
  }

  ompl::mod::MoDOptimizationObjectivePtr getOptimizationObjective() {
    return std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
        planner->ss->getOptimizationObjective());
  }

  void solutionCallback(
      const ompl::base::Planner *planner,
      const std::vector<const ompl::base::State *> &solution_states,
      const ompl::base::Cost cost, const geometry_msgs::Pose2D &start,
      const geometry_msgs::Pose2D &goal) {

    ROS_INFO_STREAM("New solution found with cost: \x1b[34m"
                    << std::fixed << std::setprecision(2) << cost.value());

    auto space_info = planner->getSpaceInformation();
    auto opt_obj =
        std::dynamic_pointer_cast<ompl::mod::MoDOptimizationObjective>(
            planner->getProblemDefinition()->getOptimizationObjective());

    // Allocate new ompl::base::State* for storing the start and goal states.
    ompl::base::State *start_state = space_info->allocState();
    ompl::base::State *goal_state = space_info->allocState();

    // Create start ompl::base::State*
    (start_state->as<ompl::base::ReedsSheppStateSpace::StateType>())
        ->setX(start.x);
    (start_state->as<ompl::base::ReedsSheppStateSpace::StateType>())
        ->setY(start.y);
    (start_state->as<ompl::base::ReedsSheppStateSpace::StateType>())
        ->setYaw(start.theta);

    // Create goal ompl::base::State*
    (goal_state->as<ompl::base::ReedsSheppStateSpace::StateType>())
        ->setX(goal.x);
    (goal_state->as<ompl::base::ReedsSheppStateSpace::StateType>())
        ->setY(goal.y);
    (goal_state->as<ompl::base::ReedsSheppStateSpace::StateType>())
        ->setYaw(goal.theta);

    // Add start and goal to the solution states.
    std::vector<const ompl::base::State *> copy_solution_states;
    copy_solution_states.insert(copy_solution_states.begin(), goal_state);
    for (const auto s : solution_states) {
      copy_solution_states.insert(copy_solution_states.begin(), s);
    }
    copy_solution_states.insert(copy_solution_states.begin(), start_state);

    // Compute costs.
    ompl::mod::Cost total_path_cost;
    double total_cost = 0.0;
    for (size_t i = 0; i < (copy_solution_states.size() - 1); i++) {
      auto this_cost = opt_obj->motionCost(copy_solution_states[i],
                                           copy_solution_states[i + 1]);
      total_cost = total_cost + this_cost.value();
      total_path_cost = total_path_cost + opt_obj->getLastCost();
    }

    // Free the start and goal.
    space_info->freeState(start_state);
    space_info->freeState(goal_state);

    ROS_INFO("Solution total cost: %lf", total_cost);
    //ROS_INFO("Solution cost components: cost_d: %lf, cost_q: %lf, cost_c: %lf",
    //         total_path_cost.cost_d_, total_path_cost.cost_q_,
    //         total_path_cost.cost_c_);
  }
};

void modGoalCallback(const mod_path_planning::MoDPlanningGoalConstPtr &msg) {
  ROS_INFO("Adding a goal to the queue.");
  goal_queue_.push(msg);
}

int main(int argn, char *args[]) {
  ros::init(argn, args, "mod_path_planning_node");

  ros::NodeHandle nhandle;

  ros::Subscriber mod_planning_sub =
      nhandle.subscribe("/mod_planning_goal", 10, &modGoalCallback);

  ros::Rate rate(1);
  ros::AsyncSpinner spinner(4);
  spinner.start();

  while (ros::ok()) {
    if (not goal_queue_.empty()) {
      const mod_path_planning::MoDPlanningGoalConstPtr goal =
          goal_queue_.front();
      goal_queue_.pop();

      ompl::mod::MapType planningGoalMapType;
      switch (goal->mod_type) {
      case mod_path_planning::MoDPlanningGoal::CLIFF:
        planningGoalMapType = ompl::mod::MapType::CLiFFMap;
        break;
      case mod_path_planning::MoDPlanningGoal::STEF:
        planningGoalMapType = ompl::mod::MapType::STeFMap;
        break;
      case mod_path_planning::MoDPlanningGoal::WHYTE:
        planningGoalMapType = ompl::mod::MapType::WHyTeMap;
        break;
      case mod_path_planning::MoDPlanningGoal::GMMT:
        planningGoalMapType = ompl::mod::MapType::GMMTMap;
        break;
      default:
        planningGoalMapType = ompl::mod::MapType::NOTSET;
        break;
      }

      // Instantiate planner
      MoDReedsSheppRRTStarPlanner mod_rs_rrtstar_planner(
          planningGoalMapType, goal->header.stamp.toSec(), goal->weight_c,
          goal->upstream);

      // Start planning
      mod_rs_rrtstar_planner.plan(goal->start, goal->goal);

      // Save path.
      std_msgs::String save_path_msg;
      char fileName[100];
      sprintf(fileName, "/home/ksatyaki/.ros/savedPaths/%s_%d.path",
              mod_rs_rrtstar_planner.getMapTypeStr().c_str(), goal->header.seq);
      save_path_msg.data = fileName;
      mod_rs_rrtstar_planner.savePathCallback(save_path_msg);

      // Save statistics.
      ROS_INFO("Solution total cost: %lf",
               mod_rs_rrtstar_planner.getSolutionCost(
                   mod_rs_rrtstar_planner.getOptimizationObjective()));
      ROS_INFO(
          "Solution cost components: "
          "cost_d: %lf, "
          "cost_q: %lf, "
          "DTC: %lf, "
          "DTW: %lf, "
          "CLiFFUpstream: %lf, "
          "STeFUpstream: %lf, "
          "GMMTUpstream: %lf, "
          "WHyTeUpstream: %lf.",
          mod_rs_rrtstar_planner.getSolutionCostComponents(mod_rs_rrtstar_planner.getOptimizationObjective()).cost_d_,
          mod_rs_rrtstar_planner.getSolutionCostComponents(mod_rs_rrtstar_planner.getOptimizationObjective()).cost_q_,
          mod_rs_rrtstar_planner.getDTCCost(),
          mod_rs_rrtstar_planner.getDTWCost(),
          mod_rs_rrtstar_planner.getCLiFFUpstreamCost(),
          mod_rs_rrtstar_planner.getSTeFUpstreamCost(),
          mod_rs_rrtstar_planner.getGMMTUpstreamCost(),
          mod_rs_rrtstar_planner.getWHyTeUpstreamCost());
    } else {
      ROS_INFO("No goals in queue yet.");
      rate.sleep();
    }
  }

  ROS_INFO("ROS Shutdown. Done.");

  return 0;
}
