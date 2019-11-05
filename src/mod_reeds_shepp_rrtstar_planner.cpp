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
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <mod_path_planning/MoDPlanningGoal.h>

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
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;
  ompl::mod::MapType mod_type_;

  ompl_planners_ros::PlannerParameters pp;
  ompl_planners_ros::VehicleParameters vp;

  std::shared_ptr<stefmap_ros::STeFMapClient> stefmap_client;
  std::shared_ptr<cliffmap_ros::CLiFFMapClient> cliffmap_client;
  std::shared_ptr<gmmtmap_ros::GMMTMapClient> gmmtmap_client;
  std::shared_ptr<whytemap_ros::WHyTeMapClient> whytemap_client;

  ros::ServiceClient map_client;

  geometry_msgs::PoseArrayPtr poses_ptr;
  boost::shared_ptr<ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>
      planner;

  double planning_time_;

public:
  MoDReedsSheppRRTStarPlanner(ompl::mod::MapType map_type, double time = 0.0)
      : planning_time_(time), nh("~") {
    map_client = private_nh.serviceClient<nav_msgs::GetMap>("static_map");
    map_client.waitForExistence();

    nav_msgs::GetMap get_map_;
    if (!map_client.call(get_map_)) {
      ROS_FATAL("Failed to call the map server for map!");
    }

    ROS_INFO_STREAM("[PLANNER]: Map received!");

    nh.getParam("planner/weight_d", pp.weight_d);
    nh.getParam("planner/weight_c", pp.weight_c);
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

    std::string mod_type;

    switch (this->mod_type_) {
    case ompl::mod::MapType::CLiFFMap:
      mod_type = "CLiFFMap";
      break;
    case ompl::mod::MapType::STeFMap:
      mod_type = "STeFMap";
      break;
    case ompl::mod::MapType::GMMTMap:
      mod_type = "GMMTMap";
      break;
    case ompl::mod::MapType::WHyTeMap:
      mod_type = "WHyTeMap";
      break;
    case ompl::mod::MapType::NOTSET:
      mod_type = "None";
      break;
    }

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
    ROS_INFO_STREAM("*** MoD type: " << std::setw(shitwidth - 6)
                                     << mod_type.c_str() << " ***");
    ROS_INFO_STREAM("*** Turning radius: " << std::setw(shitwidth - 12)
                                           << vp.turning_radius << " ***");
    ROS_INFO_STREAM("*** Inflation radius: " << std::setw(shitwidth - 14)
                                             << vp.inflation_radius << " ***");
    ROS_INFO_STREAM("*** Check footprint manually." << std::setw(shitwidth - 17)
                                                    << " ***");
    ROS_INFO_STREAM("*** *********************************** ***");

    nav_msgs::OccupancyGridPtr occ_map =
        nav_msgs::OccupancyGridPtr(new nav_msgs::OccupancyGrid);
    *occ_map = get_map_.response.map;
    planner = boost::make_shared<
        ompl_planners_ros::MultipleCirclesReedsSheppCarPlanner>(pp, vp,
                                                                occ_map);
    planner->ss->getProblemDefinition()->setIntermediateSolutionCallback(
        boost::bind(&MoDReedsSheppRRTStarPlanner::solutionCallback, this, _1,
                    _2, _3));

    if (mod_type == "CLiFF-map" or mod_type == "CLiFFMap") {
      ROS_INFO_STREAM("\x1b[34mCLiFF-map planning is activated.");

      cliffmap_client = std::make_shared<cliffmap_ros::CLiFFMapClient>();

      ob::OptimizationObjectivePtr DTCCostObjective(
          new ompl::mod::DTCOptimizationObjective(
              planner->ss->getSpaceInformation(), cliffmap_client->get(),
              pp.weight_d, pp.weight_q, pp.weight_c, vp.max_vehicle_speed));

      planner->ss->setOptimizationObjective(DTCCostObjective);

    } else if (mod_type == "STeF-map" or mod_type == "STeFMap") {
      ROS_INFO_STREAM("\x1b[34mSTeF-map planning is activated.");

      stefmap_client = std::make_shared<stefmap_ros::STeFMapClient>();

      ob::OptimizationObjectivePtr UCOO(
          new ompl::mod::UpstreamCriterionOptimizationObjective(
              planner->ss->getSpaceInformation(),
              stefmap_client->get(planning_time_, 2, -45, 55, -35, 30, 1), pp.weight_d,
              pp.weight_q, pp.weight_c));

      planner->ss->setOptimizationObjective(UCOO);

    } else if (mod_type == "GMMT-map" or mod_type == "GMMTMap") {
      ROS_INFO_STREAM("\x1b[34mGMMT-map planning is activated.");

      gmmtmap_client = std::make_shared<gmmtmap_ros::GMMTMapClient>();

      ob::OptimizationObjectivePtr UCOO(
          new ompl::mod::UpstreamCriterionOptimizationObjective(
              planner->ss->getSpaceInformation(), gmmtmap_client->get(),
              pp.weight_d, pp.weight_q, pp.weight_c));
      planner->ss->setOptimizationObjective(UCOO);
    } else if (mod_type == "WHyTe-map" or mod_type == "WHyTeMap") {
      ROS_INFO_STREAM("\x1b[34mWHyTe-map planning is activated.");
      whytemap_client = std::make_shared<whytemap_ros::WHyTeMapClient>();
      ob::OptimizationObjectivePtr DTWOO(
          new ompl::mod::DTWOptimizationObjective(
              planner->ss->getSpaceInformation(), whytemap_client->get(),
              pp.weight_d, pp.weight_q, pp.weight_c, vp.max_vehicle_speed));
      std::dynamic_pointer_cast<ompl::mod::DTWOptimizationObjective>(DTWOO)->setTimeStamp(planning_time_);
      planner->ss->setOptimizationObjective(DTWOO);
    } else {
      ROS_WARN_STREAM("Wrong or missing 'mod_type' parameter. Not using MoDs "
                      "for planning.");
    }
  }

  virtual ~MoDReedsSheppRRTStarPlanner() = default;

  void savePathCallback(const std_msgs::String &msg) {
    ROS_INFO_STREAM("\x1b[34mSaving path to file: " << msg.data.c_str());
    std::ofstream ofile(msg.data.c_str(), std::ios::out);
    if (!ofile) {
      ROS_ERROR_STREAM("Couldn't open paths file!");
      return;
    }

    for (const auto &pose : poses_ptr->poses) {
      ofile << pose.position.x << ", " << pose.position.y << ", "
            << 2 * atan2(pose.orientation.z, pose.orientation.w) << std::endl;
    }

    ofile.close();
    ROS_INFO_STREAM("Successfully saved path to file: " << msg.data.c_str());
  }

  void solutionCallback(
      const ompl::base::Planner *planner,
      const std::vector<const ompl::base::State *> &solution_states,
      const ompl::base::Cost cost) {
    ROS_INFO_STREAM("New solution found with cost: \x1b[34m"
                    << std::fixed << std::setprecision(2) << cost.value());
  }
};

void modGoalCallback(const mod_path_planning::MoDPlanningGoalConstPtr& msg) {
  ROS_INFO("Adding a goal to the queue.");
  goal_queue_.push(msg);
}

int main(int argn, char *args[]) {
  ros::init(argn, args, "mod_path_planning_node");

  ros::NodeHandle nhandle;

  ros::Subscriber mod_planning_sub = nhandle.subscribe("/mod_planning_goal", 1, &modGoalCallback);

  while(ros::ok()) {
    ros::spinOnce();
    if(not goal_queue_.empty()) {
      const mod_path_planning::MoDPlanningGoalConstPtr goal = goal_queue_.front();
      goal_queue_.pop();

      // Start planning.

      // Save path.

      // Save statistics.
    }
  }

  return 0;
}
