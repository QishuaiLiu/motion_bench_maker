// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/scene_sampler.h>
#include <motion_bench_maker/objectPos.h>

// Robowflex library
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/builder.h>
#include <motion_bench_maker/problem_generator.h>

#include <motion_bench_maker/getPoseResult.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;
const std::vector<std::string> obj_name{"Can2", "Can3", "Can4", "Can5", "Can7", "Can8", "Can9"};
std::shared_ptr<ProblemGenerator> pg;
std::shared_ptr<Scene> scene;
namespace
{
    std::vector<Eigen::Vector2d> getObjectPose(std::shared_ptr<Scene> scene)
    {
        std::vector<Eigen::Vector2d> pose;
        std::map<std::string, Eigen::Vector2d> pose_map;
        auto obj = scene->getCollisionObjects();

        for (int i = 0; i < obj_name.size(); ++i)
        {
            auto obj_pose = scene->getObjectPose(obj_name[i]).translation().head<2>();
            pose.emplace_back(obj_pose);
        }

        return pose;
    }

    void setObjectPose(const std::vector<Eigen::Vector2d> &pose, std::shared_ptr<Scene> scene)
    {
        for (int i = 0; i < obj_name.size(); ++i)
        {
            auto cur_obj_pose = scene->getObjectPose(obj_name[i]);
            auto temp_pose = cur_obj_pose;
            temp_pose.translation().head<2>() = pose[i];
            auto new_obj_pose = temp_pose * cur_obj_pose.inverse();

            scene->moveObjectGlobal(obj_name[i], new_obj_pose);
        }
    }
}  // namespace

bool getPlanningResult(std::shared_ptr<ProblemGenerator> &pg, const std::vector<Eigen::Vector2d> &pose,
                       std::shared_ptr<Scene> &scene)
{
    setObjectPose(pose, scene);
    pg->updateScene(scene);
    auto result = pg->createRandomRequest();
    return result.second;
}

bool getPopResult(motion_bench_maker::getPoseResultRequest &req,
                  motion_bench_maker::getPoseResultResponse &res)
{
    if (pg == nullptr || scene == nullptr)
        return false;
    std::vector<Eigen::Vector2d> particle_pose;
    std::vector<Eigen::Vector2d> object_pos;  // single particle

    object_pos.clear();
    auto temp_scene = scene->deepCopy();
    for (int j = 0; j < req.pop_pos.object_pos.size(); ++j)  // for each object in particle
    {
        auto &received_pos = req.pop_pos.object_pos[j];
        Eigen::Vector2d pos = Eigen::Vector2d(received_pos.x, received_pos.y);
        object_pos.emplace_back(pos);
    }
    bool ret = getPlanningResult(pg, object_pos, temp_scene);
    res.pop_result = ret;

    res.seq = req.seq;

    return true;
}

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "move_object");
    ros::NodeHandle node("~");

    std::string robot_file, scene_file_dir, var_file, request_file, planning_group, planner_name, ompl_config;

    std::string exec_name = "move_object";

    ros::ServiceServer service = node.advertiseService("/move_objects", getPopResult);

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "robot", robot_file);
    error += !parser::get(exec_name, node, "scene", scene_file_dir);
    error += !parser::get(exec_name, node, "planning_group", planning_group);
    error += !parser::get(exec_name, node, "request_file", request_file);
    error += !parser::get(exec_name, node, "ompl_config", ompl_config);
    error += !parser::get(exec_name, node, "planner_name", planner_name);
    parser::shutdownIfError(exec_name, error);

    auto robot = std::make_shared<Robot>("RoboCop");
    robot->initializeFromYAML(robot_file);
    robot->loadKinematics(planning_group);
    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    scene = std::make_shared<Scene>(robot);
    auto start_state = std::make_shared<robot_state::RobotState>(*robot->getScratchStateConst());

    const std::string request_file_name = IO::resolvePath(request_file);

    geometry_msgs::Pose ee_pos;
    ee_pos.position.x = 0;
    ee_pos.position.y = 0;
    ee_pos.position.z = 0;
    ee_pos.orientation.x = 0;
    ee_pos.orientation.y = 0;
    ee_pos.orientation.z = 0;
    ee_pos.orientation.w = 1;

    RobotPoseVector ee_offset = {TF::poseMsgToEigen(ee_pos)};

    // planner setting
    auto settings = OMPL::Settings();
    settings.hybridize_solutions = false;
    settings.interpolate_solutions = false;
    auto planner = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(robot, "planner");
    planner->initialize(ompl_config, settings);

    // auto planner = setup->createPlanner("planner", settings);

    // problem generator
    pg = std::make_shared<ProblemGenerator>(request_file_name);
    pg->setParameters(robot, planning_group, ee_offset[0]);

    std::vector<std::string> scene_file_vec;
    int total_scene_num = 24;
    for (int i = 0; i < total_scene_num; ++i)
    {
        scene_file_vec.push_back(scene_file_dir + std::to_string(i) + ".yaml");
    }
    std::vector<bool> planning_result(total_scene_num);

    scene->fromYAMLFile(scene_file_vec[4]);

    ros::spin();

    // while (true)
    // {
    //     rviz->updateScene(scene);  // auto generate_file_name =
    //     // generateNewScene(getSceneFolder(file_name));
    //     parser::waitForUser("Displaying the scene!");
    //     auto pose = getObjectPose(scene);
    //     for (int i = 0; i < pose.size(); ++i)
    //     {
    //         pose[i] += Eigen::Vector2d::Ones() * 0.02;
    //     }
    //     setObjectPose(pose, scene);

    // auto request = std::make_shared<MotionRequestBuilder>(robot);
    // request->fromYAMLFile(request_file);
    // request->setConfig("PRM");
    //     pg->updateScene(scene);
    //     auto result = pg->createRandomRequest();
    //     if (!result.second)
    //     {
    //         ROS_INFO("create request failed..");
    //         planning_result[count - 1] = false;
    //         continue;
    //     }
    //     // Try to solve with a planner to verify feasibility
    //     const auto &request = result.first;
    //     // Add the planner so we know that the correct config exists
    //     request->setPlanner(planner);
    //     request->setAllowedPlanningTime(60);
    //     request->setNumPlanningAttempts(2);
    //     if (!request->setConfig(planner_name))
    //         ROS_ERROR("Did not find planner %s", planner_name.c_str());
    //     const auto &res = planner->plan(scene, request->getRequestConst());

    //     if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    //     {
    //         auto trajectory = std::make_shared<robowflex::Trajectory>(*res.trajectory_);
    //         rviz->updateScene(scene);
    //         // Visualize start state.
    //         rviz->visualizeState(request->getStartConfiguration());
    //         parser::waitForUser("Displaying initial state!");

    //         // Visualize goal state.
    //         rviz->visualizeState(request->getGoalConfiguration());
    //         parser::waitForUser("Displaying goal state!");

    //         // Visualize the trajectory.
    //         rviz->updateTrajectory(trajectory->getTrajectory());
    //         parser::waitForUser("Displaying The trajectory!");
    //         planning_result[count - 1] = true;
    //         ROS_INFO("process scene number: %d", count - 1);
    //     }
    //     // sampled_scene->toYAMLFile(generate_file_name);
    //     parser::waitForUser("Displaying the scene!");
    // }
    return 0;
}