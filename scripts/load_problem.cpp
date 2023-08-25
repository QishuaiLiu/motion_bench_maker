// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/scene_sampler.h>

// Robowflex library
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/builder.h>
#include <motion_bench_maker/problem_generator.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "load_problem");
    ros::NodeHandle node("~");

    std::string robot_file, scene_file_dir, var_file, request_file, planning_group, planner_name, ompl_config;

    std::string exec_name = "load_problem";

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

    auto scene = std::make_shared<Scene>(robot);
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
    // chomp
    // auto planner = std::make_shared<opt::CHOMPPipelinePlanner>(robot, "planner");
    // planner->initialize(ompl_config);
    // ompl
    auto planner = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(robot, "planner");
    planner->initialize(ompl_config, settings);

    // auto planner = setup->createPlanner("planner", settings);

    // problem generator
    auto pg = std::make_shared<ProblemGenerator>(request_file_name);
    pg->setParameters(robot, planning_group, ee_offset[0]);

    std::vector<std::string> scene_file_vec;
    int total_scene_num = 90;
    for (int i = 0; i < total_scene_num; ++i)
    {
        scene_file_vec.push_back(scene_file_dir + std::to_string(i) + ".yaml");
    }
    std::vector<bool> planning_result(total_scene_num);

    int count = 0;

    while (true && ros::ok())
    {
        // count = (count + 1) % total_scene_num;
        if (count >= total_scene_num)
            break;
        ROS_INFO("current count is: %i\n", count);
        scene->fromYAMLFile(scene_file_vec[10]);

        rviz->updateScene(scene);  // auto generate_file_name =
                                   // generateNewScene(getSceneFolder(file_name));
                                   // auto request = std::make_shared<MotionRequestBuilder>(robot);
                                   // request->fromYAMLFile(request_file);
                                   // request->setConfig("PRM");
        pg->updateScene(scene);
        auto result = pg->createRandomRequest();
        if (!result.second)
        {
            ROS_INFO("create request failed.. for count is: %d", count);
            planning_result[count] = false;
            count++;
            continue;
        }
        // Try to solve with a planner to verify feasibility
        const auto &request = result.first;
        // Add the planner so we know that the correct config exists
        request->setPlanner(planner);
        request->setAllowedPlanningTime(60);
        request->setNumPlanningAttempts(2);
        if (!request->setConfig(planner_name))
            ROS_ERROR("Did not find planner %s", planner_name.c_str());
        const auto &res = planner->plan(scene, request->getRequestConst());

        if (res.error_code_.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
        {
            auto trajectory = std::make_shared<robowflex::Trajectory>(*res.trajectory_);
            rviz->updateScene(scene);
            // Visualize start state.
            rviz->visualizeState(request->getStartConfiguration());
            parser::waitForUser("Displaying initial state!");

            // Visualize goal state.
            rviz->visualizeState(request->getGoalConfiguration());
            parser::waitForUser("Displaying goal state!");

            // Visualize the trajectory.
            rviz->updateTrajectory(trajectory->getTrajectory());
            parser::waitForUser("Displaying The trajectory!");
            planning_result[count] = true;
            ROS_INFO("process scene number: %d", count);
        }
        count++;
        // sampled_scene->toYAMLFile(generate_file_name);
        parser::waitForUser("Displaying the scene!");
    }
    for (int i = 0; i < total_scene_num; ++i)
    {
        std::cout << "index is: " << i << " planning result is:" << planning_result[i] << std::endl;
    }
    return 0;
}