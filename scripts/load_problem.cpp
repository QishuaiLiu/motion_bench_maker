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

    std::string robot_file, scene_file_dir, var_file, request_file, planning_group, ompl_config;

    std::string exec_name = "load_problem";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "robot", robot_file);
    error += !parser::get(exec_name, node, "scene", scene_file_dir);
    error += !parser::get(exec_name, node, "planning_group", planning_group);
    error += !parser::get(exec_name, node, "request_file", request_file);
    error += !parser::get(exec_name, node, "ompl_config", ompl_config);
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
    auto planner = std::make_shared<robowflex::OMPL::OMPLInterfacePlanner>(robot, "planner");
    planner->initialize(ompl_config, settings);

    // auto planner = setup->createPlanner("planner", settings);

    // problem generator
    auto pg = std::make_shared<ProblemGenerator>(request_file_name);
    pg->setParameters(robot, planning_group, ee_offset[0]);

    std::vector<std::string> scene_file_vec;
    int total_scene_num = 24;
    for (int i = 0; i < total_scene_num; ++i)
    {
        scene_file_vec.push_back(scene_file_dir + std::to_string(i) + ".yaml");
    }

    int count = 0;

    while (true)
    {
        count = (count + 1) % total_scene_num;
        scene->fromYAMLFile(scene_file_vec[count]);

        rviz->updateScene(scene);  // auto generate_file_name =
                                   // generateNewScene(getSceneFolder(file_name));
        // auto request = std::make_shared<MotionRequestBuilder>(robot);
        // request->fromYAMLFile(request_file);
        // request->setConfig("PRM");
        pg->updateScene(scene);
        auto result = pg->createRandomRequest();
        // std::cout << "result: " << result.second << std::endl;

        // sampled_scene->toYAMLFile(generate_file_name);
        parser::waitForUser("Displaying the scene!");
    }
    return 0;
}