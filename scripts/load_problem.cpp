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

using namespace robowflex;

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "load_problem");
    ros::NodeHandle node("~");

    std::string robot_file, scene_file_dir, var_file, request_file, planning_group;

    std::string exec_name = "load_problem";

    // Load ROS params
    size_t error = 0;
    // Parsing the parametes from the param server
    error += !parser::get(exec_name, node, "robot", robot_file);
    error += !parser::get(exec_name, node, "scene", scene_file_dir);
    error += !parser::get(exec_name, node, "planning_group", planning_group);
    parser::shutdownIfError(exec_name, error);

    auto robot = std::make_shared<Robot>("RoboCop");
    robot->initializeFromYAML(robot_file);
    auto rviz = std::make_shared<IO::RVIZHelper>(robot);

    auto scene = std::make_shared<Scene>(robot);

    std::vector<std::string> scene_file_vec;
    for (int i = 0; i < 34; ++i)
    {
        scene_file_vec.push_back(scene_file_dir + std::to_string(i) + ".yaml");
    }

    int count = 0;

    while (true)
    {
        count = (count + 1) % 34;
        scene->fromYAMLFile(scene_file_vec[count]);

        rviz->updateScene(scene);  // auto generate_file_name =
                                   // generateNewScene(getSceneFolder(file_name));
        // auto request = std::make_shared<MotionRequestBuilder>(robot);
        // request->fromYAMLFile(request_file);
        // request->setConfig("PRM");
        auto pg = std::make_shared<ProblemGenerator>(request_file);
        // pg->setParameters();
        // sampled_scene->toYAMLFile(generate_file_name);
        parser::waitForUser("Displaying the scene!");
    }
    return 0;
}