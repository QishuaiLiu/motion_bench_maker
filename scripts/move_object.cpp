// ROS
#include <moveit_msgs/MoveItErrorCodes.h>
#include <ros/duration.h>
#include <ros/ros.h>

// Robowflex dataset
#include <motion_bench_maker/parser.h>
#include <motion_bench_maker/scene_sampler.h>
// #include <motion_bench_maker/objectPos.h>

// Robowflex library
#include <robowflex_library/io.h>
#include <robowflex_library/io/visualization.h>
#include <robowflex_library/scene.h>
#include <robowflex_library/trajectory.h>
#include <robowflex_library/robot.h>
#include <robowflex_library/builder.h>
#include <motion_bench_maker/problem_generator.h>

#include <motion_bench_maker/getTablePoseResult.h>

// Robowflex ompl
#include <robowflex_ompl/ompl_interface.h>

using namespace robowflex;
const std::vector<std::string> obj_name{"Object1", "Object2", "Object3", "Object4", "Object5"};
std::shared_ptr<ProblemGenerator> pg;
std::shared_ptr<Scene> scene;
std::shared_ptr<IO::RVIZHelper> rviz;
namespace
{
    std::vector<Eigen::Vector2d> getObjectPose(std::shared_ptr<Scene> scene)
    {
        std::vector<Eigen::Vector2d> pose;
        std::map<std::string, Eigen::Vector2d> pose_map;
        auto obj = scene->getCollisionObjects();

        for (int i = 0; i < obj_name.size(); ++i)
        {
            // auto test_pose = scene->getObjectPose(obj_name[i]);
            // get the origin coord Eigen::Isometry3d temp_pose = Eigen::Isometry3d::Identity();
            // temp_pose.translation() = Eigen::Vector3d(1.35, 0, 0.85);
            // get the shape coord Eigen::Matrix4d test_again =
            //     test_pose.matrix() * temp_pose.matrix().inverse();
            // get the quaternion for the shape coord
            // Eigen::Quaterniond q(test_again.block<3, 3>(0, 0));
            // std::cout << "quaternion is: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
            //           << std::endl;

            auto obj_pose = scene->getObjectPose(obj_name[i]).translation().head<2>();
            pose.emplace_back(obj_pose);
        }

        return pose;
    }

    void setObjectPose(const std::vector<Eigen::Vector3d> &pose, std::shared_ptr<Scene> scene)
    {
        for (int i = 0; i < obj_name.size(); ++i)
        {
            if (i == 3)
            {
                std::cout << "obj name is: " << obj_name[i] << std::endl;
                auto test_pose = scene->getObjectPose(obj_name[i]);
                test_pose.translation().head<2>() = pose[i].head<2>();
                Eigen::Vector3d angles = Eigen::Vector3d(0, 0, pose[i][2]);
                Eigen::Quaterniond rot_matrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitX())    //
                                                * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())  //
                                                * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());
                test_pose.linear() = rot_matrix.toRotationMatrix();

                // get the origin coord
                Eigen::Isometry3d temp_pose = Eigen::Isometry3d::Identity();
                temp_pose.translation() = Eigen::Vector3d(0.65, -0.2, 0.9);
                // get the shape coord
                Eigen::Matrix4d test_again = test_pose.matrix() * temp_pose.matrix().inverse();
                // get the quaternion for the shape coord
                Eigen::Quaterniond q(test_again.block<3, 3>(0, 0));
                std::cout << "quaternion is: " << q.x() << " " << q.y() << " " << q.z() << " " << q.w()
                          << std::endl;
                std::cout << "tran is: \n" << test_again.col(3) << std::endl;
            }
            auto cur_obj_pose = scene->getObjectPose(obj_name[i]);
            auto temp_pose = cur_obj_pose;
            temp_pose.translation().head<2>() = pose[i].head<2>();
            Eigen::Vector3d angles = Eigen::Vector3d(0, 0, pose[i][2]);
            Eigen::Quaterniond rot_matrix = Eigen::AngleAxisd(angles[0], Eigen::Vector3d::UnitX())    //
                                            * Eigen::AngleAxisd(angles[1], Eigen::Vector3d::UnitY())  //
                                            * Eigen::AngleAxisd(angles[2], Eigen::Vector3d::UnitZ());

            temp_pose.linear() = rot_matrix.toRotationMatrix();
            auto new_obj_pose = temp_pose * cur_obj_pose.inverse();

            scene->moveObjectGlobal(obj_name[i], new_obj_pose);
        }
    }
}  // namespace

bool get_final_scene = false;
std::vector<Eigen::Vector3d> final_scene_pos;

bool getPlanningResult(std::shared_ptr<ProblemGenerator> &pg, const std::vector<Eigen::Vector3d> &pose,
                       std::shared_ptr<Scene> &scene)
{
    setObjectPose(pose, scene);
    pg->updateScene(scene);
    // rviz->updateScene(scene);
    auto result = pg->createRandomRequest();
    return result.second;
}

bool getPopResult(motion_bench_maker::getTablePoseResultRequest &req,
                  motion_bench_maker::getTablePoseResultResponse &res)
{
    if (pg == nullptr || scene == nullptr)
        return false;
    std::vector<Eigen::Vector2d> particle_pose;
    std::vector<Eigen::Vector3d> object_pos;  // single particle

    object_pos.clear();
    auto temp_scene = scene->deepCopy();
    for (int j = 0; j < req.pop_pos.object_pos.size(); ++j)  // for each object in particle
    {
        auto &received_pos = req.pop_pos.object_pos[j];
        Eigen::Vector3d pos = Eigen::Vector3d(received_pos.x, received_pos.y, received_pos.theta);
        object_pos.emplace_back(pos);
    }
    bool ret = getPlanningResult(pg, object_pos, temp_scene);
    res.pop_result = ret;

    res.seq = req.seq;

    return true;
}

void getFinalScene(const motion_bench_maker::tableObjectPos::ConstPtr &msg)
{
    auto cur_time = ros::Time::now();
    ROS_INFO("----------------------------------------final time is: %i", cur_time);
    ROS_INFO("get object pos");
    if (msg == nullptr)
    {
        get_final_scene = false;
        return;
    }

    int size = msg->object_pos.size();
    final_scene_pos.reserve(size);
    for (int i = 0; i < size; ++i)
    {
        Eigen::Vector3d pos;
        pos.x() = msg->object_pos[i].x;
        pos.y() = msg->object_pos[i].y;
        pos.z() = msg->object_pos[i].theta;
        final_scene_pos.emplace_back(pos);
    }
    get_final_scene = true;
    rviz->updateScene(scene);
    parser::waitForUser("Displaying original state!");
    auto temp_scene = scene->deepCopy();
    setObjectPose(final_scene_pos, temp_scene);
    pg->updateScene(temp_scene);
    int ret = getPlanningResult(pg, final_scene_pos, temp_scene);
    ROS_INFO("........................ret result is: %d", (int)ret);
    rviz->updateScene(temp_scene);
    parser::waitForUser("Displaying optimization state!");
}

int main(int argc, char **argv)
{
    // Enables multiple instances running with the same name
    ros::init(argc, argv, "move_object");
    ros::NodeHandle node("~");

    std::string robot_file, scene_file_dir, var_file, request_file, planning_group, planner_name, ompl_config;

    std::string exec_name = "move_object";

    ros::ServiceServer service = node.advertiseService("/move_objects", getPopResult);

    ros::Subscriber final_scene_sub = node.subscribe("/final_scene", 100, getFinalScene);

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
    rviz = std::make_shared<IO::RVIZHelper>(robot);

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

    scene->fromYAMLFile(scene_file_vec[10]);

    auto cur_time = ros::Time::now();
    ROS_INFO("-------------------------cur time is: %i", cur_time);
    ros::spin();
    // while (true)
    // {
    //     rviz->updateScene(scene);
    //     parser::waitForUser("display the scene");
    //     std::vector<Eigen::Vector2d> object_pos(7);
    //     object_pos[0] << 0.69915537, -0.08294566;
    //     object_pos[1] << 0.49807742, 0.27463609;
    //     object_pos[2] << 0.90291438, -0.18692278;
    //     object_pos[3] << 0.71228151, 0.28583853;
    //     object_pos[4] << 0.908616, -0.0221888;
    //     object_pos[5] << 0.70149273, 0.37695294;
    //     object_pos[6] << 0.49009288, -0.23345044;
    //     auto temp_scene = scene->deepCopy();
    //     auto ret = getPlanningResult(pg, object_pos, temp_scene);
    //     rviz->updateScene(temp_scene);

    //     parser::waitForUser("final scene");
    // }

    // while (true && ros::ok())
    // {
    //     rviz->updateScene(scene);  // auto generate_file_name =
    //     // generateNewScene(getSceneFolder(file_name));
    //     parser::waitForUser("Displaying the scene!");
    //     auto pose = getObjectPose(scene);
    //     std::vector<Eigen::Vector3d> new_pose;
    //     new_pose.resize(pose.size());
    //     for (int i = 0; i < pose.size(); ++i)
    //     {
    //         std::cout << "pose x: " << pose[i][0] << " y is: " << pose[i][1] << std::endl;
    //         // pose[i] += Eigen::Vector2d::Ones() * 0.02;
    //         Eigen::Vector3d temp;
    //         temp << 0.1, 0.1, 0.01;
    //         // new_pose[i] = Eigen::Vector3d(pose[i][0], pose[i][1], 0) + temp;
    //         new_pose[i] = Eigen::Vector3d(pose[i][0], pose[i][1], 0);
    //     }
    //     auto temp_scene = scene->deepCopy();
    //     ////////////////////
    //     new_pose[3] = Eigen::Vector3d(0.54953212, -0.14932797, 1.95203857);
    //     ////////////////////
    //     setObjectPose(new_pose, temp_scene);
    //     rviz->updateScene(temp_scene);
    //     parser::waitForUser("displaying the temp scene.");

    //     auto obj = scene->getCollisionObjects();
    //     std::vector<std::string> obj_name_new{"Can1", "Can2", "Can3", "Can4", "Can5",
    //                                           "Can6", "Can7", "Can8", "Can9"};
    //     for (int i = 0; i < obj_name_new.size(); ++i)
    //     {
    //         auto obj_pose = scene->getObjectPose(obj_name_new[i]).translation().head<2>();
    //         // auto obj_pose = scene->getFramePose(obj_name_new[i]).translation().head<2>();
    //         ROS_INFO("after pose x: %.3f, y is: %.3f", obj_pose[0], obj_pose[1]);
    //     }

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
