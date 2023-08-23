#! /usr/bin/env python3

import operator
import random

import numpy
import math

from deap import base
from deap import benchmarks
from deap import creator
from deap import tools

import rospy
import sys
from std_msgs.msg import String
from motion_bench_maker.msg import *
from motion_bench_maker.srv import *

object_num = 5

creator.create("FitnessMin", base.Fitness, weights=(-1.0, ))
creator.create("Particle", list, fitness=creator.FitnessMin, speed=list, smin=None, smax=None, best=None)

# initial pose for each object in one particle
pos = numpy.array([[0.7, -0.058279, -0.358869], [0.5, 0.278224, -0.6166521], [0.9, -0.190976, 1.1623865], [0.7, 0.284192, 1.4096248], [0.9, -0.010244, -0.0733565]])

pop_num = 10
pop_pos = []  # total population pos of every particle

pub = rospy.Publisher('/final_scene', tableObjectPos, queue_size = 10)

def receiveObjectResult(particle):
    rospy.wait_for_service('/move_objects')
    object_pos_request = getTablePoseResultRequest()
    object_pos_request.seq = 1
    temp_object_pos = tableObjectPos()

    for i in range(object_num):
        temp_pose_point = tablePosePoint()
        temp_pose_point.x = particle[i][0]
        temp_pose_point.y = particle[i][1]
        temp_pose_point.theta = particle[i][2]
        temp_pose_point.z = 3
        temp_object_pos.object_pos.append(temp_pose_point)
    object_pos_request.pop_pos = temp_object_pos
    try:
        getObjectResult = rospy.ServiceProxy('move_objects', getTablePoseResult)
        resp = getObjectResult(object_pos_request)
        return resp
    except rospy.ServiceException as e:
        print("Service call failed: %s" %e)

def generate(obj_num, xy_min, xy_max, xy_smin, xy_smax):
    ## each particle has 7 object 
    part = creator.Particle(numpy.random.uniform(low = xy_min, high = xy_max, size = (obj_num, 3)))
    part.speed = numpy.random.uniform(low = xy_smin, high = xy_smax, size = (obj_num, 3))
    part.smin = xy_smin
    part.smax = xy_smax
    return part


def updateParticle(part, best, phi1, phi2):
    u1 = (random.uniform(0.1, phi1) for _ in range(len(part)))
    u2 = (random.uniform(0.3, phi2) for _ in range(len(part)))
    
    v_u1 = map(operator.mul, u1, map(operator.sub, part.best, part))
    v_u2 = map(operator.mul, u2, map(operator.sub, best, part))
    # x1 = v_u1
    # x2 = v_u2
    # print('v1', list(x1))
    # print('v2', list(x2))
    part.speed = list(map(operator.add ,part.speed, map(operator.add, v_u1, v_u2)))

    for i, speed in enumerate(part.speed):
        for j in range(len(speed)):
            if abs(speed[j]) < part.smin[j]:
                part.speed[i][j] = math.copysign(part.smin[j], speed[j])
            elif abs(speed[j]) > part.smax[j]:
                part.speed[i][j] = math.copysign(part.smax[j], speed[j])
            if j == 0:
                flag = True
                while flag:
                    if part[i][j] + part.speed[i][j] < 0.5 or part[i][j] + part.speed[i][j] > 1.3:
                        part.speed[i][j] = part.speed[i][j] / 2
                    else:
                        flag = False
            elif j == 1:
                flag = True
                while flag:
                    if part[i][j] + part.speed[i][j] < -0.35 or part[i][j] + part.speed[i][j] > 0.35:
                        part.speed[i][j] = part.speed[i][j] / 2
                    else:
                        flag = False
            elif j == 2:
                flag = True
                while flag:
                    if part[i][j] + part.speed[i][j] < -3.14 or part[i][j] + part.speed[i][j] > 3.14:
                        part.speed[i][j] = part.speed[i][j] / 2
                    else:
                        flag = False
    part[:] = list(map(operator.add, part, part.speed))


def obj_func(individual):
    feasible = receiveObjectResult(individual)
    cost = sum([100 * numpy.linalg.norm(x, 1) for x in map(operator.sub, individual, pos)])
    if not feasible.pop_result:
        cost = cost + 10000
    print("feasible is:", feasible.pop_result, "cost is:", round(cost, 3))
    return cost,


toolbox = base.Toolbox()
xy_min = [0.50, -0.35, -3.14]
xy_max = [1.3, 0.35, 3.14]
xy_smin = [-0.05, -0.05, -0.04]
xy_smax = [0.05, 0.05, 0.04]
obj_num = 5

toolbox.register("particle", generate, obj_num, xy_min = [0.50, -0.35, -3.14], xy_max=[1.3, 0.35, 3.14], xy_smin = [-0.05, -0.05, -0.04], xy_smax = [0.05, 0.05, 0.04])
toolbox.register("population", tools.initRepeat, list, toolbox.particle)
toolbox.register("update", updateParticle, phi1 = 0.3, phi2 = 0.6)
toolbox.register("evaluate", obj_func)


def main():
    rospy.init_node('pso_opt', anonymous=True)

    # print(receiveObjectResult())
    print("begin pso")
    pop = toolbox.population(n=150)
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", numpy.mean)
    stats.register("std", numpy.std)
    stats.register("min", numpy.min)
    stats.register("max", numpy.max)

    logbook = tools.Logbook()
    logbook.header = ["gen", "evals"] + stats.fields

    GEN = 100
    best = None

    for g in range(GEN):
        for part in pop:
            # print(toolbox.evaluate(part))
            part.fitness.values = toolbox.evaluate(part)
            if not part.best or part.best.fitness < part.fitness:
                part.best = creator.Particle(part)
                part.best.fitness.values = part.fitness.values
            if not best or best.fitness < part.fitness:
                best = creator.Particle(part)
                best.fitness.values = part.fitness.values
        # print("current gen:", g, " best value is: ", best.fitness.values, " and best is: ", best)
        for part in pop:
            toolbox.update(part, best)
        
        # final_scene_pos = objectPos()
        # print('send out best for visual')
        # for i in range(len(best)):
        #     temp_pose_point = posePoint()
        #     temp_pose_point.x = best[i][0]
        #     temp_pose_point.y = best[i][1]
        #     temp_pose_point.z = 1
        #     final_scene_pos.object_pos.append(temp_pose_point)
        # pub.publish(final_scene_pos)

        logbook.record(gen = g, evals=len(pop), **stats.compile(pop))
        print(logbook.stream, "best_fit:", round(best.fitness.values[0], 3))
        print(best)

    return pop, logbook, best

if __name__ == "__main__":
    res = main()
    best = res[2]
    print(best, best.fitness.values)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        final_scene_pos = tableObjectPos()
        for i in range(len(best)):
            temp_pose_point = tablePosePoint()
            temp_pose_point.x = best[i][0]
            temp_pose_point.y = best[i][1]
            temp_pose_point.z = 1
            temp_pose_point.theta = best[i][2]
            final_scene_pos.object_pos.append(temp_pose_point)
        pub.publish(final_scene_pos)
        # print("publish best pos already")
        rate.sleep()

