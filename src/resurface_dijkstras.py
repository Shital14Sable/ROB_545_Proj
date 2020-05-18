#!/usr/bin/env python

import rospy
import robot
import world
import random
import sys
import numpy as np
import copy

# base class... don't use directly
class Planner():
    def __init__(self, robot, world ):
        self.world = world
        self.robot = robot

    def plan(self):
        raise NotImplementedError()

class PlannerShortestPath(Planner):

    def set_parameters(self, vertex_start_idx, vertex_goal_idx):
        self.vertex_start_idx = vertex_start_idx
        self.vertex_goal_idx = vertex_goal_idx

        # if find_nearest_target returned None (doesn't think it knows where any targets are)
        if self.vertex_goal_idx == None: # Check with Graeme
            rospy.logerr("PlannerShortestPath() goal vertex is None") #maybe just print it instead            


    def plan(self, debug=False):
        #rospy.loginfo("PlannerShortestPath plan()")
        try: 
            self.vertex_start_idx
            self.vertex_goal_idx
        except AttributeError:
            rospy.logerr("set_parameters() for dijkstras needs to be called")
            return None

        # rospy.loginfo("Calling dijkstras")
        [distance, path] = self.dijkstras(debug)
        if path == []: # Check with Graeme
            return path 
        if len(path) > 1:
            path = path[1:]
            return path
        else:
            #rospy.logwarn("no path to goal, or goal has been reached")
            return None
        
    def dijkstras(self, debug=False):
        num_vertices = len(self.world.vertices)
        dist_to_go = [sys.maxint] * num_vertices
        prev = [-1] * num_vertices
        dist_to_go[self.vertex_start_idx] = 0

        open_set = [True] * num_vertices

        #iteration_count = 0 # for debugging

        while not self.is_open_set_empty(open_set):

            #iteration_count = iteration_count + 1
            #rospy.logwarn("dijkstra iteration_count: " + str(iteration_count))

            # find the vertex in open_set that has minimum dist_to_go
            v_current = self.find_min_vertex(dist_to_go, open_set)

            # remove it from the open set
            open_set[v_current] = False

            # get the set of neighbours
            neighbours = self.get_neighbours(v_current)

            # expand neighbouring nodes
            for e in neighbours:
                v_next = e.vertex_end_idx
                if open_set[v_next] == True:
                    alternative_distance = dist_to_go[v_current] + e.cost
                    if alternative_distance < dist_to_go[v_next]:
                        dist_to_go[v_next] = alternative_distance
                        prev[v_next] = v_current

        # backtrack to find path and distance
        path = []
        v = self.vertex_goal_idx
        # Check if goal vertex is None # Check with Graeme
        if v == None:
            return [None,[]]
        d = dist_to_go[v]
        if debug:
            print "dijkstra goal: " + str(self.vertex_goal_idx)
        if prev[v] >= 0 or v == self.vertex_start_idx:
            while v >= 0:
                path.insert(0, v)
                v = prev[v]
        if debug:
            print path
        return [d, path] 

    def is_open_set_empty(self, open_set):
        for i in open_set:
            if i == True:
                return False
        return True

    def get_neighbours(self, vertex_idx):
        '''
        edges_out = self.world.get_edges_out(vertex_idx)

        # filter out the non-existent edges
        edges_out_keep = []
        for e in edges_out:
            if e.exists:
                edges_out_keep.append(e)
        return edges_out_keep
        '''
        return self.world.edge_adjacency_edge_lists[vertex_idx]


    def find_min_vertex(self, dist_to_go, open_set):
        
        min_idx = -1
        min_value = sys.maxint

        for i in xrange(len(dist_to_go)):
            
            value = dist_to_go[i]
            if open_set[i] == True:
                if value <= min_value:

                    min_value = value
                    min_idx = i
        return min_idx

class PlannerResurface(PlannerShortestPath):

    def set_parameters(self, vertex_start_idx):
        self.vertex_start_idx = vertex_start_idx

    def plan(self, debug=False):
        #rospy.loginfo("PlannerShortestPath plan()")
        #debug = True
        try: 
            self.vertex_start_idx
        except AttributeError:
            rospy.logerr("set_parameters() for dijkstras needs to be called")
            return None

        # rospy.loginfo("Calling dijkstras")
        [distance, path] = self.dijkstras(debug)
        if len(path) > 1:
            path = path[1:]
            return path
        else:
            #rospy.logwarn("no path to goal, or goal has been reached")
            return None

    def dijkstras(self, debug=False):
        num_vertices = len(self.world.vertices)
        dist_to_go = [sys.maxint] * num_vertices
        prev = [-1] * num_vertices
        dist_to_go[self.vertex_start_idx] = 0

        if debug:
            print(self.vertex_start_idx)

        open_set = [True] * num_vertices

        #iteration_count = 0 # for debugging

        while not self.is_open_set_empty(open_set):

            #iteration_count = iteration_count + 1
            #rospy.logwarn("dijkstra iteration_count: " + str(iteration_count))

            # find the vertex in open_set that has minimum dist_to_go
            v_current = self.find_min_vertex(dist_to_go, open_set)

            # check if vertex is at surface, and if so that is the goal
            # i.e. the goal is the shortest path to any vertex in comms range (calc'd outside loop)
            if self.world.vertices[v_current].position.z == 0: # Check with Graeme DONE
                if debug:
                    print('vertex at surface',v_current,self.world.vertices[v_current].position)

                break

            # remove it from the open set
            open_set[v_current] = False

            # get the set of neighbours
            neighbours = self.get_neighbours(v_current)

            # expand neighbouring nodes
            for e in neighbours:
                v_next = e.vertex_end_idx
                if open_set[v_next] == True:
                    alternative_distance = dist_to_go[v_current] + e.cost
                    if alternative_distance < dist_to_go[v_next]:
                        dist_to_go[v_next] = alternative_distance
                        prev[v_next] = v_current

        # backtrack to find path and distance
        path = []
        v = v_current
        d = dist_to_go[v]
        if debug:
            print('distance to go',d)
        if debug:
            print "dijkstra goal: " + str(v_current)
            print(v,self.vertex_start_idx)
            print('prev',prev[v])
        if prev[v] >= 0 or v == self.vertex_start_idx:
            while v >= 0:
                path.insert(0, v)
                v = prev[v]
        if debug:
            print path
        return [d, path] 