#!/usr/bin/env python
# coding=UTF-8
import math
import numpy as np
import matplotlib.pyplot as plt

import rospy

show_animation = False


class AStarPlanner:

    def __init__(self, reso, rr, w, h):
        self.reso = reso
        self.rr = rr
        self.minx, self.miny = -10, -10
        self.width, self.height = w, h
        self.motion = self.get_motion_model()
        self.delta = 1

    class Node:
        def __init__(self, x, y, g, f, pind):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.g = g
            self.f = f
            self.pind = pind

        def __str__(self):
            return str(self.x) + "," + str(self.y) + "," + str(
                self.g) + "," + str(self.f)

    def planning(self, sx, sy, gx, gy, map):
        self.map = map
        nstart = self.Node(self.calc_xyindex(sx, self.minx),
                           self.calc_xyindex(sy, self.miny), 0.0, 0.0, -1)
        ngoal = self.Node(self.calc_xyindex(gx, self.minx),
                          self.calc_xyindex(gy, self.miny), 0.0, 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(nstart)] = nstart

        while 1:
            min_f= float("inf")
            for i in open_set.keys():
                if open_set[i].f < min_f:
                   min_f = open_set[i].f
                   record = i

            # no available path
            if not len(open_set):
                print("----------NO PATH----------")
                break

            temp = open_set.pop(record)
            closed_set[record] = temp

            if temp.x == ngoal.x and temp.y == ngoal.y:
                ngoal = temp
                break

            # check adjacent vertex
            for item in self.motion:
                newnode = self.Node(temp.x+item[0], temp.y+item[1], temp.g+item[2], 0.0, self.calc_grid_index(temp)) 
                newnode.f = newnode.g + self.calc_heuristic(newnode, ngoal)
                index = self.calc_grid_index(newnode)

                if newnode.x<0 or newnode.x>self.height or newnode.y<0 or newnode.y>self.width:
                    continue

                if not self.verify_node(newnode) or index in closed_set:
                    continue
                
                if not index in open_set:
                    open_set[index] = newnode
                elif open_set[index].g > newnode.g:
                    open_set[index] = newnode

        rx, ry = self.calc_final_path(ngoal, closed_set)

        return rx, ry

    def calc_final_path(self, ngoal, closedset):
        # generate final path
        rx, ry = [self.calc_grid_position(ngoal.x, self.minx)], [
            self.calc_grid_position(ngoal.y, self.miny)]
        pind = ngoal.pind
        while pind != -1:
            n = closedset[pind]
            rx.append(self.calc_grid_position(n.x, self.minx))
            ry.append(self.calc_grid_position(n.y, self.miny))
            pind = n.pind

        rx,ry = self.smooth(rx,ry)
        return rx, ry

    def smooth(self, x, y):
         for i in range(len(x)-1,1,-2):
            midx = (x[i]+x[i-2]) / 2
            midy = (y[i]+y[i-2]) / 2
            flag = 1
            if(self.map[self.calc_xyindex(midx, self.minx),self.calc_xyindex(midy, self.miny)]<50):
                x[i-1] = midx
                y[i-1] = midy
         return  x, y

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x-n2.x, n1.y-n2.y)
        return d

    def calc_grid_position(self, index, minp):
        pos = index*self.reso + minp
        return pos

    def calc_xyindex(self, position, min_pos):
        return int(round((position-min_pos) / self.reso))

    def calc_grid_index(self, node):
        return int(node.y*self.width + node.x)

    def verify_node(self, node):
        xl, xr = max(node.x-self.delta, 0), min(node.x+self.delta, self.height)
        yl, yr = max(node.y-self.delta, 0), min(node.y+self.delta, self.width)
        return np.sum(np.array(self.map[xl:xr+1, yl:yr+1]>50)) == 0

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

        return motion
