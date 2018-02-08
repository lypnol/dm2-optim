#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from math import sqrt
import sys
import numpy as np
from scipy.spatial import Delaunay
from ortools.linear_solver import pywraplp
from collections import defaultdict
from operator import itemgetter
from wrappers import timeit


def dist(a, b):
    return sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def circumcircle(a, b, c):
    (x1, y1), (x2, y2), (x3, y3) = a, b, c
    A = np.array([[x3 - x1, y3 - y1], [x3 - x2, y3 - y2]])
    Y = np.array([(x3 ** 2 + y3 ** 2 - x1 ** 2 - y1 ** 2), (x3 ** 2 + y3 ** 2 - x2 ** 2 - y2 ** 2)])
    if np.linalg.det(A) == 0:
        return False
    Ainv = np.linalg.inv(A)
    X = 0.5 * np.dot(Ainv, Y)
    x, y = X[0], X[1]
    r = sqrt((x - x1) ** 2 + (y - y1) ** 2)
    return (x, y), r


def middle(a, b):
    return (a[0] + b[0]) / 2, (a[1] + b[1]) / 2


def neighbors(u, closest, depth=4):
    if depth == 0:
        return set()
    res = set()
    for v in closest[u]:
        res |= {v} | neighbors(v, closest, depth - 1)
    return res


@timeit
def solve(radius, cost, points):
    # Parameters
    N = len(points)

    m_radius = max(radius)

    # Solver
    solver = pywraplp.Solver('cameras', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
    var = {}

    # Delaunay triangulation
    triangulation = Delaunay(points)

    # Construct variables
    connected = defaultdict(set)
    possible = set()
    visited = set()
    for triplet in triangulation.simplices:
        i, j, k = np.sort(triplet)

        center, r = circumcircle(points[i], points[j], points[k])
        if r > m_radius:
            continue
        if (i, j, k) not in visited:
            for camera_type, rad in enumerate(radius, start=1):
                if r <= rad:
                    connected[i].add((center, camera_type))
                    connected[j].add((center, camera_type))
                    connected[k].add((center, camera_type))
                    possible.add((center, camera_type))
            visited.add((i, j, k))

        for u, v in ((i, j), (i, k), (j, k)):
            if (u, v) in visited:
                continue
            d = dist(points[u], points[v])
            center = middle(points[u], points[v])
            if d > m_radius:
                continue
            for camera_type, rad in enumerate(radius, start=1):
                if rad <= d:
                    connected[u].add((center, camera_type))
                    connected[v].add((center, camera_type))
                    possible.add((center, camera_type))
            visited.add((u, v))

    for u, (x, y) in enumerate(points):
        camera_type = 1
        center = x + 1, y + 1
        possible.add((center, camera_type))
        connected[u].add((center, camera_type))

    def is_in_camera_square(center, r, point):
        return -r <= center[0] - point[0] <= r and -r <= center[1] - point[1] <= r

    for i, (center, camera_type) in enumerate(possible):
        for u, (x, y) in enumerate(points):
            if is_in_camera_square(center, r, (x, y)) and dist(center, (x, y)) <= r:
                connected[u].add((center, camera_type))
        print(f"\r processed {100*(i+1)/len(possible):8.2f}%", file=sys.stderr, end='')
    print(file=sys.stderr)

    for center, camera_type in possible:
        var[(center, camera_type)] = solver.IntVar(0, 1, f'camera_{camera_type}_{center}')
    print(len(var), "variables", file=sys.stderr)

    # Construct constraints
    constraints = 0
    for u in range(N):
        possible_cameras = []
        for center, camera_type in connected[u]:
            possible_cameras.append(var[(center, camera_type)])
        solver.Add(solver.Sum(possible_cameras) >= 1)
        constraints += 1

    keys = sorted(var, key=itemgetter(0))
    obj_expr = solver.Sum([cost[key[1] - 1] * var[key] for key in keys])
    solver.Minimize(obj_expr)
    solver.Solve()

    solution = set()
    for (center, camera_type), value in var.items():
        if value.SolutionValue() == 1:
            solution.add((camera_type, round(center[0], 2), round(center[1], 2)))

    return solution
