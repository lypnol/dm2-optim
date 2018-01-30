#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import argparse
from time import time
from solve import solve
from score import score


def main():
    parser = argparse.ArgumentParser(description="Solves camera placing problem")
    parser.add_argument("input", help="Path to problem input file", type=str)
    args = parser.parse_args()

    radius = []
    cost = []
    points = []

    with open(args.input) as f:
        for i, line in enumerate(f):
            if i == 0:
                radius = list(map(lambda x: int(x) - 0.01, line.split(',')))
            elif i == 1:
                cost = list(map(int, line.split(',')))
            else:
                x, y = list(map(int, line.split(',')))
                points.append((x, y))
    
    start = time()
    solution = solve(radius, cost, points)
    end = time()
    print("solution score:", score(cost, solution), file=sys.stderr)
    print("time:", "{:.2f}s".format(end-start), file=sys.stderr)
    for c, x, y in solution:
        print("{:d},{:.2f},{:.2f}".format(c, x, y))
    
if __name__ == "__main__":
    main()