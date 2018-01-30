#!/usr/bin/env python3
# -*- coding: utf-8 -*-

def score(cost, solution):
    return sum(cost[c-1] for c, _, _ in solution)
