#!/usr/bin/env python
# _run.py
"""Node for following a planned trajectory of the car.
"""
######################
# Imports & Globals
######################

from autopsy.node import Node


######################
# RecordNode
######################

class RunNode(Node):

    def __init__(self):
        super(Node, self).__init__("follow_trajectory2")

