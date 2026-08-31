#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: aBot kids-learning goal <-> child exploration guide.

GIVEN the intent graph contains the aBot 亲子共学 Goal and the repo contains
the child motion-exploration guide
WHEN validating the learning goal artifact
THEN the Goal element exists with type 'Goal', its file_paths include
docs/learning/robot-motion-explore.md, that file exists, and the file lists
the 9 robot actions and the 'how to make the robot move' steps.
"""

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

GOAL_ID = "abot-kids-learning-001"
GUIDE = "docs/learning/robot-motion-explore.md"
ACTIONS = ["relax", "tpose", "apose", "idle", "wave", "walk", "nod", "look", "run"]


def main():
    failures = []

    graph_path = os.path.join(ROOT, "design", "KG", "SystemArchitecture.json")
    if not os.path.exists(graph_path):
        print("FAIL: %s not found" % graph_path)
        return 1
    with open(graph_path, encoding="utf-8") as fh:
        graph = json.load(fh)

    goal = next(
        (e for e in graph.get("elements", []) if str(e.get("id")) == GOAL_ID), None
    )
    if goal is None:
        failures.append("goal '%s' not found in intent graph" % GOAL_ID)
    else:
        if goal.get("type") != "Goal":
            failures.append("goal type = %r, want 'Goal'" % goal.get("type"))
        attrs = {}
        for attr in goal.get("attributes", []) or []:
            if isinstance(attr, dict):
                attrs[attr.get("name")] = attr.get("value")
        if GUIDE not in (attrs.get("file_paths") or ""):
            failures.append("goal file_paths missing %s" % GUIDE)

    guide_path = os.path.join(ROOT, *GUIDE.split("/"))
    if not os.path.exists(guide_path):
        failures.append("guide file not found: %s" % GUIDE)
    else:
        with open(guide_path, encoding="utf-8") as fh:
            content = fh.read()
        for action in ACTIONS:
            if ("`%s`" % action) not in content and action not in content:
                failures.append("guide missing action %s" % action)
        for kw in ["播放", "空格", "Action Editor", "骨骼", "关节"]:
            if kw not in content:
                failures.append("guide missing keyword %s" % kw)

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: aBot kids-learning goal & motion-exploration guide consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
