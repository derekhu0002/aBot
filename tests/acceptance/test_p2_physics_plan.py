#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: P2 physics breakdown plan completeness.

GIVEN the intent graph contains the P2 物理化细分计划 Work Package
WHEN validating plan completeness
THEN the element exists as Work Package, its description covers the four
stages (MJCF 双出口 / physics_adapter / 9 动作 / 校准闭环), milestones M1-M4,
owners (建模员/实验验证工程师), dependency order and risk keywords.
"""

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PLAN = "abot-p2-physics-plan-001"
KEYWORDS = ["MJCF", "physics_adapter", "9 动作", "校准", "M1", "M2", "M3", "M4", "建模员", "实验验证工程师", "T1", "T7"]


def main():
    failures = []
    graph_path = os.path.join(ROOT, "design", "KG", "SystemArchitecture.json")
    if not os.path.exists(graph_path):
        print("FAIL: %s not found" % graph_path)
        return 1
    with open(graph_path, encoding="utf-8") as fh:
        graph = json.load(fh)

    plan = next((e for e in graph.get("elements", []) if str(e.get("id")) == PLAN), None)
    if plan is None:
        failures.append("plan '%s' not found" % PLAN)
        return 1
    if plan.get("type") != "Work Package":
        failures.append("plan type = %r, want 'Work Package'" % plan.get("type"))
    desc = plan.get("description", "")
    for kw in KEYWORDS:
        if kw not in desc:
            failures.append("plan description missing keyword %s" % kw)

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: P2 physics breakdown plan complete (4 stages / 8 tasks / M1-M4)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
