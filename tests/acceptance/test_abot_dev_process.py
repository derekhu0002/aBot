#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: aBot 3-stage dev process (digital twin -> MCP -> real robot).

GIVEN the intent graph contains the aBot 开发过程 Business Process
WHEN validating the three-stage development process
THEN the element exists as Business Process, its description contains the
three stages (数字孪生 / MCP / 真实机器人) with keyboard-control, MCP and
real-robot keywords, and Association relationships exist to abot-vision-001,
twin-control-001 and humanoid-model-001.
"""

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PROCESS = "abot-dev-process-001"
TARGETS = {
    "abot-vision-001": "dev-process-relates-vision",
    "twin-control-001": "dev-process-relates-twin-control",
    "humanoid-model-001": "dev-process-relates-humanoid-model",
}
KEYWORDS = ["数字孪生", "MCP", "真实物理 ROBOT", "键盘", "真实桌面人形机器人"]


def main():
    failures = []
    graph_path = os.path.join(ROOT, "design", "KG", "SystemArchitecture.json")
    if not os.path.exists(graph_path):
        print("FAIL: %s not found" % graph_path)
        return 1
    with open(graph_path, encoding="utf-8") as fh:
        graph = json.load(fh)

    process = next((e for e in graph.get("elements", []) if str(e.get("id")) == PROCESS), None)
    if process is None:
        failures.append("process '%s' not found" % PROCESS)
    else:
        if process.get("type") != "Business Process":
            failures.append("process type = %r, want 'Business Process'" % process.get("type"))
        desc = process.get("description", "")
        for kw in KEYWORDS:
            if kw not in desc:
                failures.append("process description missing keyword %s" % kw)

    rels = {str(r.get("id")): r for r in graph.get("relationships", [])}
    for target, rel_id in TARGETS.items():
        rel = rels.get(rel_id)
        if rel is None:
            failures.append("relationship '%s' not found" % rel_id)
            continue
        if rel.get("type") != "Association":
            failures.append("relationship '%s' type = %r, want Association" % (rel_id, rel.get("type")))
        endpoints = {str(rel.get("source_id")), str(rel.get("target_id"))}
        if PROCESS not in endpoints or target not in endpoints:
            failures.append("relationship '%s' endpoints not process<->%s" % (rel_id, target))

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: aBot 3-stage dev process (twin->MCP->real robot) consistent")
    return 0


if __name__ == "__main__":
    sys.exit(main())
