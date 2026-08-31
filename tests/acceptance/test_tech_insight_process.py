#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: tech-insight team structure & collaboration workflow.

GIVEN aBot intent graph contains the 技术洞察流程 Business Process
WHEN validating team structure and collaboration relationships
THEN the process exists as Business Process, all 5 team actors are Assignment
to the process, the process is Association to abot-vision-001, and the
tech-insight-team-001 sub-view is mounted under the process and includes all
actors and the Assignment relationships.
"""

import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PROCESS = "tech-insight-process-001"
VISION = "abot-vision-001"
ACTORS = [
    "tech-insight-lead-001",
    "tech-radar-analyst-001",
    "tech-evaluator-001",
    "tech-validator-001",
    "tech-report-writer-001",
]
ASSIGNMENTS = [
    "lead-assigned-to-insight-process",
    "radar-assigned-to-insight-process",
    "evaluator-assigned-to-insight-process",
    "validator-assigned-to-insight-process",
    "writer-assigned-to-insight-process",
]
VIEW = "tech-insight-team-001"
RELATES = "insight-process-relates-abot-vision"


def main():
    failures = []

    graph_path = os.path.join(ROOT, "design", "KG", "SystemArchitecture.json")
    if not os.path.exists(graph_path):
        print("FAIL: %s not found" % graph_path)
        return 1
    with open(graph_path, encoding="utf-8") as fh:
        graph = json.load(fh)

    elements = {str(e.get("id")): e for e in graph.get("elements", [])}
    relationships = {str(r.get("id")): r for r in graph.get("relationships", [])}
    views = {str(v.get("view_id")): v for v in graph.get("views", [])}

    process = elements.get(PROCESS)
    if process is None:
        failures.append("process '%s' not found" % PROCESS)
    elif process.get("type") != "Business Process":
        failures.append("process type = %r, want 'Business Process'" % process.get("type"))

    for actor in ACTORS:
        if actor not in elements:
            failures.append("actor '%s' not found" % actor)
        elif elements[actor].get("type") != "Business Actor":
            failures.append("actor '%s' type = %r, want 'Business Actor'" % (actor, elements[actor].get("type")))

    for rid in ASSIGNMENTS:
        rel = relationships.get(rid)
        if rel is None:
            failures.append("assignment relationship '%s' not found" % rid)
            continue
        if rel.get("type") != "Assignment":
            failures.append("relationship '%s' type = %r, want 'Assignment'" % (rid, rel.get("type")))
        if str(rel.get("target_id")) != PROCESS:
            failures.append("relationship '%s' target = %r, want %s" % (rid, rel.get("target_id"), PROCESS))

    relates = relationships.get(RELATES)
    if relates is None:
        failures.append("relationship '%s' not found" % RELATES)
    else:
        if str(relates.get("source_id")) != PROCESS or str(relates.get("target_id")) != VISION:
            failures.append("relationship '%s' endpoints not process->vision" % RELATES)

    view = views.get(VIEW)
    if view is None:
        failures.append("view '%s' not found" % VIEW)
    else:
        if view.get("parent_element_id") != PROCESS:
            failures.append("view '%s' parent = %r, want %s" % (VIEW, view.get("parent_element_id"), PROCESS))
        incl_elements = set(view.get("included_elements", []) or [])
        for actor in ACTORS:
            if actor not in incl_elements:
                failures.append("view '%s' missing actor %s" % (VIEW, actor))
        if PROCESS not in incl_elements:
            failures.append("view '%s' missing process %s" % (VIEW, PROCESS))
        incl_rels = set(view.get("included_relationships", []) or [])
        for rid in ASSIGNMENTS:
            if rid not in incl_rels:
                failures.append("view '%s' missing assignment %s" % (VIEW, rid))

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: tech-insight team structure & workflow consistent (5 actors -> process -> vision)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
