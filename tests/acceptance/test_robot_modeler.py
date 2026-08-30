#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: robot-modeler actor <-> opencode agent consistency.

GIVEN aBot intent graph contains the 机器人3D建模员 actor and the repo contains
its opencode agent definition
WHEN validating consistency
THEN the actor exists with type 'Business Actor', attributes
agent=robot-modeler and model=alibaba-cn/qwen3.8-max, and
.opencode/agent/robot-modeler.md exists with matching frontmatter model.
"""

import json
import os
import re
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ACTOR_ID = "robot-modeler-001"
AGENT_NAME = "robot-modeler"
EXPECTED_MODEL = "alibaba-cn/qwen3.8-max"


def load_attributes(element):
    attrs = {}
    for attr in element.get("attributes", []) or []:
        if isinstance(attr, dict):
            attrs[attr.get("name")] = attr.get("value")
    return attrs


def main():
    failures = []

    graph_path = os.path.join(REPO_ROOT, "design", "KG", "SystemArchitecture.json")
    if not os.path.exists(graph_path):
        print("FAIL: %s not found" % graph_path)
        return 1
    with open(graph_path, encoding="utf-8") as fh:
        graph = json.load(fh)

    actor = next(
        (e for e in graph.get("elements", []) if str(e.get("id")) == ACTOR_ID), None
    )
    if actor is None:
        failures.append("actor '%s' not found in intent graph" % ACTOR_ID)
    else:
        if actor.get("type") != "Business Actor":
            failures.append("actor type = %r, want 'Business Actor'" % actor.get("type"))
        attrs = load_attributes(actor)
        if attrs.get("agent") != AGENT_NAME:
            failures.append("actor attribute agent = %r, want %r" % (attrs.get("agent"), AGENT_NAME))
        if attrs.get("model") != EXPECTED_MODEL:
            failures.append("actor attribute model = %r, want %r" % (attrs.get("model"), EXPECTED_MODEL))

    agent_path = os.path.join(REPO_ROOT, ".opencode", "agent", "%s.md" % AGENT_NAME)
    if not os.path.exists(agent_path):
        failures.append(".opencode/agent/%s.md not found" % AGENT_NAME)
    else:
        with open(agent_path, encoding="utf-8") as fh:
            content = fh.read()
        match = re.search(r"^model:\s*(\S+)", content, re.MULTILINE)
        actual_model = match.group(1).strip() if match else None
        if actual_model != EXPECTED_MODEL:
            failures.append(
                ".opencode/agent/%s.md model = %r, want %r"
                % (AGENT_NAME, actual_model, EXPECTED_MODEL)
            )

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: robot-modeler actor and opencode agent are consistent (model=%s)" % EXPECTED_MODEL)
    return 0


if __name__ == "__main__":
    sys.exit(main())
