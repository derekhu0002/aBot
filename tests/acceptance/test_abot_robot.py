#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: aBot robot persona actor <-> opencode agent consistency.

GIVEN the intent graph contains the aBot 机器人本体 Business Actor and the repo
contains its opencode persona agent definition
WHEN validating consistency
THEN the actor exists with type 'Business Actor', attributes agent=abot and
model=alibaba-cn/qwen3.8-max, .opencode/agent/abot.md exists with matching
frontmatter model and mode=primary, and the persona body references the
digital twin.
"""

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

ACTOR_ID = "abot-robot-001"
AGENT_NAME = "abot"
EXPECTED_MODEL = "alibaba-cn/qwen3.8-max"


def main():
    failures = []

    graph_path = os.path.join(ROOT, "design", "KG", "SystemArchitecture.json")
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
        attrs = {}
        for attr in actor.get("attributes", []) or []:
            if isinstance(attr, dict):
                attrs[attr.get("name")] = attr.get("value")
        if attrs.get("agent") != AGENT_NAME:
            failures.append("actor attribute agent = %r, want %r" % (attrs.get("agent"), AGENT_NAME))
        if attrs.get("model") != EXPECTED_MODEL:
            failures.append("actor attribute model = %r, want %r" % (attrs.get("model"), EXPECTED_MODEL))

    agent_path = os.path.join(ROOT, ".opencode", "agent", "%s.md" % AGENT_NAME)
    if not os.path.exists(agent_path):
        failures.append(".opencode/agent/%s.md not found" % AGENT_NAME)
    else:
        with open(agent_path, encoding="utf-8") as fh:
            content = fh.read()
        model_m = re.search(r"^model:\s*(\S+)", content, re.MULTILINE)
        if model_m and model_m.group(1).strip() != EXPECTED_MODEL:
            failures.append("agent model = %r, want %r" % (model_m.group(1).strip(), EXPECTED_MODEL))
        mode_m = re.search(r"^mode:\s*(\S+)", content, re.MULTILINE)
        if not mode_m or mode_m.group(1).strip() != "primary":
            failures.append("agent mode = %r, want primary (user talks directly)" % (mode_m.group(1).strip() if mode_m else None))
        if "humanoid.blend" not in content or "twin-control" not in content:
            failures.append("persona must reference digital twin body (humanoid.blend) and twin-control")

    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1

    print("PASS: aBot persona agent consistent (actor/agent/model/mode/body)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
