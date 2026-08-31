#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Shared helper: verify an aBot Business Actor <-> opencode agent consistency.

Checks the intent graph actor element (id/type/agent/model attributes) against
the matching .opencode/agent/<name>.md frontmatter model.
"""

import json
import os
import re


def _repo_root():
    return os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def check(actor_id, agent_name, expected_model):
    failures = []
    root = _repo_root()

    graph_path = os.path.join(root, "design", "KG", "SystemArchitecture.json")
    if not os.path.exists(graph_path):
        return ["intent graph not found: %s" % graph_path]
    with open(graph_path, encoding="utf-8") as fh:
        graph = json.load(fh)

    actor = next(
        (e for e in graph.get("elements", []) if str(e.get("id")) == actor_id), None
    )
    if actor is None:
        failures.append("actor '%s' not found in intent graph" % actor_id)
    else:
        if actor.get("type") != "Business Actor":
            failures.append("actor type = %r, want 'Business Actor'" % actor.get("type"))
        attrs = {}
        for attr in actor.get("attributes", []) or []:
            if isinstance(attr, dict):
                attrs[attr.get("name")] = attr.get("value")
        if attrs.get("agent") != agent_name:
            failures.append("actor attribute agent = %r, want %r" % (attrs.get("agent"), agent_name))
        if attrs.get("model") != expected_model:
            failures.append("actor attribute model = %r, want %r" % (attrs.get("model"), expected_model))

    agent_path = os.path.join(root, ".opencode", "agent", "%s.md" % agent_name)
    if not os.path.exists(agent_path):
        failures.append(".opencode/agent/%s.md not found" % agent_name)
    else:
        with open(agent_path, encoding="utf-8") as fh:
            content = fh.read()
        match = re.search(r"^model:\s*(\S+)", content, re.MULTILINE)
        actual_model = match.group(1).strip() if match else None
        if actual_model != expected_model:
            failures.append(
                ".opencode/agent/%s.md model = %r, want %r" % (agent_name, actual_model, expected_model)
            )

    return failures
