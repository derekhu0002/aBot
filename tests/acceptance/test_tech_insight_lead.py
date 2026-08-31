#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: tech-insight-lead actor <-> opencode agent consistency."""

import sys

from _actor_consistency import check

ACTOR = "tech-insight-lead-001"
AGENT = "tech-insight-lead"
MODEL = "alibaba-cn/qwen3.8-max"


def main():
    failures = check(ACTOR, AGENT, MODEL)
    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: %s actor and agent consistent (model=%s)" % (AGENT, MODEL))
    return 0


if __name__ == "__main__":
    sys.exit(main())
