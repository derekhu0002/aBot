#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Acceptance test: tech-report-writer actor <-> opencode agent consistency."""

import sys

from _actor_consistency import check

ACTOR = "tech-report-writer-001"
AGENT = "tech-report-writer"
MODEL = "alibaba-cn/qwen3.8-flash"


def main():
    failures = check(ACTOR, AGENT, MODEL)
    if failures:
        print("FAIL:\n  - " + "\n  - ".join(failures))
        return 1
    print("PASS: %s actor and agent consistent (model=%s)" % (AGENT, MODEL))
    return 0


if __name__ == "__main__":
    sys.exit(main())
