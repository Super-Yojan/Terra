#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["eclipse-zenoh==1.10.1"]
# ///
"""Print why a rover is moving or holding, from its autonomy status.

Connects to the fleet router the same way as save-occupancy-map.py (GCP router,
user `terra`, TERRA_ROUTER_PASSWORD) and prints the first
`<prefix>/<rover>/autonomy/status` message: level, goal, the hold reason, the
route, and the local planner's rejected candidates. Add --raw for the full JSON.

  TERRA_ROUTER_PASSWORD=... uv run scripts/dashboard/show-autonomy-status.py
"""
import argparse
import collections
import json
import os
import queue
import sys

import zenoh

GCP_ROUTER = "tls/argos.34.135.204.210.sslip.io:7448"


def main():
    parser = argparse.ArgumentParser(description="Print a rover's autonomy status and hold reason.")
    parser.add_argument("--endpoint", default=GCP_ROUTER)
    parser.add_argument("--prefix", default="terra/phone")
    parser.add_argument("--rover", default="0")
    parser.add_argument("--timeout", type=float, default=10.0)
    parser.add_argument("--user", default=os.environ.get("TERRA_ROUTER_USER", "terra"))
    parser.add_argument("--password", default=os.environ.get("TERRA_ROUTER_PASSWORD"))
    parser.add_argument("--raw", action="store_true", help="print the whole status JSON")
    args = parser.parse_args()
    if args.user and not args.password:
        sys.exit(f"Set TERRA_ROUTER_PASSWORD (or --password) for user '{args.user}'; use --user \"\" for no login.")

    config = zenoh.Config()
    config.insert_json5("mode", '"client"')
    config.insert_json5("connect/endpoints", json.dumps([args.endpoint]))
    config.insert_json5("scouting/multicast/enabled", "false")
    if args.user:
        config.insert_json5("transport/auth/usrpwd/user", json.dumps(args.user))
        config.insert_json5("transport/auth/usrpwd/password", json.dumps(args.password))
    key = f"{args.prefix}/{args.rover}/autonomy/status"
    statuses = queue.Queue()
    try:
        session = zenoh.open(config)
    except zenoh.ZError as error:
        sys.exit(f"Could not connect to {args.endpoint}: {error}")
    with session:
        subscriber = session.declare_subscriber(key, lambda s: statuses.put(s.payload.to_bytes()))
        try:
            payload = statuses.get(timeout=args.timeout)
        except queue.Empty:
            sys.exit(f"No status on {key} within {args.timeout:g}s. Is the rover connected?")
        finally:
            subscriber.undeclare()

    status = json.loads(payload)
    if args.raw:
        print(json.dumps(status, indent=2))
        return
    inner = status
    route = inner.get("global_route") or []
    level = inner.get("effective_level") or inner.get("requested_level")
    print(f"level:   {level}   source: {inner.get('active_source')}   safety: {inner.get('safety')}")
    print(f"reason:  {inner.get('reason')}")
    print(f"route:   {len(route)} points" + (f", first {route[:3]} ... last {route[-1]}" if route else " (none)"))
    trace = inner.get("planner_trace")
    if trace:
        rejections = collections.Counter(c.get("rejection") or "admissible" for c in trace.get("candidates", []))
        print(f"planner: {dict(rejections)}  selected={trace.get('selected')}")
        if trace.get("maneuver_segments"):
            print(f"         maneuver segments: {len(trace['maneuver_segments'])}")
    navigation = status.get("navigation")
    if navigation:
        print(f"body:    {navigation.get('footprint')}  profile={navigation.get('profile')}")


if __name__ == "__main__":
    main()
