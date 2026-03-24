# gimbal_stabilizer — Gimbal camera stabilization control

## Architecture
See [ARCHITECTURE.md](ARCHITECTURE.md) for node graph and data flow.

## Session Workflow Protocol

At the **START** of each session:
- Read `doc/active/feature_list.json` and `doc/active/progress.txt`
- Read `ARCHITECTURE.md` for node boundaries and topic wiring

At the **END** of each session:
- Append to `doc/active/progress.txt`: what was done, what's next
- Update `doc/active/feature_list.json` if any feature status changed
- Update the relevant node's `CONTEXT.md` if its interface (topics, services, parameters) changed
- Update `ARCHITECTURE.md` if node dependencies or topic wiring changed

## Node Navigation
Each node has a `CONTEXT.md` describing its purpose, subscriptions, publishers, services, parameters, and dependencies. Read the relevant `CONTEXT.md` before working on a node.

## Specs
Authoritative specifications live in `doc/*_spec.md`. These define *what* to build. Do not duplicate spec content elsewhere.

## Folder Semantics
```
gimbal_stabilizer/
├── ARCHITECTURE.md       # Node graph and topic/service flow
├── CLAUDE.md             # This file (session workflow)
├── CONTEXT.md            # Per-node routing contracts
├── doc/
│   ├── active/           # Multi-session tracking (feature_list.json, progress.txt)
│   ├── backlog/          # Low-urgency future tasks and research
│   └── *_spec.md         # Authoritative specs
├── config/               # Parameter YAML files
├── launch/               # Launch files
├── gimbal_stabilizer/    # Python node implementations
├── package.xml           # ROS2 package manifest
└── setup.py              # Build config (ament_python)
```
