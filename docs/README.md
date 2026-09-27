# RobotCode Documentation Index

Current as of 2026-08-11.

This index classifies documentation by authority. A document containing a
proposal, recommendation, remaining-task list, or acceptance gate is not an
active backlog unless it appears under **Retained future initiatives** below
and the user explicitly requests implementation.

## Current operating documents

- `../../HANDOFF.md` — authoritative workspace status, evidence, priorities,
  and handoff rules.
- `../../AGENTS.md` — execution, safety, source-of-truth, and verification
  instructions.
- `../README.md` — active firmware architecture, commands, sensing, and
  operator-facing behavior.
- `CURRENT_STATE_AND_NEXT_STEPS.md` — concise active capability and retained
  work summary.
- `../../NAVIGATION_DESIGN.md` — current navigation API and ownership design.
- `../../CALL_FLOW_GUIDE.md` and `../../LEARNING_GUIDE.md` — explanatory
  architecture references.

## Retained future initiatives

These are the only intentionally retained future plans. They are not scheduled
and must not be implemented without an explicit user request.

1. `TOF_DOMINANT_WALL_ANCHORED_LOCALIZATION_PLAN.md` — retained localization
   initiative, updated for the current sensor architecture and observability
   limits.
2. `IMU_AIDED_POSITION_TRACKING_PLAN.md` — supporting IMU/encoder estimator
   layer for the TOF localization initiative.
3. `INTERNAL_FUNNEL_TOF_INTEGRATION_GUIDE.md` — sequence-based payload
   confirmation inside the funnel; software implemented, with physical
   characterization and acceptance still pending.

## Completed implementation records

These explain how current behavior was developed. Their proposed follow-ups
are context only.

- `CODE_CLEANUP_AND_LOCAL_PLANNER_SPLIT.md`
- `FRONT_MATRIX_THREE_REAR_VL53L1X_IMPLEMENTATION_PLAN.md`
- `FRONT_MATRIX_THREE_REAR_IMPLEMENTATION_STATUS.md`
- `PATH_EFFICIENCY_REVIEW.md`
- `ROUTE_CHOICE_AUDIT_2026-08-09.md`

## Historical and contextual evidence

These may help diagnose a related request, but they do not define current
priorities or authorize implementation.

- `ROBOT_CODEBASE_AUDIT.md` — July audit snapshot with many recommendations
  that were later implemented, superseded, rejected, or deprioritized.
- `../../REPORT_HANDOFF.md` — report-writing evidence snapshot and open-question
  catalogue, not an engineering roadmap.
- `archive/` — historical handoffs and workstreams retained as evidence.

Do not delete historical evidence merely because it is no longer active. Add a
new current decision to `HANDOFF.md` or this index instead of silently changing
the meaning of an old measurement or audit result.
