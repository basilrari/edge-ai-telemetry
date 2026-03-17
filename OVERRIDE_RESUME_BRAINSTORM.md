# Override & Resume — Brainstorm

**Goal:** While the drone is flying an **original mission** (AUTO + waypoints), **inject waypoints** (override). When the override is **complete**, **resume the original mission** from the right place.

---

## 1. Desired behavior (summary)

| Phase | What happens |
|-------|----------------|
| **Original mission** | Drone is in AUTO, following waypoints W0 → W1 → … → WN. We know current index (e.g. W3). |
| **Override** | User/LLM says “go here, then there” (one or more waypoints). We **pause** the mission, run the override waypoints, then **resume**. |
| **Resume** | Drone goes back to AUTO and continues from the **same** waypoint index we left off (e.g. still W3, or next W4 — see below). |

So we need:

1. **Pause** — Leave AUTO without losing “where we were” in the mission.
2. **Run override** — Execute the injected waypoints (e.g. in GUIDED).
3. **Resume** — Restore the FC mission and set “current” waypoint, then AUTO again.

---

## 2. MAVLink / ArduPilot constraints

- The **flight controller (FC)** holds a single mission in RAM (list of waypoints). In AUTO it runs that list; **MISSION_CURRENT** tells us the current sequence index.
- We **already** receive **MISSION_ITEM_INT** and **MISSION_CURRENT** in the TUI and could keep a copy of the mission on our side.
- Options for override:
  - **GUIDED + DO_REPOSITION (goto_global)** — Switch to GUIDED, send one or more MAV_CMD_DO_REPOSITION. No change to the FC’s stored mission. When done, we re-upload the same mission and set current index.
  - **MISSION_WRITE_PARTIAL_LIST** — Replace a segment of the mission in place (e.g. insert override waypoints after current index). More complex (re-numbering, takeoff/land handling) and modifies the mission on the FC; resume is “just continue AUTO” but we must have saved the original if we want true “restore”.
- **Resume** requires telling the FC “resume from waypoint index K”. ArduPilot supports **MISSION_SET_CURRENT** (or MAV_CMD_DO_SET_MISSION_CURRENT) to set the current waypoint index, then we switch to AUTO and send **MISSION_START**.

---

## 3. Implementation options

### Option A: GUIDED override + save/restore mission (recommended)

- **Pause:** Record current mission (we already have it from MISSION_ITEM_INT) and **current_seq** (from MISSION_CURRENT). Switch to **GUIDED**.
- **Override:** Execute override waypoints one-by-one in GUIDED using **goto_global** (MAV_CMD_DO_REPOSITION). We need a small loop: send goto WP1, wait until “reached”, send goto WP2, … until last.
- **Resume:** Re-upload the **original** mission to the FC (MISSION_COUNT + MISSION_ITEM_INT for each item), then **MISSION_SET_CURRENT(seq)** (the index we had at pause, or the next one if we want to skip the one we were heading to), then set mode **AUTO** and send **MISSION_START**.

**Pros:** Clear separation: FC mission is unchanged during override; we only restore it. No partial-list indexing bugs.  
**Cons:** We must implement “reached” detection for each override waypoint (position threshold or COMMAND_ACK / NAV_CONTROLLER_OUTPUT).

---

### Option B: MISSION_WRITE_PARTIAL_LIST to insert override

- **Pause:** Record mission and current_seq. Build new mission = waypoints [0..current_seq] + override_waypoints + waypoints [current_seq..end]. Use **MISSION_WRITE_PARTIAL_LIST** to write from index current_seq.
- **Override:** FC is still in AUTO; the “mission” now has the override waypoints in the middle. When the vehicle reaches the end of the override segment, we **restore** the original mission (write back from current_seq) so the rest of the flight is the original plan.

**Pros:** No mode switch to GUIDED; AUTO all the way.  
**Cons:** More complex (sequence numbers, partial list semantics, takeoff/land items). Restoring “original” still requires a second partial write. Easy to get sequence numbering wrong.

---

### Option C: Hybrid — override in GUIDED, resume by MISSION_SET_CURRENT only

- Same as A, but **resume** does **not** re-upload the mission: we assume the FC still has the same mission. We only send **MISSION_SET_CURRENT(seq)** then AUTO + MISSION_START.

**Pros:** Simpler resume (no upload).  
**Cons:** If anything else changed the mission during override (e.g. another GCS), we’re out of sync. Safer to re-upload.

---

## 4. Recommended: Option A (GUIDED + save/restore)

- **Mission store** (in drone-server): Keeps the last known full mission (from MISSION_ITEM_INT) and the **current_seq** at the time we switched to override. Updated whenever we receive MISSION_ITEM_INT / MISSION_CURRENT (or on a dedicated “capture at pause”).
- **Override executor**: Given a list of (lat, lon, alt), switch to GUIDED and for each waypoint: send **goto_global**; wait until “reached” (distance < threshold or use COMMAND_ACK / NAV_CONTROLLER_OUTPUT); then next. When all done, call **resume**.
- **Resume**: Take stored mission + stored seq → upload via MAVLink mission protocol (MISSION_COUNT, then MISSION_ITEM_INT for each) → MISSION_SET_CURRENT(seq) → set_mode_auto() → MISSION_START. Then clear “override state” and go back to “mission running”.

**“Reached” detection:** Use either (1) **distance from current position to target** (GLOBAL_POSITION_INT vs target) below a configurable threshold (e.g. 5–10 m), or (2) ArduPilot’s COMMAND_ACK for DO_REPOSITION when it considers the command reached (if available). (1) is simple and works everywhere.

---

## 5. State machine (drone-server side)

```
  ┌─────────────────┐
  │ MissionRunning  │  AUTO, following FC mission. We have mission + current_seq.
  └────────┬────────┘
           │ override(waypoints)
           ▼
  ┌─────────────────┐     override waypoints done
  │ OverrideActive   │ ──────────────────────────────┐
  │ (GUIDED,        │                               │
  │  running        │                               │
  │  override WPs)  │                               │
  └─────────────────┘                               │
                                                    ▼
  ┌─────────────────┐     upload + set_current + AUTO + start
  │ Resuming        │ ───────────────────────────────────────► MissionRunning
  │ (restore        │
  │  mission)       │
  └─────────────────┘
```

- **MissionRunning**: Normal AUTO. On “override” we save mission + current_seq, switch to OverrideActive.
- **OverrideActive**: GUIDED; override executor runs the injected waypoints; when the last is “reached”, transition to Resuming.
- **Resuming**: Re-upload original mission, MISSION_SET_CURRENT(saved_seq), AUTO, MISSION_START; then back to MissionRunning.

We can also support **cancel override** (e.g. user says “never mind, just resume now”) from OverrideActive → Resuming with the same saved mission + seq.

---

## 6. What to add in code

| Piece | Where | Purpose |
|-------|--------|--------|
| **Mission store** | New module e.g. `mission_store.rs` or inside existing `mission.rs` / TUI | Hold full mission (MISSION_ITEM_INT list) + current_seq. Updated from telemetry; “snapshot” at pause. |
| **MAVLink mission upload** | `cmd.rs` or new `mission_upload.rs` | Send MISSION_COUNT(n), then MISSION_ITEM_INT for each waypoint (from our stored mission). Handle MISSION_ACK / MISSION_REQUEST_INT handshake. |
| **MISSION_SET_CURRENT** | `cmd.rs` | Send command/message to set current waypoint index. |
| **Override executor** | New module or in TUI | Loop: for each override WP, goto_global; wait until reached (position threshold); then next. On last reached → trigger resume. |
| **Reached detection** | Same as executor | Compare GLOBAL_POSITION_INT to target; when horizontal distance (and optionally alt) < threshold, consider “reached”. Run in same thread as telemetry or small state machine in recv thread. |
| **State** | TUI or shared state | Enum: MissionRunning | OverrideActive { override_waypoints, index } | Resuming. Drives mode changes and executor. |
| **TUI / API** | TUI key or future HTTP | “Start override” with list of waypoints (e.g. current position + one “go here” from map). “Cancel override” → resume immediately. |

---

## 7. Resume index: same vs next waypoint

- **Same index (current_seq):** Drone will fly again to the waypoint it was heading to when we paused. Good if we want no “skip”.
- **Next index (current_seq + 1):** Skip the waypoint we were heading to. Good if we consider “we already went near it” during override.

Recommendation: **resume from same index (current_seq)** by default; optionally make it configurable (e.g. “resume from next” as a parameter).

---

## 8. Open questions

1. **Frame for altitude:** Stored mission uses frame (AMSL vs relative). When we inject override waypoints, use same frame as current mission or default (e.g. relative to home)? Need to pass frame through goto_global / DO_REPOSITION if we support both.
2. **Takeoff in mission:** Original mission may have takeoff at seq 0. When we re-upload on resume, we upload the full list including takeoff; MISSION_SET_CURRENT(seq) with seq > 0 is fine. No need to strip takeoff.
3. **RTL/land during override:** If user sends RTL or land during OverrideActive, we can treat as “cancel override and RTL/land” (no resume). Simple.
4. **Multiple overrides in a row:** If we’re in OverrideActive and user sends another “go here”, we can either append to current override list or replace. Append is more intuitive for “go here, then there, then resume.”

---

## 9. Implementation order (suggested)

1. **Mission store** — Snapshot mission + current_seq from existing MISSION_ITEM_INT / MISSION_CURRENT in TUI (or shared state). No MAVLink upload yet.
2. **Mission upload** — Implement full mission upload (MISSION_COUNT + MISSION_ITEM_INT handshake) + **MISSION_SET_CURRENT** in library.
3. **Override executor** — GUIDED + sequence of goto_global + “reached” by distance threshold; on last reached call resume.
4. **Resume** — Wire: upload stored mission → set current → AUTO → MISSION_START. Integrate with executor.
5. **TUI** — Key (e.g. `i` = inject: add current position as override waypoint, then resume when done; or `o` = override with one “go to current cursor position”). Later: API for frontend/Gateway.

Once this is done, we have “inject waypoints in between mission and return to original mission” working; then we can refine (cancel, append override, next-vs-same index) and plug in the LLM.
