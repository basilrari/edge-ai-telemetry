# Project Roadmap: Drone Control with Edge LLM

## High-level goal

**Control the drone with an edge LLM.** Natural-language input (text or voice) goes to the LLM, which formats it into structured commands for the drone (and any AI server). The operator controls the drone by intent, not by low-level MAVLink alone.

---

## Step-by-step path

| Step | Milestone | Status |
|------|-----------|--------|
| **1** | **Remote control** — Control the drone over the network (e.g. SSH tunnel; MAVLink to the FC via the Jetson). | Done |
| **2** | **Inject waypoints remotely** — From the ground (TUI or later frontend): add waypoints in flight (e.g. “add current position”, “add custom point”; append to mission or one-off “go here”). | Next |
| **3** | **Fully autonomous flight path** — Upload a full mission (predefined shape or custom list) and let the drone fly it start-to-finish. | After 2 |
| **4** | **Override in between** — While the drone is on a mission, override (e.g. “go here now”, RTL, or a short sub-mission). Original mission is paused. | After 3 |
| **5** | **Resume after override** — When the override is complete, the drone resumes the original mission from the right place (e.g. next waypoint). | After 4 |
| **6** | **Edge LLM in the loop** — Natural language → LLM → structured commands (waypoints, modes, overrides) → drone (+ AI server). Full “tell the drone in natural language” flow. | End goal |

---

## Why this order

- **Step 2** gives the primitive (add/change points remotely) that **3** (full path) and **4** (override) both use.
- **Step 3** defines “original mission” so **4** and **5** (override and resume) have a clear meaning.
- **4** and **5** together give “interrupt, do something else, then continue” — the behaviour you want before the LLM can say things like “go check that tree, then continue your survey.”
- **6** then only needs to output the same kinds of commands you already support: waypoints, modes, overrides, resume.

---

## Summary

**Goal:** Control the drone via an edge LLM from natural language.

**Path:** Remote control → inject waypoints remotely → full autonomous path → override mid-mission → resume original mission → add the LLM.

**Next concrete step:** Implement remote waypoint injection (from the TUI or a small API the future frontend/LLM will call).
