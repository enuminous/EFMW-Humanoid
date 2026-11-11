# EFMW Humanoid — Research Testbed

**Monolithic LLC · Matthew Chenoweth Wright · 2026-11-11**

> **A physical humanoid has not been built, operated, or safety-certified.** This repository documents a proposed 32-actuator research platform and an interactive **browser-based kinematic and approximate-dynamics testbed**. Its physics outputs are pedagogical model estimates, not hardware-qualified results or whole-body rigid-contact simulation.

## Explore

- **[Interactive 3D humanoid testbed](index.html)** — a multi-joint 3D robot, repeatable motion scenarios, synthetic sensors and fault injection, EFMW-style advisory anomaly monitor, joint inspector, and frozen assembly-gate demo.
- **[Complete system engineering specification v0.1](SPEC.md)** — mission, requirements, architecture, actuation, safety, controls, recursive assembly, evidence rules, and test plan.
- **[Original reference package](research/README.md)** — preserved local Python prototype and source documentation.
- **[Testbed assumptions and limitations](docs/TESTBED.md)** — exact approximation, nonclaims, reproducibility, controls, and scientific boundaries.

**Expected GitHub Pages URL:** https://enuminous.github.io/EFMW-Humanoid/ — **do not infer that Pages is live from the presence of `index.html`.** See GitHub Settings → Pages or the deployment workflow status.

## The core engineering idea

Conventional robot estimation, control and functional-safety mechanisms govern real-world motion. EFMW-inspired **state/history/coherence monitoring** is tested *alongside* ordinary baseline monitors; it does not bypass safety logic or directly drive actuators.

In the browser demo, actuators and sensors are simulated and no hardware interface is present. The recursive assembly graph may propose stages and record simulated evidence but **cannot approve physical assembly**, certify a result, or authorize hardware motion.

## What is actually implemented

- Static Three.js/WebGL humanoid scene (procedurally generated rigid-link representation; not CAD/MJCF or calibrated hardware).
- 32 named joints matching the preserved `research/config/architecture.json`, with selected joint sliders and deterministic motion programs.
- Explicit synthetic disturbances: sensor noise, delayed samples, contact disagreement, thermal risk, and tracking-error faults.
- Deterministic integration and replayable reset/seed controls.
- Toy balance/support-proxy and collision-proxy observations; there is **no full contact dynamics, no forward dynamics of all 32 joints, no torque-limit-certified controller, and no real robot control**.
- A conventional instantaneous threshold monitor and EFMW-inspired history/coherence monitor as comparative *software demonstrations*.
- Evidence-gated assembly stage navigator, with the original graph and physical-stage authorizations remaining **UNRESOLVED**.
- Automated Node.js tests, Python reference tests and publishable GitHub Pages entry point.

## Canonical source and evidence

The project is based on the published paper and research bundle in [enuminous/papers](https://github.com/enuminous/papers/tree/main/research/efmw-humanoid-platform-v0.1). EFMW's proposed additional physical/gravitational interpretation is not validated; the humanoid design does not depend on it.

**PASS** = precise executed software check, **FAIL** = failed check, **UNRESOLVED** = not established. A passing simulator test is not a physical robot acceptance test.

*The robot can remember a mistake. It cannot grant itself permission to make one dangerous.*
