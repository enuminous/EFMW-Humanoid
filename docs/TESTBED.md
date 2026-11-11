# EHP-3D-SIM-001 — Browser testbed methodology and evidence contract

**Revision:** v0.1 · **Date:** 2026-11-11  
**Owner:** Matthew Chenoweth Wright / Monolithic LLC  
**Classification:** interactive research visualization + deterministic kinematic simulator + synthetic sensor/monitor experiment.  
**Readiness:** SOFTWARE DEMONSTRATION ONLY. **No actual humanoid robot exists in this package. No hardware movement or safety certification is asserted.**

## 1. How to launch

The repository's \`index.html\` is designed for GitHub Pages at \`https://enuminous.github.io/EFMW-Humanoid/\`. Because ES modules and \`fetch\` are used, serve a local checkout through HTTP if opening it on your computer:

\`\`\`bash
python3 -m http.server 8000
# then open http://localhost:8000/
\`\`\`

The 3D viewport uses **Three.js 0.187.0** and the \`OrbitControls\` addon through a pinned public jsDelivr import map. Network access to the CDN and browser WebGL support are required for the rendered view; the page text, documentation links and deterministic core simulation modules are served from the repository. If the 3D dependency cannot load, the interface explains the failure and never substitutes unverified 3D physics.

Sources: [Three.js installation guide](https://threejs.org/manual/#en/installation), [Three.js](https://threejs.org), [Three.js source](https://github.com/mrdoob/three.js). Third-party runtime is MIT-licensed; see Three.js's upstream license. No third-party source or font is bundled.

## 2. Exact simulated body

The browser generates a rigid-link **visual representation** of a 32-DoF body using Three.js Groups, cylinders, boxes, spheres and joint markers. This is **not a calibrated CAD model, URDF, or MJCF**, has not been compared against hardware dimensions, and is not mass/inertia-accurate.

The 32 joints and seven modules are read from the specification's 32-actuator allocation, preserved in \`research/config/architecture.json\`. The browser joint table and procedural 3D rig are checked against these names in the Node test suite.

- Neck 2 (yaw, pitch)
- Left arm 7 and right arm 7 (shoulder pitch/roll/yaw, elbow pitch, forearm roll, wrist pitch/yaw)
- Waist 2 (yaw, pitch)
- Left leg 6 and right leg 6 (hip yaw/roll/pitch, knee pitch, ankle pitch/roll)
- Left and right gripper apertures 2.

The body is modeled with a floating base visually fixed around 0.81 m ground-relative height. Gripper aperture is a dimensionless linear command from 0 to 1 and **not** a calibrated rotary actuator. Intended physical actuator torque, inertia, gearing, current, heat, battery and contact forces are not modeled. The visual robot has no actual ability to walk.

## 3. Deterministic motion integration

\`assets/sim-core.mjs\` provides a fixed-step \(\Delta t=1/60\,\mathrm{s}\) reference loop. A prescribed motion program returns joint targets \(q^*_j(t)\) from bounded sine waves, or a manual target vector. For each joint:

\[
\Delta q_j=\operatorname{clip}\left( (q^*_j-q_j)\min(1,9\Delta t),\,-v_{\max}\Delta t,\,+v_{\max}\Delta t\right),
\qquad
q_j^+ = \operatorname{clip}(q_j+\Delta q_j,q_{j,\min},q_{j,\max}).
\]

The illustrative maximum rate is \(v_{\max}=2.4\) units per second in nominal modes, or \(0.12\) units per second in the synthetic actuator-lag fault. Those units are radians for rotary joints and aperture fraction for grippers. They are *design/example parameters*, not motor specifications.

Available scenarios: idle, arm signal/wave, reach, walking pattern, squat cycle, balance challenge and manual joint control. These are **scripted kinematics**. Their existence is not evidence that a real robot can execute them without falling.

A simulation run is reproducible by its random seed (default 23), scenario, fault, amplitude, speed, action log and 60-Hz sequence of steps. An xorshift PRNG synthesizes limited sensor noise.

## 4. Synthetic sensors and faults

Each cycle produces four normalized features between 0 and 1:

1. \`tracking_error\` — scaled difference between the target and realized *kinematic* pose.
2. \`sensor_innovation\` — deterministic low-amplitude noise, optionally increased with drift.
3. \`contact_disagreement\` — a scripted contact proxy disturbed by a simulated lost-contact state.
4. \`thermal_margin_risk\` — a proxy based on scripted joint motion rate, optionally elevated by a thermal fault.

All four values are constructed, **not received from actual sensors**.

Fault options:
- **None:** nominal synthetic observations.
- **Encoder drift:** increasing normalized innovation term.
- **Contact disagreement:** altered contact proxy and observation.
- **Thermal risk:** elevated synthetic thermal feature.
- **Actuator lag:** slower kinematic rate limits causing larger tracking error.
- **Stale telemetry:** after 2.0 simulated seconds, monitor input is marked stale and the advisory result is UNRESOLVED/HOLD.

There are no hardware encoders, inertial measurements, torque sensors, force/torque plates, temperature measurements, networked devices, or ROS topic subscriptions.

## 5. Support and center-of-mass illustrations are *not* physical balance calculations

The luminous floor plane, center marker, foot geometry and "support margin" are visual **proxies**. The center marker is derived from prescribed joint angles, not from actual mass-weighted rigid-body kinematics. The support margin is a heuristic:

\[
m_{\rm proxy}=\max(-0.24,\,0.14-|x_{\rm proxy}|-0.10(1-c_{\rm proxy})).
\]

The contact proxy is also heuristic; no force or pressure center is solved. **Do not mistake these values for real ZMP, CoM, a verified support polygon, self-collision detection, or stable locomotion.** They are meant only to make fault-monitor UX inspectable.

For real evaluation, replace with verified generalized coordinates \(q,\dot q\), articulated inertias, geometric collision and contact constraints, external force models, support transitions, whole-body torque-based control, and safety-controlled hardware envelopes.

## 6. Comparison monitor: conventional instant rule versus EFMW-inspired history

**C0 — Instant conventional alert:** \(\text{alert}=\bigvee_i f_i>0.8\) for the four normalized features. This is a simple baseline for UI comparison, **not** a strong or matched conventional monitoring family.

**M1 — Proposed read-only history/coherence model:** initialized with feature memory \(m_0=f_0\) and a neutral coherence \(C_0=0.5\). For later samples:

\[
\nu_t=\sqrt{\frac14\sum_i(f_{t,i}-m_{t-1,i})^2},\qquad
C_t=\operatorname{clip}_{[0,1]}\bigl(0.88C_{t-1}+0.12e^{-2\nu_t}-0.05\,\mathbf 1_{\rm anomaly}\bigr),
\]
\[
m_{t,i}=0.85m_{t-1,i}+0.15f_{t,i}.
\]

HOLD is only a **recommendation** triggered by \(C_t<0.40\), any instantaneous feature \(>0.8\), or a separate simulated anomaly. Stale telemetry yields **UNRESOLVED/HOLD**. Memory is compared before updating to avoid erasing innovation. It is a simplified algorithm closely aligned with the earlier offline \`research/prototype/efmw_supervisor.py\`; **it is not a full implementation of source equation ME-024**.

The two monitors are **not** information or alarm-budget matched. This dashboard does **not** establish higher precision, lower false-positive rate, fault-identification accuracy, calibrated confidence, or EFMW-specific improvement over controls.

### Required independent physical study

To claim improvement, preregister a real-valued fault-prediction endpoint, sensor suite, sampling rates, frozen train/validation/test separation, classical alternative monitors (CUSUM, EWMA, Kalman innovation gates, spectral/multiscale features, ML baselines as appropriate), common data access and matched false-alarm budgets. Evaluate on realistic physics-based or measured sequences, fault-onset times and blind independent holdout, then independently replicate and publish both positive and negative outcomes. A regression or deployment safety case would need separate qualification.

## 7. Recursive assembly: a preview, not a sign-off

\`research/config/assembly_graph.json\` retains nine source stages A0–A8 with dependencies and original status \`UNRESOLVED\`. The web interface generates one next *dependency-eligible preview* and lets the user record a synthetic review event:

\`{stage, decision:"SIMULATED_PREVIEW_ONLY", approved:false, physical:<boolean>}\`

All original source approvals remain \`false\`; the app does not accept a person-name string, bypass an independent reviewer, issue a physical permit, or mark any physical step as PASS. The first physical work remains blocked pending hazard, engineering, energy-isolation and emergency-stop evidence. See [full v0.1 spec](../SPEC.md), [hazard register](../research/config/hazards.json), and [original offline supervisor](../research/prototype/efmw_supervisor.py).

**No browser operation has an output path to an actuator or external device.**

## 8. Source tests and provenance

Run the JavaScript test suite with Node.js 22+:

\`\`\`sh
node --test tests/test_sim_core.mjs
node --check assets/sim-core.mjs
node --check assets/scene3d.mjs
node --check assets/app.mjs
\`\`\`

Run the preserved conventional Python reference tests:

\`\`\`sh
cd research
python3 -m unittest discover -s tests -v
python3 examples/simulate_offline.py
python3 examples/two_link_offline.py
\`\`\`

The Python prototype and v0.1 source specification are copied from [enuminous/papers](https://github.com/enuminous/papers/tree/main/research/efmw-humanoid-platform-v0.1). They are retained as source snapshots rather than being silently upgraded. Test green is a local software-contract result, not hardware acceptance.

## 9. Research-status register

| Question | Status |
|---|---|
| Full 32-joint source inventory and rig naming | Testable in repo |
| Browser/WebGL 3D articulated visualization | Implemented, subject to browser compatibility |
| Deterministic scripted kinematics and synthetic faults | Implemented in JavaScript |
| Realistic contact/whole-body forward dynamics | **UNRESOLVED / NOT IMPLEMENTED** |
| Physical robot CAD, bill of materials, actuators and calibration | **UNRESOLVED / NOT BUILT** |
| ROS 2, ros2_control, MuJoCo, MoveIt 2 or Isaac Sim integration | **PROPOSED / NOT WIRED** |
| Safety assessment, E-stop, functional safety, hardware authorization | **UNRESOLVED / NOT APPROVED** |
| EFMW-specific superiority over conventional robotics baselines | **UNRESOLVED** |
| A new EFMW physical field | **NOT ESTABLISHED** |

### Engineering rule

**A digital twin can be useful before its claims become evidence. But a green dashboard must never be mistaken for an independently certified machine.**
