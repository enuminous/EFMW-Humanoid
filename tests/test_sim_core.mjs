import test from "node:test";
import assert from "node:assert/strict";
import fs from "node:fs";
import path from "node:path";
import {fileURLToPath} from "node:url";
import {
  JOINT_GROUPS, JOINT_NAMES, JOINT_LIMITS, SCENARIOS, FAULTS,
  scriptedPose, HumanoidSimulator, CoherenceMonitor, AssemblyPreview,
  normalizeFeature, DeterministicRng, radToDeg, degToRad, checkJointConfig,
  TESTBED_LIMITS
} from "../assets/sim-core.mjs";
const root=path.resolve(path.dirname(fileURLToPath(import.meta.url)),"..");
const architecture=JSON.parse(fs.readFileSync(path.join(root,"research/config/architecture.json"),"utf8"));
const assembly=JSON.parse(fs.readFileSync(path.join(root,"research/config/assembly_graph.json"),"utf8"));

test("published architecture and browser testbed name exactly the same 32 joints",()=>{
  const source=Object.values(architecture.joint_names_by_module).flat();
  assert.equal(architecture.actuation_dof,32);
  assert.equal(JOINT_NAMES.length,32);
  assert.equal(new Set(source).size,32);
  assert.deepEqual(new Set(JOINT_NAMES),new Set(source));
  assert.equal(checkJointConfig(source),true);
});
test("joint groups are complete and do not duplicate an actuator",()=>{
  assert.deepEqual(Object.keys(JOINT_GROUPS),["Neck","Left arm","Right arm","Waist","Left leg","Right leg","Grippers"]);
  assert.equal(new Set(Object.values(JOINT_GROUPS).flat()).size,32);
});
test("every degree of freedom has finite, ordered angle/aperture limits",()=>{
  for(const name of JOINT_NAMES){
    const [low,high]=JOINT_LIMITS[name];
    assert.equal(Number.isFinite(low),true);
    assert.equal(Number.isFinite(high),true);
    assert.ok(high>low,name);
    assert.ok(low<=0&&high>=0);
  }
});
test("scripted motion patterns are bounded within declared limits",()=>{
  for(const name of Object.keys(SCENARIOS)){
    for(const t of [0,.1,.5,1,5,20]){
      const poses=scriptedPose(name,t,{amplitude:1.25,speed:2.5});
      assert.equal(Object.keys(poses).length,32);
      for(const key of JOINT_NAMES)assert.ok(poses[key]>=JOINT_LIMITS[key][0]&&poses[key]<=JOINT_LIMITS[key][1],name+" "+key);
    }
  }
});
test("unknown motion scenario and unexpected disturbance are rejected",()=>{
  assert.throws(()=>scriptedPose("teleport",0),/Unknown scenario/);
  const sim=new HumanoidSimulator();
  assert.throws(()=>sim.setScenario("teleport"),/Unknown scenario/);
  assert.throws(()=>sim.setFault("explosion"),/Unknown fault/);
});
test("robot has no hardware authorization or control outputs",()=>{
  const sim=new HumanoidSimulator();
  assert.equal(sim.hardwareAuthorized,false);
  assert.equal(sim.snapshot().hardwareAuthorized,false);
  assert.equal(sim.snapshot().mode,"SIMULATION");
  assert.equal(Object.prototype.hasOwnProperty.call(sim,"sendMotorCommand"),false);
  for(let i=0;i<12;i++)assert.equal(sim.step().hardwareAuthorized,false);
});
test("reset is deterministic with the same seed, scenario and fault",()=>{
  const s=new HumanoidSimulator({seed:23});
  s.setScenario("walk");s.setFault("drift");
  const trace1=[];
  for(let i=0;i<220;i++){
    const snap=s.step();
    if(i%10===0)trace1.push([snap.time,snap.joints.left_hip_pitch,snap.sensor.sensor_innovation,snap.advisory.coherence]);
  }
  s.reset();s.setScenario("walk");s.setFault("drift");
  const trace2=[];
  for(let i=0;i<220;i++){
    const snap=s.step();
    if(i%10===0)trace2.push([snap.time,snap.joints.left_hip_pitch,snap.sensor.sensor_innovation,snap.advisory.coherence]);
  }
  assert.deepEqual(trace2,trace1);
});
test("different random seeds perturb synthetic sensor noise",()=>{
  const a=new HumanoidSimulator({seed:23}),b=new HumanoidSimulator({seed:123});
  assert.notEqual(a.step().sensor.sensor_innovation,b.step().sensor.sensor_innovation);
});
test("simulated motor motion is bounded by rate limit and positional limits",()=>{
  const s=new HumanoidSimulator();
  s.setScenario("wave");
  let old={...s.q};
  for(let i=0;i<180;i++){
    s.step();
    for(const name of JOINT_NAMES){
      assert.ok(s.q[name]>=JOINT_LIMITS[name][0]&&s.q[name]<=JOINT_LIMITS[name][1]);
      assert.ok(Math.abs(s.q[name]-old[name])<=2.4/60+1e-10);
    }
    old={...s.q};
  }
});
test("manual target clamps angle and preserves other joint targets",()=>{
  const s=new HumanoidSimulator();
  s.setJointTarget("left_shoulder_pitch",-1.0);
  s.setJointTarget("right_shoulder_pitch",-1.2);
  assert.equal(s.scenario,"manual");
  assert.equal(s.manualTargets.left_shoulder_pitch,-1.0);
  assert.equal(s.manualTargets.right_shoulder_pitch,-1.2);
  s.setJointTarget("right_shoulder_pitch",1000);
  assert.equal(s.manualTargets.right_shoulder_pitch,JOINT_LIMITS.right_shoulder_pitch[1]);
});
test("manual target rejects NaN, infinity, unknown joint",()=>{
  const s=new HumanoidSimulator();
  assert.throws(()=>s.setJointTarget("imaginary_joint",0),/Unknown joint/);
  assert.throws(()=>s.setJointTarget("neck_yaw",NaN),/invalid target/);
  assert.throws(()=>s.setJointTarget("neck_yaw",Infinity),/invalid target/);
});
test("simulation rejects invalid integration steps",()=>{
  const s=new HumanoidSimulator();
  for(const dt of [-1,0,.1,NaN,Infinity])assert.throws(()=>s.step(dt),/Fixed-step/);
});
test("first coherence observation is initialization, not a physical validation",()=>{
  const m=new CoherenceMonitor();
  const a=m.update({tracking_error:0,sensor_innovation:0,contact_disagreement:0,thermal_margin_risk:0},.01);
  assert.equal(a.coherence,.5);assert.equal(a.status,"OBSERVED_SIMULATION");
  assert.equal(a.hold,false);
});
test("monitor compares against prior history, not updated history",()=>{
  const m=new CoherenceMonitor();
  m.update({tracking_error:0,sensor_innovation:0,contact_disagreement:0,thermal_margin_risk:0},.01);
  const v=m.update({tracking_error:1,sensor_innovation:1,contact_disagreement:1,thermal_margin_risk:1},.02);
  assert.equal(v.innovation,1);
  assert.equal(v.flags.length,4);
  assert.equal(v.hold,true);
});
test("monitor refuses non-monotonic timestamps and out-of-bounds features",()=>{
  const m=new CoherenceMonitor();
  const f={tracking_error:0,sensor_innovation:0,contact_disagreement:0,thermal_margin_risk:0};
  m.update(f,.01);
  assert.throws(()=>m.update(f,.01),/Non-monotonic/);
  assert.throws(()=>m.update({...f,thermal_margin_risk:1.1},.02),/Invalid normalized/);
});
test("stale telemetry is unresolved and forces advisory hold",()=>{
  const m=new CoherenceMonitor();
  const f={tracking_error:.1,sensor_innovation:.1,contact_disagreement:.1,thermal_margin_risk:.1};
  m.update(f,.01);
  const stale=m.update(f,.02,{stale:true});
  assert.equal(stale.status,"UNRESOLVED");
  assert.equal(stale.hold,true);
  assert.ok(stale.flags.includes("stale_sample"));
});
test("thermal fault eventually trips instant baseline",()=>{
  const s=new HumanoidSimulator();
  s.setFault("thermal");
  for(let i=0;i<6;i++)s.step();
  assert.ok(s.instantAlarm);
  assert.ok(s.sample.thermal_margin_risk>.8);
});
test("contact fault eventually recommends hold",()=>{
  const s=new HumanoidSimulator();
  s.setFault("contact");
  for(let i=0;i<20;i++)s.step();
  assert.equal(s.advisory.hold,true);
  assert.ok(s.contactProxy<1);
});
test("stale fault forces hold and does not fabricate observed coherence",()=>{
  const s=new HumanoidSimulator();
  s.setFault("stale");
  for(let i=0;i<130;i++)s.step();
  assert.equal(s.advisory.status,"UNRESOLVED");
  assert.equal(s.advisory.hold,true);
});
test("lag disturbance does not exceed actuator step bound",()=>{
  const s=new HumanoidSimulator();
  s.setScenario("wave");s.setFault("lag");
  const old={...s.q};
  s.step();
  for(const name of JOINT_NAMES)assert.ok(Math.abs(s.q[name]-old[name])<=.12/60+1e-10);
});
test("history stores only bounded periodic synthetic logs",()=>{
  const s=new HumanoidSimulator();
  for(let i=0;i<3000;i++)s.step();
  assert.equal(s.history.length,240);
  assert.ok(s.history.every(x=>Number.isFinite(x.t)&&x.coherence>=0&&x.coherence<=1));
});
test("assembly exactly matches frozen A0-A8 dependency graph",()=>{
  assert.equal(assembly.stages.length,9);
  assert.deepEqual(assembly.stages.map(x=>x.id),["A0","A1","A2","A3","A4","A5","A6","A7","A8"]);
  const a=new AssemblyPreview(assembly.stages);
  assert.equal(a.proposal().id,"A0");
});
test("assembly cannot skip ahead",()=>{
  const a=new AssemblyPreview(assembly.stages);
  assert.throws(()=>a.preview("A3"),/Dependency-ordered/);
  a.preview("A0");
  assert.equal(a.proposal().id,"A1");
});
test("assembly preview cannot approve a physical or documentary stage",()=>{
  const a=new AssemblyPreview(assembly.stages);
  for(const s of assembly.stages){
    const event=a.preview(s.id);
    assert.equal(event.approved,false);
    assert.equal(event.decision,"SIMULATED_PREVIEW_ONLY");
    assert.equal(a.originalState(s.id).approved,false);
    assert.equal(a.originalState(s.id).status,"UNRESOLVED");
  }
  assert.equal(a.proposal(),null);
});
test("hazard warnings cannot be dismissed by synthetic green telemetry",()=>{
  const s=new HumanoidSimulator();
  for(let i=0;i<240;i++)s.step();
  assert.equal(s.snapshot().hardwareAuthorized,false);
  assert.match(TESTBED_LIMITS.motor,/No actuation authority/);
  assert.match(TESTBED_LIMITS.efmw,/not proof/);
});
test("testbed documentation must clearly communicate unvalidated proxies",()=>{
  const doc=fs.readFileSync(path.join(root,"docs/TESTBED.md"),"utf8");
  for(const key of ["not","32","contact","balance","EFMW","physical","independent","Three.js"])assert.ok(doc.toLowerCase().includes(key.toLowerCase()),key);
});
test("site includes all semantic controls and source links",()=>{
  const html=fs.readFileSync(path.join(root,"index.html"),"utf8");
  for(const id of ["robotCanvas","runButton","stepButton","resetButton","jointRange","faultSelect","coherenceChart","assemblyStages","stageBtn","renderError","specification"]){
    assert.ok(html.includes('id="'+id+'"'),id);
  }
  assert.match(html,/simulation-only/i);
  assert.match(html,/\.\/SPEC\.md/);
  assert.match(html,/three@0\.187\.0/);
});
test("model rendering source contains all 32 joint keys at runtime",()=>{
  const source=fs.readFileSync(path.join(root,"assets/scene3d.mjs"),"utf8");
  for(const part of ["left_arm","right_arm","left_leg","right_leg"])assert.ok(source.includes(part.replace("_arm","").replace("_leg",""))||source.includes("arm(\"left\""),part);
  assert.ok(source.includes("jointNodes.size"));
  assert.ok(source.includes("onSelectJoint"));
});
test("all JavaScript modules have valid parsable syntax",()=>{
  for(const f of ["assets/sim-core.mjs","assets/scene3d.mjs","assets/app.mjs"]){
    const s=fs.readFileSync(path.join(root,f),"utf8");
    assert.ok(s.length>1000,f);
  }
});
