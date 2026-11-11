/*
 * EHP-3D-SIM-001: deterministic SIMULATION-ONLY humanoid kinematic testbed.
 * No hardware drivers, ROS interfaces, firmware paths, motors or network outputs.
 * Engineering disclaimer: proxy metrics != validated whole-body dynamics.
 */
export const JOINT_GROUPS = Object.freeze({
  "Neck": ["neck_yaw", "neck_pitch"],
  "Left arm": ["left_shoulder_pitch","left_shoulder_roll","left_shoulder_yaw","left_elbow_pitch","left_forearm_roll","left_wrist_pitch","left_wrist_yaw"],
  "Right arm": ["right_shoulder_pitch","right_shoulder_roll","right_shoulder_yaw","right_elbow_pitch","right_forearm_roll","right_wrist_pitch","right_wrist_yaw"],
  "Waist": ["waist_yaw","waist_pitch"],
  "Left leg": ["left_hip_yaw","left_hip_roll","left_hip_pitch","left_knee_pitch","left_ankle_pitch","left_ankle_roll"],
  "Right leg": ["right_hip_yaw","right_hip_roll","right_hip_pitch","right_knee_pitch","right_ankle_pitch","right_ankle_roll"],
  "Grippers": ["left_gripper_aperture","right_gripper_aperture"]
});
export const JOINT_NAMES = Object.freeze(Object.values(JOINT_GROUPS).flat());
if(JOINT_NAMES.length!==32 || new Set(JOINT_NAMES).size!==32)throw Error("Expected 32 unique joints");
export const SCENARIOS = Object.freeze({
  idle:{label:"Standby / neutral",description:"Near-neutral standing posture",level:"STATIC"},
  wave:{label:"Arm signal",description:"Right-arm scripted cyclic gesture",level:"KINEMATIC"},
  reach:{label:"Object reach",description:"Bimanual forward reach sequence",level:"KINEMATIC"},
  walk:{label:"Walking pattern",description:"Prescribed alternating leg cycles, no contact solver",level:"KINEMATIC"},
  squat:{label:"Squat cycle",description:"Scripted hip/knee flexion, no ground reaction model",level:"KINEMATIC"},
  balance:{label:"Balance challenge",description:"Lateral center-of-mass proxy perturbation",level:"PROXY"},
  manual:{label:"Manual joint lab",description:"Drive 32 targets through clamped, rate-limited kinematics",level:"KINEMATIC"}
});
export const FAULTS=Object.freeze({
  none:{label:"Clean sensors"},
  drift:{label:"Encoder drift"},
  contact:{label:"Contact disagreement"},
  thermal:{label:"Thermal risk"},
  lag:{label:"Actuator lag"},
  stale:{label:"Stale telemetry"}
});
const PI=Math.PI;
export const clamp=(x,a,b)=>Math.max(a,Math.min(b,x));
export const radToDeg=x=>x*180/PI;
export const degToRad=x=>x*PI/180;
const rangeFor=name=>{
  if(name.endsWith("_gripper_aperture"))return [0,1];
  if(name.startsWith("neck_"))return name.endsWith("yaw")?[-1.0,1.0]:[-0.55,0.55];
  if(name.startsWith("waist_"))return [-0.65,0.65];
  if(name.includes("_knee_pitch"))return [0,2.0];
  if(name.includes("_elbow_pitch"))return [0,2.35];
  if(name.includes("_ankle_"))return [-0.70,0.70];
  if(name.includes("_wrist_")||name.includes("_forearm_"))return [-1.25,1.25];
  if(name.includes("_hip_"))return name.endsWith("pitch")?[-1.1,1.05]:[-0.75,0.75];
  if(name.includes("_shoulder_"))return [-1.7,1.7];
  return [-1.0,1.0];
};
export const JOINT_LIMITS=Object.freeze(Object.fromEntries(JOINT_NAMES.map(n=>[n,Object.freeze(rangeFor(n))])));
export function jointGroup(name){
  for(const [group,names] of Object.entries(JOINT_GROUPS))if(names.includes(name))return group;
  return "Unknown";
}
export function checkJointConfig(names=JOINT_NAMES){
  return names.length===32&&new Set(names).size===32&&names.every(n=>JOINT_LIMITS[n]!==undefined);
}
export class DeterministicRng {
  constructor(seed=23){if(!Number.isSafeInteger(seed))throw Error("Seed must be an integer");this.seed=seed>>>0||1;}
  next(){let x=this.seed;x^=x<<13;x^=x>>>17;x^=x<<5;this.seed=x>>>0;return this.seed/4294967296;}
}
const put=(pose,key,val)=>{pose[key]=val;};
export function scriptedPose(scenario,t,{amplitude=0.65,speed=1}={}){
  if(!SCENARIOS[scenario])throw Error("Unknown scenario");
  const a=clamp(amplitude,0,1.25),s=clamp(speed,.2,2.5),phase=t*1.85*s;
  const p=Object.fromEntries(JOINT_NAMES.map(n=>[n,0]));
  p.left_elbow_pitch=.18;p.right_elbow_pitch=.18;
  p.left_gripper_aperture=.15;p.right_gripper_aperture=.15;
  switch(scenario){
    case "wave":
      put(p,"right_shoulder_roll",-.9*a);put(p,"right_shoulder_pitch",-.55*a);
      put(p,"right_elbow_pitch",1.6*a);
      put(p,"right_wrist_yaw",Math.sin(phase*2)*.8*a);
      put(p,"neck_yaw",Math.sin(phase*.2)*.14*a);
      break;
    case "reach":{
      const blend=.5-.5*Math.cos(phase*.62);
      for(const side of ["left","right"]){
        put(p,side+"_shoulder_pitch",-1.03*a*blend);
        put(p,side+"_shoulder_roll",(side==="left"?.22:-.22)*a);
        put(p,side+"_elbow_pitch",(.18+.72*blend)*a);
        put(p,side+"_gripper_aperture",.13+.63*blend);
      }
      put(p,"waist_pitch",-.11*a*blend);
      break;
    }
    case "walk":{
      const l=Math.sin(phase),r=-l;
      for(const [side,v] of [["left",l],["right",r]]){
        put(p,side+"_hip_pitch",.62*a*v);
        put(p,side+"_knee_pitch",Math.max(0,v)*.79*a+.075);
        put(p,side+"_ankle_pitch",-0.37*a*v);
        put(p,side+"_shoulder_pitch",-.32*a*v);
        put(p,side+"_elbow_pitch",.25);
      }
      put(p,"waist_yaw",.14*a*Math.sin(phase));
      put(p,"waist_pitch",.025*Math.sin(phase*2));
      break;
    }
    case "squat":{
      const depth=a*(.5-.5*Math.cos(phase*.9));
      for(const side of ["left","right"]){
        put(p,side+"_hip_pitch",-.87*depth);
        put(p,side+"_knee_pitch",1.42*depth);
        put(p,side+"_ankle_pitch",-.55*depth);
        put(p,side+"_shoulder_pitch",-.40*depth);
      }
      put(p,"waist_pitch",-.14*depth);
      break;
    }
    case "balance":{
      const sway=Math.sin(phase*.55);
      put(p,"waist_yaw",.12*a*sway);
      put(p,"waist_pitch",.13*a*Math.cos(phase*.6));
      put(p,"left_hip_roll",.28*a*sway);
      put(p,"right_hip_roll",.28*a*sway);
      put(p,"left_ankle_roll",-.22*a*sway);
      put(p,"right_ankle_roll",-.22*a*sway);
      break;
    }
    default:break;
  }
  for(const n of JOINT_NAMES)p[n]=clamp(p[n],...JOINT_LIMITS[n]);
  return p;
}
export function normalizeFeature(input){
  const names=["tracking_error","sensor_innovation","contact_disagreement","thermal_margin_risk"];
  if(!input||Object.keys(input).length!==4||!names.every(n=>Number.isFinite(input[n])&&input[n]>=0&&input[n]<=1))throw Error("Invalid normalized sensor feature schema");
  return Object.fromEntries(names.map(n=>[n,input[n]]));
}
export class CoherenceMonitor{
  constructor(){this.reset();}
  reset(){this.memory=null;this.coherence=.5;this.time=null;this.last={coherence:.5,innovation:0,status:"UNRESOLVED",hold:false,flags:[],baseline:false};}
  update(features,t,{stale=false,anomaly=false}={}){
    if(!Number.isFinite(t)||(this.time!==null&&t<=this.time))throw Error("Non-monotonic simulation telemetry");
    if(stale){
      this.last={...this.last,status:"UNRESOLVED",hold:true,flags:["stale_sample"],baseline:true};
      return this.last;
    }
    const f=normalizeFeature(features), keys=Object.keys(f);
    const baseline=keys.some(n=>f[n]>.8);
    let innov=0;
    if(this.memory===null)this.memory={...f};
    else{
      innov=Math.sqrt(keys.reduce((sum,n)=>sum+(f[n]-this.memory[n])**2,0)/keys.length);
      const desired=Math.exp(-2*innov);
      this.coherence=clamp(.88*this.coherence+.12*desired-(anomaly?.05:0),0,1);
      for(const n of keys)this.memory[n]=.85*this.memory[n]+.15*f[n];
    }
    this.time=t;
    const flags=keys.filter(n=>f[n]>.8);
    const hold=this.coherence<.40||flags.length>0||Boolean(anomaly);
    this.last={coherence:this.coherence,innovation:innov,status:hold?"RECOMMENDED_HOLD":"OBSERVED_SIMULATION",hold,flags,baseline};
    return this.last;
  }
}
/*
 * Documentary build-stage PREVIEW only. A preview is not a PASS, and no
 * physical stage can gain approval through this class.
 */
export class AssemblyPreview{
  constructor(stages){this.stages=stages.map(s=>({...s}));this.events=[];this.previewed=new Set();}
  proposal(){
    return this.stages.find(s=>!this.previewed.has(s.id)&&s.depends_on.every(d=>this.previewed.has(d)))||null;
  }
  preview(id){
    const proposed=this.proposal();
    if(!proposed||proposed.id!==id)throw Error("Dependency-ordered stage required");
    this.previewed.add(id);
    const event={stage:id,decision:"SIMULATED_PREVIEW_ONLY",approved:false,physical:proposed.physical};
    this.events.push(event);return event;
  }
  originalState(id){
    const stage=this.stages.find(s=>s.id===id);
    if(!stage)throw Error("Unknown stage");
    return {status:stage.status,approved:stage.approved};
  }
}
export class HumanoidSimulator{
  constructor({seed=23}={}){this.seed=seed;this.reset();}
  reset(){
    this.time=0;this.frame=0;this.running=false;this.scenario="idle";this.fault="none";this.speed=1;this.amplitude=.65;
    this.rng=new DeterministicRng(this.seed);
    this.q=Object.fromEntries(JOINT_NAMES.map(n=>[n,0]));
    this.target=scriptedPose("idle",0);
    this.manualTargets={...this.target};
    this.monitor=new CoherenceMonitor();
    this.history=[];
    this.sample=null;this.advisory=this.monitor.last;this.instantAlarm=false;
    this.comX=0;this.balanceMargin=.12;this.contactProxy=1;
    this.mode="SIMULATION";this.hardwareAuthorized=false;
    return this.snapshot();
  }
  setScenario(key){
    if(!SCENARIOS[key])throw Error("Unknown scenario");
    this.scenario=key;
    if(key==="manual")this.manualTargets={...this.q};
    return key;
  }
  setFault(key){if(!FAULTS[key])throw Error("Unknown fault");this.fault=key;return key;}
  setSpeed(x){if(!Number.isFinite(x))throw Error("Invalid speed");this.speed=clamp(x,.2,2.5);}
  setAmplitude(x){if(!Number.isFinite(x))throw Error("Invalid amplitude");this.amplitude=clamp(x,0,1.25);}
  setJointTarget(name,angle){
    if(!JOINT_LIMITS[name]||!Number.isFinite(angle))throw Error("Unknown joint or invalid target");
    this.setScenario("manual");
    this.manualTargets[name]=clamp(angle,...JOINT_LIMITS[name]);
  }
  step(dt=1/60){
    if(!Number.isFinite(dt)||dt<=0||dt>.075)throw Error("Fixed-step dt required in (0, .075]");
    this.time+=dt;this.frame++;
    const pose=this.scenario==="manual"?this.manualTargets:scriptedPose(this.scenario,this.time,{amplitude:this.amplitude,speed:this.speed});
    const target={...pose};
    const frameFault=this.fault;
    const actuatorLag=frameFault==="lag";
    let errorSum=0,velocitySum=0;
    for(const n of JOINT_NAMES){
      const v0=this.q[n],v1=target[n];
      const maxRate=actuatorLag?.12:2.4;
      const change=clamp((v1-v0)*Math.min(1,dt*9),-maxRate*dt,maxRate*dt);
      this.q[n]=clamp(v0+change,...JOINT_LIMITS[n]);
      errorSum+=Math.abs(v1-this.q[n]);
      velocitySum+=Math.abs(change/dt);
    }
    this.target=target;
    const amp=this.amplitude;
    const leftSwing=this.q.left_hip_pitch;
    const rightSwing=this.q.right_hip_pitch;
    this.comX=clamp(.015*Math.sin(this.time*.9)+.13*this.q.waist_yaw+.16*(this.q.left_hip_roll+this.q.right_hip_roll)*.5,-.4,.4);
    this.contactProxy=clamp(1-Math.abs(leftSwing-rightSwing)*.32-(frameFault==="contact"?.65:0),0,1);
    // Deliberately labeled heuristics, not solved rigid contact forces.
    this.balanceMargin=Math.max(-.24,.14-Math.abs(this.comX)-(1-this.contactProxy)*.10);
    const noise=(this.rng.next()-.5)*.022;
    const tracking=clamp(errorSum/JOINT_NAMES.length*.92+(actuatorLag?.68:0),0,1);
    const sensor=clamp(.06+Math.abs(noise)+(frameFault==="drift"?Math.min(1,this.time*.17):0),0,1);
    const contact=clamp((1-this.contactProxy)*.4+(frameFault==="contact"?.78:0),0,1);
    const thermal=clamp(velocitySum/JOINT_NAMES.length*.08+(frameFault==="thermal"?.91:0),0,1);
    this.sample={tracking_error:tracking,sensor_innovation:sensor,contact_disagreement:contact,thermal_margin_risk:thermal};
    this.instantAlarm=Object.values(this.sample).some(x=>x>.8);
    const stale=frameFault==="stale"&&this.time>=2;
    const anomaly=this.balanceMargin<0||this.contactProxy<.25;
    this.advisory=this.monitor.update(this.sample,this.time,{stale,anomaly});
    if(this.frame%6===0){
      this.history.push({
        t:+this.time.toFixed(4),coherence:+this.advisory.coherence.toFixed(5),
        innovation:+this.advisory.innovation.toFixed(5),
        tracking:+tracking.toFixed(5),margin:+this.balanceMargin.toFixed(5),
        instant:this.instantAlarm,hold:this.advisory.hold
      });
      if(this.history.length>240)this.history.shift();
    }
    return this.snapshot();
  }
  snapshot(){
    return {time:this.time,frame:this.frame,scenario:this.scenario,fault:this.fault,mode:this.mode,
      running:this.running,hardwareAuthorized:false,
      joints:{...this.q},targets:{...this.target},jointCount:JOINT_NAMES.length,
      balanceMargin:this.balanceMargin,comX:this.comX,contactProxy:this.contactProxy,
      sensor:this.sample?{...this.sample}:null,
      advisory:{...this.advisory,flags:[...this.advisory.flags]},
      instantAlarm:this.instantAlarm,history:this.history.slice(),seed:this.seed
    };
  }
}
export const TESTBED_LIMITS=Object.freeze({
  dynamics:"32-joint clamped/rate-limited kinematic pose integration; no multi-rigid-body forward dynamics",
  contact:"Approximate foot/contact proxy only, not collision geometry or pressure-force solution",
  balance:"Heuristic support margin, not computed physical ZMP or verified center of pressure",
  sensors:"Synthetic normalized measurements and deterministic faults; no measured encoder, IMU or load cell",
  efmw:"Read-only EWMA/coherence monitor; not proof of ME-024, physical field or superior safety",
  assembly:"Simulation-only preview, never physical gate approval",
  motor:"No actuation authority, hardware drivers or motor outputs"
});
