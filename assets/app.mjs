/*
 * EFMW Humanoid GitHub Pages cockpit — browser-only simulation UI.
 * Robot scene uses Three.js; controls/monitoring continue if WebGL/CDN fails.
 */
import {
  HumanoidSimulator, AssemblyPreview, JOINT_GROUPS, JOINT_NAMES,
  JOINT_LIMITS, SCENARIOS, FAULTS, radToDeg, degToRad, clamp
} from "./sim-core.mjs";

const $=id=>document.getElementById(id);
const sim=new HumanoidSimulator({seed:23});
const FIXED_STEP=1/60;
let view=null,assembly=null,selectedJoint="right_elbow_pitch",prevFrame=null,accumulator=0,lastPaint=0;
const icons={idle:"◈",wave:"⌁",reach:"↗",walk:"⇄",squat:"↓",balance:"◇",manual:"⊞"};
const els=["runButton","stepButton","resetButton","amplitudeRange","speedRange","scenarioButtons","faultSelect",
  "jointGroup","jointSelect","jointRange","jointValue","jointCurrent","jointLimits","simTime",
  "runIndicator","coherenceValue","coherenceStatus","coherenceBar","coherenceChart","historyPath","historyArea",
  "innovationValue","trackingValue","balanceValue","contactValue","baselineStatus","efmwStatus",
  "monitorReason","sensorReadouts","stageProposal","stageEvidence","assemblyStages","stageBtn",
  "stageEventLog","stageResetBtn","selectedJointStatus","scenarioStatus","faultHelp",
  "amplitudeValue","speedValue","modelStatus","renderError","robotCanvas"];
for(const id of els)if(!$(id))throw Error("Missing UI component "+id);

function formatSeconds(t){const min=Math.floor(t/60);return String(min).padStart(2,"0")+":"+((t%60).toFixed(2)).padStart(5,"0");}
function htmlEsc(s){return String(s).replace(/[&<>"']/g,c=>({"&":"&amp;","<":"&lt;",">":"&gt;",'"':"&quot;","'":"&#39;"}[c]));}
function buttonState(){
  $("runButton").textContent=sim.running?"Ⅱ PAUSE":"▶ RUN";
  $("runIndicator").textContent=sim.running?"RUNNING · 60 Hz TARGET":"PAUSED";
  $("runIndicator").style.color=sim.running?"#c9f58b":"#e5d4a2";
}
function renderScenarioButtons(){
  $("scenarioButtons").innerHTML=Object.entries(SCENARIOS).map(([k,v])=>
    '<button type="button" class="scenario-btn '+(sim.scenario===k?"active":"")+
    '" data-scenario="'+k+'" aria-pressed="'+(sim.scenario===k)+'" title="'+htmlEsc(v.description)+'">'+
    '<span aria-hidden="true">'+(icons[k]||"◉")+'</span>'+htmlEsc(v.label.split("/")[0])+'</button>'
  ).join("");
  $("scenarioStatus").textContent=SCENARIOS[sim.scenario].label.toUpperCase();
}
function buildSelectors(){
  $("faultSelect").innerHTML=Object.entries(FAULTS).map(([k,v])=>'<option value="'+k+'">'+htmlEsc(v.label)+'</option>').join("");
  $("jointGroup").innerHTML=Object.keys(JOINT_GROUPS).map(g=>'<option>'+htmlEsc(g)+'</option>').join("");
  selectJoint(selectedJoint);
  renderScenarioButtons();
}
function selectJoint(name){
  if(!JOINT_NAMES.includes(name))return;
  selectedJoint=name;
  const group=Object.entries(JOINT_GROUPS).find(([,names])=>names.includes(name))[0];
  $("jointGroup").value=group;
  $("jointSelect").innerHTML=JOINT_GROUPS[group].map(j=>'<option value="'+j+'">'+j+'</option>').join("");
  $("jointSelect").value=name;
  const [min,max]=JOINT_LIMITS[name],grip=name.endsWith("_gripper_aperture");
  $("jointRange").min=grip?0:Math.round(radToDeg(min));
  $("jointRange").max=grip?100:Math.round(radToDeg(max));
  $("jointRange").step=1;
  const value=sim.q[name]||0;
  $("jointRange").value=grip?Math.round(value*100):Math.round(radToDeg(value));
  $("jointValue").textContent=grip?Math.round(value*100)+"%":radToDeg(value).toFixed(0)+"°";
  $("jointLimits").textContent=grip?"0–100%":"LIMIT "+Math.round(radToDeg(min))+"° to "+Math.round(radToDeg(max))+"°";
  $("selectedJointStatus").textContent=name;
  view?.highlightJoint(name);
  updateJointCurrent();
}
function updateJointCurrent(){
  const name=selectedJoint,grip=name.endsWith("_gripper_aperture");
  $("jointCurrent").textContent=grip?Math.round(sim.q[name]*100)+"%":radToDeg(sim.q[name]).toFixed(1)+"°";
}
function updateTelemetry(s){
  $("simTime").textContent=formatSeconds(s.time);
  $("coherenceValue").textContent=s.advisory.coherence.toFixed(3);
  $("innovationValue").textContent=s.advisory.innovation.toFixed(3);
  $("trackingValue").textContent=s.sensor?s.sensor.tracking_error.toFixed(3):"—";
  $("balanceValue").textContent=s.balanceMargin.toFixed(3)+" m*";
  $("contactValue").textContent=Math.round(s.contactProxy*100)+"%";
  $("coherenceBar").style.width=(s.advisory.coherence*100).toFixed(2)+"%";
  $("coherenceStatus").textContent=s.advisory.hold?"HOLD / ADVISORY":s.advisory.status==="UNRESOLVED"?"UNRESOLVED":"SIM / OBSERVED";
  $("coherenceStatus").className="status-chip "+(s.advisory.hold?"alert":s.advisory.status==="UNRESOLVED"?"":"ok");
  $("baselineStatus").textContent=s.instantAlarm?"ALERT":"NO ALARM";
  $("baselineStatus").style.color=s.instantAlarm?"#ff9ca3":"#b8ebcb";
  $("efmwStatus").textContent=s.advisory.status==="UNRESOLVED"?"UNRESOLVED":s.advisory.hold?"HOLD ADVISED":"NO HOLD";
  $("efmwStatus").style.color=s.advisory.hold?"#ff9ca3":"#b8ebcb";
  const messages=[];
  if(!s.sensor)messages.push("Not running: telemetry not initialized");
  else if(s.fault==="stale"&&s.time>=2)messages.push("STALE INPUT — SUPERVISOR UNRESOLVED; advisory hold");
  else if(s.advisory.flags.length)messages.push("Exceeded normalized proxy threshold: "+s.advisory.flags.join(", "));
  else if(s.advisory.hold)messages.push("History/coherence or balance proxy recommends hold");
  else messages.push("No simulated advisory hold; does not certify safety");
  $("monitorReason").textContent=messages.join(". ")+".";
  const featureLabels={
    tracking_error:"TRACKING ERROR",sensor_innovation:"SENSOR INNOVATION",
    contact_disagreement:"CONTACT DISAGREEMENT",thermal_margin_risk:"THERMAL RISK"
  };
  $("sensorReadouts").innerHTML=Object.entries(featureLabels).map(([key,label])=>{
    const val=s.sensor?.[key]??0;
    return '<div class="sensor-row '+(val>.8?"alert":"")+'"><div><span>'+label+'</span><strong>'+Math.round(val*100)+'%</strong></div>'+
      '<div class="sensor-rail"><span style="width:'+Math.round(val*100)+'%"></span></div></div>';
  }).join("");
  updateJointCurrent();
}
function updateChart(history){
  if(!history.length){$("historyPath").setAttribute("d","M0 60H310");$("historyArea").setAttribute("d","");return;}
  const values=history.slice(-120);
  const points=values.map((p,i)=>{
    const x=values.length<=1?0:i*310/(values.length-1);
    const y=clamp(110-p.coherence*100,5,113);
    return [x,y];
  });
  if(points.length===1)points.push([310,points[0][1]]);
  const path="M"+points.map(p=>p[0].toFixed(2)+" "+p[1].toFixed(2)).join("L");
  $("historyPath").setAttribute("d",path);
  $("historyArea").setAttribute("d",path+"L310 118L0 118Z");
}
function refresh(){
  const s=sim.snapshot();
  updateTelemetry(s);updateChart(s.history);
  return s;
}
function wireControls(){
  $("runButton").addEventListener("click",()=>{sim.running=!sim.running;buttonState();});
  $("stepButton").addEventListener("click",()=>{
    sim.running=false;buttonState();
    const s=sim.step(FIXED_STEP);
    view?.update(s);view?.render();refresh();
  });
  $("resetButton").addEventListener("click",()=>{
    sim.reset();renderScenarioButtons();$("faultSelect").value="none";
    $("amplitudeRange").value="65";$("amplitudeValue").textContent="65%";
    $("speedRange").value="100";$("speedValue").textContent="1.00×";
    buttonState();selectJoint(selectedJoint);const s=refresh();view?.update(s);view?.render();
  });
  $("scenarioButtons").addEventListener("click",e=>{
    const b=e.target.closest("[data-scenario]");if(!b)return;
    sim.setScenario(b.dataset.scenario);renderScenarioButtons();
    if(sim.scenario==="manual")selectJoint(selectedJoint);
  });
  $("faultSelect").addEventListener("change",e=>{
    sim.setFault(e.target.value);
    $("faultHelp").textContent=e.target.value==="none"?"Nominal synthetic sensor features; no measured inputs.":"Injecting deterministic "+FAULTS[e.target.value].label.toLowerCase()+" into local testbed only.";
  });
  $("amplitudeRange").addEventListener("input",e=>{sim.setAmplitude(+e.target.value/100);$("amplitudeValue").textContent=e.target.value+"%";});
  $("speedRange").addEventListener("input",e=>{sim.setSpeed(+e.target.value/100);$("speedValue").textContent=(+e.target.value/100).toFixed(2)+"×";});
  $("jointGroup").addEventListener("change",e=>selectJoint(JOINT_GROUPS[e.target.value][0]));
  $("jointSelect").addEventListener("change",e=>selectJoint(e.target.value));
  $("jointRange").addEventListener("input",e=>{
    const grip=selectedJoint.endsWith("_gripper_aperture");
    const val=grip?+e.target.value/100:degToRad(+e.target.value);
    sim.setJointTarget(selectedJoint,val);renderScenarioButtons();
    $("jointValue").textContent=grip?e.target.value+"%":e.target.value+"°";
  });
  document.querySelectorAll("[data-camera]").forEach(b=>b.addEventListener("click",()=>{
    view?.setCamera(b.dataset.camera);
    document.querySelectorAll("[data-camera]").forEach(x=>x.classList.toggle("selected",x===b));
  }));
}
function renderAssembly(){
  if(!assembly)return;
  const ready=assembly.proposal();
  $("assemblyStages").innerHTML=assembly.stages.map(stage=>{
    const seen=assembly.previewed.has(stage.id);
    const isReady=ready?.id===stage.id;
    return '<article class="stage-card '+(seen?"previewed":"")+'">'+
      '<div class="stage-num"><span>'+htmlEsc(stage.id)+'</span><span>'+(seen?"MOCK REVIEWED":isReady?"NEXT PROPOSAL":"BLOCKED")+'</span></div>'+
      '<h3>'+htmlEsc(stage.name)+'</h3>'+
      '<p>DEPENDENCIES: '+htmlEsc(stage.depends_on.length?stage.depends_on.join(" · "):"ROOT")+'</p>'+
      '<span class="stage-mock">REAL: '+htmlEsc(stage.status||"UNRESOLVED")+' / APPROVED: NO</span></article>';
  }).join("");
  if(!ready){
    $("stageProposal").textContent="All stages previewed";
    $("stageEvidence").textContent="No physical stage is approved. Mock previews cannot release hardware work.";
    $("stageBtn").disabled=true;
  }else{
    $("stageProposal").textContent=ready.id+" — "+ready.name;
    $("stageEvidence").textContent="Needed for real gate: "+(ready.required_evidence||[]).join(" · ")+". This app does not generate those approvals.";
    $("stageBtn").disabled=false;
  }
  $("stageEventLog").textContent=assembly.events.length ?
    assembly.events.slice(-9).map((ev,i)=>"["+String(i+1).padStart(2,"0")+"] "+ev.stage+" / SIMULATED PREVIEW / NOT APPROVED").join("\n") :
    "No preview events recorded.";
}
async function initAssembly(){
  try{
    const r=await fetch("./research/config/assembly_graph.json",{cache:"no-store"});
    if(!r.ok)throw Error("HTTP "+r.status);
    const data=await r.json();
    if(!Array.isArray(data.stages)||data.stages.length!==9)throw Error("Unexpected frozen DAG schema");
    assembly=new AssemblyPreview(data.stages);
    renderAssembly();
    $("stageBtn").addEventListener("click",()=>{
      const next=assembly.proposal();if(!next)return;
      assembly.preview(next.id);renderAssembly();
    });
    $("stageResetBtn").addEventListener("click",()=>{
      assembly=new AssemblyPreview(data.stages);renderAssembly();
    });
  }catch(e){
    $("stageProposal").textContent="Frozen DAG unavailable";
    $("stageEvidence").textContent="Load the GitHub Pages site with HTTP(S) to inspect research/config/assembly_graph.json. "+String(e.message);
    $("stageBtn").disabled=true;
  }
}
async function init3D(){
  try{
    const {createRobotScene}=await import("./scene3d.mjs");
    view=createRobotScene($("robotCanvas"),selectJoint);
    const original=new Set(JOINT_NAMES),rendered=new Set(view.jointNames);
    if(view.jointCount!==32||original.size!==rendered.size||[...original].some(n=>!rendered.has(n)))throw Error("3D rig does not match 32-joint schema");
    $("modelStatus").textContent="32 / 32 JOINTS";
    view.highlightJoint(selectedJoint);
    view.update(sim.snapshot());view.render();
  }catch(e){
    $("modelStatus").textContent="WEBGL UNAVAILABLE";
    $("renderError").hidden=false;
    console.warn("3D renderer failed; simulation and research links remain available",e);
  }
}
function mainLoop(now){
  if(prevFrame===null)prevFrame=now;
  const elapsed=Math.min((now-prevFrame)/1000,.1);
  prevFrame=now;
  if(sim.running){
    accumulator+=elapsed;
    let n=0;
    while(accumulator>=FIXED_STEP&&n<6){
      sim.step(FIXED_STEP);
      accumulator-=FIXED_STEP;n++;
    }
    if(n===6)accumulator=0; // shed lag instead of nondeterministic enormous catch-up
  }
  if(view){view.update(sim.snapshot());view.render();}
  if(now-lastPaint>75){refresh();lastPaint=now;}
  window.requestAnimationFrame(mainLoop);
}
async function boot(){
  buildSelectors();wireControls();buttonState();refresh();
  $("seedValue").textContent=String(sim.seed);
  await Promise.all([init3D(),initAssembly()]);
  window.requestAnimationFrame(mainLoop);
}
boot();
