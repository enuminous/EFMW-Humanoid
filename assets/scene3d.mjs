/* Three.js/WebGL demonstration renderer for EHP-3D-SIM-001.
   Visual rigid-link model; not calibrated CAD, FEA, MJCF or full dynamics. */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { JOINT_NAMES } from "./sim-core.mjs";

const COLORS={
  bg:0x07111c,steel:0x94a8b7,steelDark:0x263948,body:0x344b5c,bodyLight:0x496579,
  teal:0x7bf1df,blue:0x81bfff,lime:0xd0ff85,rose:0xff8c97,ground:0x102332,
  joint:0x162832
};
function material(color,metalness=.35,roughness=.35,emissive=null){
  return new THREE.MeshStandardMaterial({color,metalness,roughness,emissive:emissive||0x000000,emissiveIntensity:emissive?.23:0});
}
export function createRobotScene(canvas,onSelectJoint=()=>{}){
  if(!canvas)throw Error("A canvas element is required");
  const renderer=new THREE.WebGLRenderer({canvas,antialias:true,alpha:true,powerPreference:"high-performance"});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
  renderer.shadowMap.enabled=true;
  renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  renderer.toneMapping=THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure=1.17;
  renderer.outputColorSpace=THREE.SRGBColorSpace;

  const scene=new THREE.Scene();
  scene.background=new THREE.Color(COLORS.bg);
  scene.fog=new THREE.Fog(COLORS.bg,6,13);
  const camera=new THREE.PerspectiveCamera(40,1,.04,80);
  camera.position.set(2.15,1.88,3.55);
  const orbit=new OrbitControls(camera,renderer.domElement);
  orbit.target.set(0,.88,0);orbit.enableDamping=true;orbit.dampingFactor=.055;
  orbit.maxPolarAngle=Math.PI*.49;orbit.minDistance=1.18;orbit.maxDistance=7;
  orbit.enablePan=false;
  const hemis=new THREE.HemisphereLight(0xa6eaff,0x07101b,2.1);
  scene.add(hemis);
  const key=new THREE.DirectionalLight(0xb9fff5,3.5);key.position.set(-2,5,3);key.castShadow=true;
  key.shadow.mapSize.set(2048,2048);key.shadow.camera.left=-3;key.shadow.camera.right=3;
  key.shadow.camera.top=3;key.shadow.camera.bottom=-3;
  key.shadow.bias=-.00025;scene.add(key);
  const rim=new THREE.DirectionalLight(0x7180ff,2.5);rim.position.set(1.8,2,-2.5);scene.add(rim);
  const warm=new THREE.PointLight(0xd1ff8f,11,5);warm.position.set(0,2.5,-2.2);scene.add(warm);
  const groundMat=new THREE.MeshStandardMaterial({color:COLORS.ground,metalness:.35,roughness:.61});
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(40,40),groundMat);
  ground.rotation.x=-Math.PI/2;ground.position.y=-.105;ground.receiveShadow=true;scene.add(ground);
  const grid=new THREE.GridHelper(14,70,0x4b8496,0x1d3a4b);
  grid.position.y=-.095;
  grid.material.transparent=true;grid.material.opacity=.24;
  scene.add(grid);
  const platform=new THREE.Mesh(new THREE.CylinderGeometry(.88,.95,.06,96),material(0x172e3b,.65,.4));
  platform.position.y=-.074;platform.receiveShadow=true;scene.add(platform);
  function torus(radius,color,opacity,thickness=.004){
    const mesh=new THREE.Mesh(new THREE.TorusGeometry(radius,thickness,8,128),
      new THREE.MeshBasicMaterial({color,transparent:true,opacity}));
    mesh.rotation.x=-Math.PI/2;mesh.position.y=-.037;scene.add(mesh);return mesh;
  }
  torus(.80,COLORS.teal,.43,.007);torus(.94,COLORS.blue,.20,.002);
  const pulseRing=torus(.60,COLORS.lime,.20,.003);
  for(let i=0;i<24;i++){
    const ang=i*2*Math.PI/24;
    const tick=new THREE.Mesh(new THREE.BoxGeometry(.033,.004,.006),new THREE.MeshBasicMaterial({color:COLORS.teal,transparent:true,opacity:.64}));
    tick.position.set(.835*Math.cos(ang),-.036,.835*Math.sin(ang));
    tick.rotation.y=-ang;scene.add(tick);
  }
  const shadowBlob=new THREE.Mesh(new THREE.CircleGeometry(.35,48),new THREE.MeshBasicMaterial({color:0x020a13,transparent:true,opacity:.19,depthWrite:false}));
  shadowBlob.rotation.x=-Math.PI/2;shadowBlob.position.y=-.033;scene.add(shadowBlob);
  const steel=material(COLORS.steel,.68,.26),dark=material(COLORS.steelDark,.60,.36);
  const armor=material(COLORS.body,.58,.30),armorLight=material(COLORS.bodyLight,.55,.31);
  const cyan=material(COLORS.teal,.26,.28,COLORS.teal),blue=material(COLORS.blue,.4,.3,COLORS.blue);
  const jointMat=material(COLORS.joint,.52,.25),black=material(0x0d1c27,.25,.35);
  const accent=new THREE.MeshBasicMaterial({color:COLORS.lime});
  const glass=new THREE.MeshPhysicalMaterial({color:0x569fc9,metalness:.15,roughness:.12,transparent:true,opacity:.75,clearcoat:1});
  const robot=new THREE.Group();robot.position.set(0,.81,0);scene.add(robot);
  const jointNodes=new Map(),pickables=[],jointMarkers=new Map(),sideLights=[];
  function shell(parent,size,pos,mat=armor,corner=.0){
    const mesh=new THREE.Mesh(new THREE.BoxGeometry(...size),mat);
    mesh.position.set(...pos);mesh.castShadow=true;mesh.receiveShadow=true;parent.add(mesh);return mesh;
  }
  function ball(parent,r,pos,mat=steel){
    const m=new THREE.Mesh(new THREE.SphereGeometry(r,20,14),mat);
    m.position.set(...pos);m.castShadow=true;parent.add(m);return m;
  }
  function rod(parent,a,b,r=.055,mat=armor){
    const p1=new THREE.Vector3(...a),p2=new THREE.Vector3(...b),dir=p2.clone().sub(p1);
    const geo=new THREE.CylinderGeometry(r,r*.9,dir.length(),14);
    const mesh=new THREE.Mesh(geo,mat);
    mesh.position.copy(p1.add(p2).multiplyScalar(.5));
    mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),dir.normalize());
    mesh.castShadow=true;parent.add(mesh);return mesh;
  }
  function rotary(parent,name,where,axis="x",size=.066){
    const p=new THREE.Group();
    p.position.set(...where);parent.add(p);
    jointNodes.set(name,{group:p,axis});
    const marker=ball(p,size,[0,0,0],jointMat);
    marker.userData.jointName=name;pickables.push(marker);
    const rimMesh=new THREE.Mesh(new THREE.TorusGeometry(size*.78,.008,7,25),new THREE.MeshBasicMaterial({color:COLORS.teal,transparent:true,opacity:.70}));
    rimMesh.rotation[axis==="x"?"y":axis==="y"?"x":"x"]=axis==="y"?-Math.PI/2:0;
    p.add(rimMesh);
    jointMarkers.set(name,rimMesh);
    return p;
  }
  function rail(parent,pos,len,side){
    const a=side==="left"?COLORS.teal:COLORS.blue;
    const m=new THREE.Mesh(new THREE.BoxGeometry(.014,len,.017),new THREE.MeshBasicMaterial({color:a}));
    m.position.set(...pos);parent.add(m);return m;
  }

  // Pelvis and torso / floating-base representation.
  shell(robot,[.36,.15,.19],[0,.012,0],armor);
  shell(robot,[.28,.07,.20],[0,-.07,0],dark);
  const waistYaw=rotary(robot,"waist_yaw",[0,.10,0],"y",.085);
  const waistPitch=rotary(waistYaw,"waist_pitch",[0,0,0],"x",.075);
  shell(waistPitch,[.43,.40,.19],[0,.22,0],armorLight);
  shell(waistPitch,[.47,.17,.22],[0,.34,0],armor);
  shell(waistPitch,[.34,.18,.235],[0,.19,-.004],dark);
  shell(waistPitch,[.12,.06,.032],[0,.205,.124],cyan);
  // Emissive vertical system lines and visible 'sensor' pods.
  rail(waistPitch,[-.185,.19,.115],.24,"left");rail(waistPitch,[.185,.19,.115],.24,"right");
  for(const side of [-1,1]){
    ball(waistPitch,.027,[side*.17,.305,.13],cyan);
    shell(waistPitch,[.085,.055,.12],[side*.195,.37,.00],armorLight);
  }

  function arm(side,sign){
    const prefix=side+"_";
    const sh1=rotary(waistPitch,prefix+"shoulder_pitch",[sign*.255,.385,0],"x",.087);
    const sh2=rotary(sh1,prefix+"shoulder_roll",[0,0,0],"z",.076);
    const sh3=rotary(sh2,prefix+"shoulder_yaw",[0,0,0],"y",.064);
    shell(sh3,[.115,.165,.126],[sign*.013,-.10,0],armorLight);
    rod(sh3,[0,-.11,0],[0,-.275,0],.052,dark);
    shell(sh3,[.125,.185,.130],[0,-.20,0],armor);
    rail(sh3,[sign*.059,-.195,.065],.13,side);
    const elbow=rotary(sh3,prefix+"elbow_pitch",[0,-.298,0],"x",.069);
    rod(elbow,[0,-.015,0],[0,-.215,0],.053,armorLight);
    shell(elbow,[.105,.15,.112],[0,-.132,0],armor);
    const forearm=rotary(elbow,prefix+"forearm_roll",[0,-.23,0],"y",.058);
    const wristPitch=rotary(forearm,prefix+"wrist_pitch",[0,-.023,0],"x",.049);
    const wristYaw=rotary(wristPitch,prefix+"wrist_yaw",[0,-.027,0],"y",.044);
    shell(wristYaw,[.113,.073,.098],[0,-.075,0],dark);
    const grip=rotary(wristYaw,prefix+"gripper_aperture",[0,-.111,0],"y",.032);
    const fingers=[];
    for(const d of [-1,1])fingers.push(shell(grip,[.025,.080,.035],[d*.041,-.160,.00],steel));
    jointNodes.get(prefix+"gripper_aperture").fingers=fingers;
  }
  arm("left",-1);arm("right",1);

  function leg(side,sign){
    const p=side+"_";
    const hipYaw=rotary(robot,p+"hip_yaw",[sign*.118,-.062,0],"y",.079);
    const hipRoll=rotary(hipYaw,p+"hip_roll",[0,0,0],"z",.075);
    const hipPitch=rotary(hipRoll,p+"hip_pitch",[0,0,0],"x",.074);
    shell(hipPitch,[.162,.23,.165],[0,-.149,0],armorLight);
    rod(hipPitch,[0,-.10,0],[0,-.365,0],.073,dark);
    rail(hipPitch,[sign*.085,-.24,.076],.23,side);
    const knee=rotary(hipPitch,p+"knee_pitch",[0,-.382,0],"x",.079);
    shell(knee,[.155,.18,.148],[0,-.115,0],armor);
    rod(knee,[0,-.11,0],[0,-.318,0],.063,steel);
    const anklePitch=rotary(knee,p+"ankle_pitch",[0,-.337,0],"x",.060);
    const ankleRoll=rotary(anklePitch,p+"ankle_roll",[0,-.043,0],"z",.048);
    shell(ankleRoll,[.185,.072,.33],[0,-.038,.080],dark);
    shell(ankleRoll,[.13,.028,.17],[0,-.075,.125],armorLight);
    const light=ball(ankleRoll,.026,[0,-.025,.238],cyan);
    sideLights.push(light);
  }
  leg("left",-1);leg("right",1);

  // Neck and head: sensor housing, binocular optical ports, top beacon.
  const neckYaw=rotary(waistPitch,"neck_yaw",[0,.465,0],"y",.055);
  const neckPitch=rotary(neckYaw,"neck_pitch",[0,.045,0],"x",.05);
  shell(neckPitch,[.192,.177,.150],[0,.13,0],armorLight);
  shell(neckPitch,[.183,.076,.156],[0,.214,-.001],dark);
  shell(neckPitch,[.163,.063,.025],[0,.135,.084],glass);
  for(const s of [-1,1]){
    ball(neckPitch,.018,[s*.054,.134,.098],cyan);
  }
  ball(neckPitch,.019,[0,.253,0],blue);

  // High-contrast torso label generated locally, not a fetched image.
  const labelCanvas=document.createElement("canvas");
  labelCanvas.width=384;labelCanvas.height=128;
  const ctx=labelCanvas.getContext("2d");
  if(ctx){
    ctx.clearRect(0,0,384,128);
    ctx.font="900 76px system-ui, sans-serif";ctx.textAlign="center";ctx.fillStyle="#d5fff0";
    ctx.fillText("EFMW",192,83);
    ctx.font="bold 19px monospace";ctx.fillStyle="#84c7c2";ctx.fillText("HUMANOID / 001",192,114);
    const texture=new THREE.CanvasTexture(labelCanvas);texture.colorSpace=THREE.SRGBColorSpace;
    const logo=new THREE.Mesh(new THREE.PlaneGeometry(.244,.085),new THREE.MeshBasicMaterial({map:texture,transparent:true,depthWrite:false}));
    logo.position.set(0,.265,.131);waistPitch.add(logo);
  }

  // Subtle physically-labelled support proxy; these are not real ground forces.
  const support=new THREE.Mesh(new THREE.PlaneGeometry(.45,.34),
    new THREE.MeshBasicMaterial({color:COLORS.teal,transparent:true,opacity:.11,side:THREE.DoubleSide,depthWrite:false}));
  support.rotation.x=-Math.PI/2;support.position.y=-.031;scene.add(support);
  const supportEdge=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.PlaneGeometry(.45,.34)),
    new THREE.LineBasicMaterial({color:COLORS.teal,transparent:true,opacity:.42}));
  supportEdge.rotation.x=-Math.PI/2;supportEdge.position.y=-.030;scene.add(supportEdge);
  const comMarker=ball(scene,.040,[0,1.12,0],accent);
  const comLineMat=new THREE.LineDashedMaterial({color:COLORS.lime,dashSize:.035,gapSize:.025,transparent:true,opacity:.48});
  const comGeom=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,-.025,0),new THREE.Vector3(0,1.12,0)]);
  const comLine=new THREE.Line(comGeom,comLineMat);comLine.computeLineDistances();scene.add(comLine);

  // Selectable joints, with highlight and physical joint name rendered in DOM UI.
  const ray=new THREE.Raycaster(),cursor=new THREE.Vector2(),cameraHome=camera.position.clone();
  let selected="right_elbow_pitch";
  function highlightJoint(name){
    if(!jointNodes.has(name))return false;
    selected=name;
    for(const [n,ring] of jointMarkers){
      ring.material.color.setHex(n===name?COLORS.lime:COLORS.teal);
      ring.material.opacity=n===name?1:.68;
    }
    return true;
  }
  highlightJoint(selected);
  canvas.addEventListener("pointerdown",e=>{
    if(e.button!==0)return;
    const rect=canvas.getBoundingClientRect();
    cursor.x=2*(e.clientX-rect.left)/rect.width-1;
    cursor.y=1-2*(e.clientY-rect.top)/rect.height;
    ray.setFromCamera(cursor,camera);
    const hit=ray.intersectObjects(pickables,false)[0];
    if(hit?.object?.userData?.jointName){
      const name=hit.object.userData.jointName;
      highlightJoint(name);onSelectJoint(name);
    }
  });
  function setCamera(preset){
    const target={
      front:[0,1.27,3.2],side:[3.2,1.32,0],top:[2.2,3.5,2.05],iso:[2.15,1.88,3.55]
    }[preset];
    if(!target)return;
    camera.position.set(...target);orbit.target.set(0,.85,0);orbit.update();
  }
  const tmp1=new THREE.Vector3(),tmp2=new THREE.Vector3();
  function update(snapshot){
    for(const [name,node] of jointNodes){
      const val=snapshot.joints[name]||0;
      if(name.endsWith("gripper_aperture")){
        if(node.fingers){
          node.fingers[0].position.x=-.025-.046*val;
          node.fingers[1].position.x=.025+.046*val;
        }
      }else node.group.rotation[node.axis]=val;
    }
    const w=snapshot.scenario==="walk"?.009*Math.cos(snapshot.time*3.7):0;
    robot.position.y=.81+w;
    robot.rotation.y=.025*Math.sin(snapshot.time*.23);
    comMarker.position.set(snapshot.comX,1.07+w,0);
    const xyz=comLine.geometry.attributes.position;
    xyz.setXYZ(0,snapshot.comX,-.018,0);
    xyz.setXYZ(1,snapshot.comX,1.07+w,0);
    xyz.needsUpdate=true;
    comLine.computeLineDistances();
    const hazard=snapshot.advisory.hold;
    support.material.color.setHex(hazard?COLORS.rose:COLORS.teal);
    supportEdge.material.color.setHex(hazard?COLORS.rose:COLORS.teal);
    comMarker.material.color.setHex(hazard?COLORS.rose:COLORS.lime);
    pulseRing.material.opacity=.17+.10*(.5+.5*Math.sin(snapshot.time*2.6));
    sideLights.forEach(x=>x.material.emissiveIntensity=hazard?.35:.27);
  }
  function resize(){
    const w=Math.max(320,canvas.clientWidth||720);
    const h=Math.max(320,canvas.clientHeight||610);
    camera.aspect=w/h;camera.updateProjectionMatrix();renderer.setSize(w,h,false);
  }
  if("ResizeObserver" in window){
    const observer=new ResizeObserver(resize);observer.observe(canvas);
  }else window.addEventListener("resize",resize);
  resize();
  function render(){orbit.update();renderer.render(scene,camera);}
  function dispose(){orbit.dispose();renderer.dispose();}
  return {scene,camera,renderer,update,render,resize,setCamera,highlightJoint,dispose,
    get jointCount(){return jointNodes.size;},get jointNames(){return [...jointNodes.keys()];}};
}
