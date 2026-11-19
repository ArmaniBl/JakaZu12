// Real-browser selection test using Edge/CDP and Node built-ins only.
const {spawn}=require('node:child_process');
const fs=require('node:fs');
const os=require('node:os');
const path=require('node:path');
const assert=require('node:assert/strict');
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
const base=process.argv[2]||'http://127.0.0.1:8090';
const output=process.argv[3]||path.join(os.tmpdir(),'weld-seams.png');
const port=19337;
const profile=fs.mkdtempSync(path.join(os.tmpdir(),'weld-seam-test-'));
const edge=spawn('C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe',[
  '--headless=new','--disable-gpu','--no-first-run','--no-default-browser-check',
  `--remote-debugging-port=${port}`,`--user-data-dir=${profile}`,'about:blank'],
  {windowsHide:true,stdio:'ignore'});
let ws,id=0;const pending=new Map(),errors=[],requests=[];
async function send(method,params={}){
  const request=++id;
  const promise=new Promise((resolve,reject)=>pending.set(request,{resolve,reject}));
  ws.send(JSON.stringify({id:request,method,params}));return promise;
}
async function evaluate(expression){
  const result=await send('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
  if(result.exceptionDetails)throw Error(result.exceptionDetails.text+': '+result.exceptionDetails.exception?.description);
  return result.result.value;
}
async function until(expression){
  for(let n=0;n<100;n++){if(await evaluate(expression))return;await sleep(150);}
  throw Error('Timed out: '+expression+'; errors='+JSON.stringify(errors)+'; requests='+JSON.stringify(requests.slice(-12).map(url=>new URL(url).pathname))+'; page='+JSON.stringify(await evaluate("({url:location.href,ready:document.readyState,title:document.title,app:typeof readSettings,picker:!!document.getElementById('seam-list'),cards:document.querySelectorAll('.seam-card').length,toast:document.querySelector('#toast')?.textContent,connected:typeof serverConnected==='undefined'?null:serverConnected})")));
}
(async()=>{
  let target;
  for(let n=0;n<80;n++){
    try{target=await (await fetch(`http://127.0.0.1:${port}/json/new?about:blank`,{method:'PUT'})).json();break;}catch{await sleep(100);}
  }
  assert(target,'Edge remote debugging did not start');
  ws=new WebSocket(target.webSocketDebuggerUrl);
  await new Promise((resolve,reject)=>{ws.onopen=resolve;ws.onerror=reject;});
  ws.onmessage=event=>{
    const message=JSON.parse(event.data);
    if(message.id){const task=pending.get(message.id);pending.delete(message.id);message.error?task.reject(Error(JSON.stringify(message.error))):task.resolve(message.result);}
    if(message.method==='Runtime.exceptionThrown')errors.push(message.params.exceptionDetails.exception?.description||message.params.exceptionDetails.text);
    if(message.method==='Network.requestWillBeSent')requests.push(message.params.request.url);
  };
  await send('Runtime.enable');await send('Page.enable');await send('Network.enable');
  await send('Emulation.setDeviceMetricsOverride',{width:1500,height:1200,deviceScaleFactor:1,mobile:false});
  await send('Page.navigate',{url:base});
  await until("document.querySelectorAll('.seam-card').length>0");
  await until("document.getElementById('scan-last-update')!==null");
  assert(await evaluate("(()=>{const old=state;state={...state,scan_progress:{state:'CAPTURING',observed_unix:Date.now()/1000,completed_frames:29,total_frames:41,percent:71,started_unix:Date.now()/1000-60}};renderScanProgress();const ok=document.getElementById('scan-progress').value===71&&document.getElementById('scan-phase').textContent.includes('29/41');state={...state,scan_progress:{...state.scan_progress,state:'DECODING'}};renderScanProgress();const decoding=!document.getElementById('scan-progress').hasAttribute('value');state={...state,scan_progress:{state:'UNAVAILABLE',observed_unix:Date.now()/1000}};renderScanProgress();const offline=document.getElementById('scan-progress').hidden;state=old;renderScanProgress();return ok&&decoding&&offline;})()"),'Capture progress, indeterminate decoding and unavailable state must render');
  assert(await evaluate("!document.querySelector('#scan-info,#scan-detect,#scan-apply,#scan-tcp')"),'Old scanner controls must be absent');
  assert(await evaluate("(()=>{const old=state;state={...state,scan_updated_unix:1,health:{...state.health,isaac:true,feedback:true,moveit:true,controller:true,objects:[{id:'welding_workpiece',shape:'mesh',dimensions:[]}]}};updateControls();renderScanTime();const enabled=!document.getElementById('execute').disabled;state=old;updateControls();return enabled;})()"),'Scan age must not gate scene-based execution');
  const seams=await evaluate("Array.from(document.querySelectorAll('.seam-card')).map(b=>({id:b.dataset.seam,label:b.textContent}))");
  let selectedId;
  if(seams.some(s=>s.id==='seam_center')){
    assert.equal(seams.length,1);
    assert.match(seams[0].label,/Центральный стыковой шов/);
    await evaluate("document.querySelector('[data-seam=seam_center]').click()");
    await until("geometry.selected_face==='seam_center'");
    assert(await evaluate("geometry.seam.length>100 && Math.hypot(...geometry.seam[0].map((v,i)=>v-geometry.seam.at(-1)[i]))>.1"),'The center weld must be an open CAD path');
    await evaluate("document.querySelector('#begin').value=25;document.querySelector('#begin').dispatchEvent(new Event('input',{bubbles:true}))");
    await until('readSettings().begin===.25 && geometry.length>.106 && geometry.length<.109');
    selectedId='seam_center';
  }else{
  assert.equal(seams.length,3);
  const slot=seams.find(s=>s.label.includes('прорези')),hole=seams.find(s=>s.label.includes('отверстия'));
  assert(slot&&hole,'Slot and hole must be individual selectable seams');
  await evaluate(`document.querySelector('[data-seam="${slot.id}"]').click()`);
  await until(`geometry.selected_face==='${slot.id}'`);
  assert.equal(await evaluate("document.querySelector('#face').value"),slot.id);
  assert.match(await evaluate("document.querySelector('#selection-title').textContent"),/прорези/);
  await evaluate(`document.querySelector('[data-seam-hit="${hole.id}"]').dispatchEvent(new MouseEvent('click',{bubbles:true}))`);
  await until(`geometry.selected_face==='${hole.id}'`);
  assert.match(await evaluate("document.querySelector('#selection-title').textContent"),/отверстия/);
  await evaluate("document.querySelector('#begin').value=25;document.querySelector('#begin').dispatchEvent(new Event('input',{bubbles:true}))");
  await until('readSettings().begin===.25 && geometry.length>.012 && geometry.length<.016');
  assert(await evaluate("document.querySelectorAll('path[fill-rule=evenodd]').length>0"),'CAD holes need even-odd surface rendering');
  selectedId=hole.id;
  }
  assert(!requests.some(url=>/\/api\/(start|workpiece)$/.test(url)),'Selecting a seam must not start motion or replace CAD');
  assert.deepEqual(errors,[]);
  const screenshot=await send('Page.captureScreenshot',{format:'png',captureBeyondViewport:true,clip:{x:0,y:450,width:1500,height:1200,scale:1}});
  fs.writeFileSync(output,Buffer.from(screenshot.data,'base64'));
  console.log(JSON.stringify({result:'PASS',seams,selected:selectedId,screenshot:output,robotCommands:0}));
})().catch(error=>{console.error(error.stack);process.exitCode=1;}).finally(async()=>{
  if(ws&&ws.readyState===WebSocket.OPEN){try{await send('Browser.close');}catch{}ws.close();}
  edge.kill();
});
