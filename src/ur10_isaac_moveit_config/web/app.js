'use strict';
const $ = (id) => document.getElementById(id);
const defaults = {object:'welding_workpiece',selection:'face',face:'z_plus',edge:'x_plus__z_plus',begin:0,end:1,offset:.03,clearance:.05,step:.005,speed:.02,scaling:.05,roll:0};
const faceNames = {z_plus:'Верхняя грань · Z+',z_minus:'Нижняя грань · Z−',x_plus:'Ближняя грань · X+',x_minus:'Дальняя грань · X−',y_plus:'Боковая грань · Y+',y_minus:'Боковая грань · Y−'};
const axisName = (s) => s.replace('_plus','+').replace('_minus','−').toUpperCase();
const faceLabel = (id) => faceNames[id] || `Грань ${id}`;
const robotName = () => state?.robot_profile?.real ? 'JAKA ZU12' : 'UR10';
let selection='face', token='', lastSequence=0, state=null, geometry=null, geometryTimer=null, geometryVersion=0;
let busy=false, polling=false, logLines=0, toastTimer=null, objectSignature='', serverConnected=false;
let yaw=.75, pitch=.48, zoom=1, dragging=null, dragMoved=false;
let sceneFraming=null, robotAvailable=false, lastRobotSignature='';
const SVG_NS='http://www.w3.org/2000/svg';

function notify(message,error=false){
  $('toast').textContent=message;$('toast').classList.toggle('error',error);$('toast').hidden=false;
  clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').hidden=true,error?8000:4000);
}
async function api(path,data){
  const response=await fetch(path,data===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json','X-Weld-Token':token},body:JSON.stringify(data)});
  const result=await response.json();if(!response.ok)throw new Error(result.error||`Ошибка ${response.status}`);return result;
}
function readSettings(){
  return {object:$('object').value,selection,face:$('face').value,edge:$('edge').value,
    begin:Number($('begin-number').value)/100,end:Number($('end-number').value)/100,
    speed:Number($('speed').value)/1000,offset:Number($('offset').value)/1000,
    clearance:Number($('clearance').value)/1000,step:Number($('step').value)/1000,
    scaling:Number($('scaling').value)/100,roll:Number($('roll').value)};
}
function validSettings(){const s=readSettings();return $('settings-form').checkValidity()&&s.begin<s.end&&s.begin>=0&&s.end<=1;}
function applySettings(settings){
  const s={...defaults,...settings};selection=s.selection==='edge'?'edge':'face';
  for(const key of ['face','edge']){
    $(key).value=s[key];
    if(!$(key).value)$(key).value=$(key).options[0]?.value||defaults[key];
  }
  for(const key of ['begin','end']){$(key).value=s[key]*100;$(key+'-number').value=s[key]*100;}
  for(const key of ['speed','offset','clearance','step'])$(key).value=Math.round(s[key]*1000*100)/100;
  $('scaling').value=s.scaling*100;$('roll').value=s.roll;
  if([...$('object').options].some(o=>o.value===s.object))$('object').value=s.object;
  changed();
}
function changed(){
  $('mode-face').classList.toggle('selected',selection==='face');$('mode-edge').classList.toggle('selected',selection==='edge');
  $('face').hidden=selection!=='face';$('edge').hidden=selection!=='edge';
  const s=readSettings();$('scaling-value').textContent=`${$('scaling').value}%`;
  const highSpeed=s.scaling>0.5;
  $('scaling-warning').hidden=!highSpeed;
  $('scaling-value').classList.toggle('high-speed',highSpeed);
  $('range-fill').style.left=`${Math.min(100,Math.max(0,s.begin*100))}%`;
  $('range-fill').style.width=`${Math.max(0,(s.end-s.begin)*100)}%`;
  $('selection-title').textContent=selection==='face'?faceLabel(s.face):`Ребро ${s.edge.split('__').map(axisName).join(' / ')}`;
  try{localStorage.setItem('weld-studio-settings',JSON.stringify(s));}catch{}
  if(geometry)draw();
  clearTimeout(geometryTimer);geometryTimer=setTimeout(refreshGeometry,130);updateControls();
}
async function refreshGeometry(){
  if(!token||!validSettings())return;
  const version=++geometryVersion;
  try{const result=await api('/api/geometry',readSettings());if(version!==geometryVersion)return;
      const oldShape=geometry?JSON.stringify(geometry.faces.map(f=>f.vertices)):null;
      geometry=result;if(oldShape!==JSON.stringify(result.faces.map(f=>f.vertices)))sceneFraming=null;
      const options=(id,items,value)=>{$(id).replaceChildren(...items.map(([key,label])=>Object.assign(document.createElement('option'),{value:key,textContent:label})));$(id).value=value;};
      options('face',result.faces.map(f=>[f.id,faceLabel(f.id)]),result.selected_face||readSettings().face);
      options('edge',result.edges.map(e=>[e.id,e.id]),result.selected_edge||readSettings().edge);
      $('selection-title').textContent=selection==='face'?faceLabel($('face').value):`Ребро ${$('edge').value}`;
      const mm=Math.round(result.length*1000).toLocaleString('ru-RU');
    $('path-length').replaceChildren(document.createTextNode(mm+' '),Object.assign(document.createElement('span'),{textContent:'мм'}));
    $('range-summary').textContent=`${Math.round((readSettings().end-readSettings().begin)*100)}% ${selection==='face'?'периметра':'ребра'} · ${mm} мм`;
    $('model-source').textContent=result.demo?'Образец · сцена ещё не подключена':'Геометрия из сцены MoveIt';
    $('scene-view-label').textContent=result.demo?'ОБРАЗЕЦ':'LIVE GEOMETRY';draw();
  }catch(error){notify(error.message,true);}
}
function svg(tag,attrs={},parent){const node=document.createElementNS(SVG_NS,tag);for(const [key,value]of Object.entries(attrs))node.setAttribute(key,String(value));if(parent)parent.append(node);return node;}
function draw(){
  if(!geometry)return;
  const root=$('scene-svg');root.replaceChildren();
  const points=geometry.faces.flatMap(f=>f.vertices);
  const objectCenter=[0,1,2].map(i=>(Math.min(...points.map(p=>p[i]))+Math.max(...points.map(p=>p[i])))/2);
  const size=[0,1,2].map(i=>Math.max(...points.map(p=>p[i]))-Math.min(...points.map(p=>p[i])));
  const robot=serverConnected&&state?.health?.robot?.available?state.health.robot:null;
  const showRobot=$('show-robot').checked&&robot&&!geometry.demo;
  const robotPoints=showRobot?robot.points:[];
  const framingPoints=[...points,...robotPoints.map(p=>p.position),...geometry.tool];
  let low=[0,1,2].map(i=>Math.min(...framingPoints.map(p=>p[i]))-.12);
  let high=[0,1,2].map(i=>Math.max(...framingPoints.map(p=>p[i]))+.12);
  if(!sceneFraming||low.some((v,i)=>v<sceneFraming.low[i]-.05*sceneFraming.extent)||high.some((v,i)=>v>sceneFraming.high[i]+.05*sceneFraming.extent)){
    if(sceneFraming){low=low.map((v,i)=>Math.min(v,sceneFraming.low[i]));high=high.map((v,i)=>Math.max(v,sceneFraming.high[i]));}
    sceneFraming={low,high,center:low.map((v,i)=>(v+high[i])/2),extent:Math.max(...low.map((v,i)=>high[i]-v),.3)};
  }
  const {center,extent}=sceneFraming,scale=360/extent*zoom;
  $('robot-status').textContent=robot?(geometry.demo?`Поза ${robotName()} получена · ожидаем деталь в MoveIt`:`${robotName()} · текущая поза из ROS`):serverConnected?(state?.health?.robot?.reason||'Ожидаем состояние робота'):'Нет связи с панелью';
  $('robot-status').classList.toggle('live',Boolean(robot));
  const base=robot?.points[0],tip=robot?.points[robot.points.length-1];
  $('robot-position').textContent=base&&!geometry.demo?`База → центр детали: ${objectCenter.map((v,i)=>`${'XYZ'[i]} ${(v-base.position[i]).toFixed(3)}`).join('  ')} м`:'Положение робота появится после подключения ROS-сцены';
  $('robot-tool-position').textContent=tip?`${state?.robot_profile?.tip||'tool0'}: ${tip.position.map((v,i)=>`${'XYZ'[i]} ${v.toFixed(3)}`).join('  ')} м`:'';
  function camera(p){const x=p[0]-center[0],y=p[1]-center[1],z=p[2]-center[2];const u=Math.cos(yaw)*x-Math.sin(yaw)*y,v=Math.sin(yaw)*x+Math.cos(yaw)*y;return [u,Math.sin(pitch)*v-Math.cos(pitch)*z,Math.cos(pitch)*v+Math.sin(pitch)*z];}
  const project=p=>{const q=camera(p);return [400+q[0]*scale,285+q[1]*scale];};
  const projectString=p=>project(p).map(n=>n.toFixed(2)).join(',');
  const defs=svg('defs',{},root),glow=svg('radialGradient',{id:'ground-shadow'},defs);
  svg('stop',{offset:'0%','stop-color':'#060c14','stop-opacity':'.7'},glow);svg('stop',{offset:'100%','stop-color':'#060c14','stop-opacity':'0'},glow);
  const metal=svg('linearGradient',{id:'robot-metal',x1:'0%',y1:'0%',x2:'100%',y2:'100%'},defs);
  for(const [offset,color]of [['0%','#7e91a5'],['45%','#d6e0e8'],['100%','#647b91']])svg('stop',{offset,'stop-color':color},metal);
  svg('ellipse',{cx:400,cy:410,rx:230*zoom,ry:75*zoom,fill:'url(#ground-shadow)'},root);
  const grid=svg('g',{stroke:'#3a5872','stroke-opacity':'.24','stroke-width':'.7'},root);
  const ground=Math.min(objectCenter[2]-size[2]/2,base?.position[2]??Infinity)-.018;
  for(let i=-7;i<=7;i++){
    const delta=i*extent/5;
    const a=[[center[0]-extent*1.4,center[1]+delta,ground],[center[0]+extent*1.4,center[1]+delta,ground]];
    const b=[[center[0]+delta,center[1]-extent*1.4,ground],[center[0]+delta,center[1]+extent*1.4,ground]];
    for(const pair of [a,b])svg('polyline',{points:pair.map(projectString).join(' '),fill:'none'},grid);
  }
  const s=readSettings(),selectedFaces=selection==='face'?[s.face]:(geometry.edges.find(e=>e.id===s.edge)?.faces||[]);
  const light=[-.3,-.2,1];
  const drawables=[];
  for(const face of geometry.faces){
    drawables.push({depth:face.vertices.reduce((v,p)=>v+camera(p)[2],0)/face.vertices.length,render:()=>{
    const selected=selectedFaces.includes(face.id);
    const shade=.45+.4*Math.max(0,face.normal.reduce((v,n,i)=>v+n*light[i],0));
    const fill=selected?`rgb(${Math.round(160*shade+30)},${Math.round(104*shade+22)},${Math.round(58*shade+14)})`:`rgb(${Math.round(52*shade+26)},${Math.round(70*shade+30)},${Math.round(94*shade+34)})`;
    const polygon=svg('polygon',{points:face.vertices.map(projectString).join(' '),fill,stroke:selected?'#d49b65':'#6b829b','stroke-width':selected?'1.5':'1','stroke-linejoin':'round',tabindex:selection==='face'?'0':'-1',role:'button','aria-label':faceLabel(face.id)},root);
    const pick=()=>{if(dragMoved||isLocked()||selection!=='face')return;$('face').value=face.id;changed();};
    polygon.addEventListener('click',pick);polygon.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();pick();}});
    svg('title',{},polygon).textContent=faceLabel(face.id);
    }});
  }
  if(showRobot){
    // Link origins come from TF. Capsules are a schematic, not collision meshes.
    for(let i=1;i<robotPoints.length;i++){
      const a=robotPoints[i-1].position,b=robotPoints[i].position;
      const width=Math.max(5,(i<5?.105:.075)*scale);
      drawables.push({depth:(camera(a)[2]+camera(b)[2])/2,render:()=>{
        const group=svg('g',{'pointer-events':'none','class':'robot-link','data-link':robotPoints[i].frame},root);
        for(const [stroke,w]of [['#162331',width+4],['url(#robot-metal)',width]])svg('polyline',{points:[a,b].map(projectString).join(' '),fill:'none',stroke,'stroke-width':w,'stroke-linecap':'round'},group);
      }});
    }
    robotPoints.slice(0,-1).forEach((point,i)=>drawables.push({depth:camera(point.position)[2]+.001,render:()=>{
      const [cx,cy]=project(point.position),radius=Math.max(4,(i<5?.067:.048)*scale);
      const group=svg('g',{'pointer-events':'none','class':'robot-joint','data-frame':point.frame},root);
      svg('circle',{cx,cy,r:radius,fill:i===0?'#263c50':'#82b7d2',stroke:'#14293c','stroke-width':2},group);
      svg('circle',{cx:cx-radius*.12,cy:cy-radius*.12,r:radius*.57,fill:i===0?'#657d93':'#5294b9',stroke:'#b6d7e7','stroke-width':1},group);
    }}));
    const b=base.position,baseRing=Array.from({length:33},(_,i)=>[b[0]+.12*Math.cos(i*Math.PI/16),b[1]+.12*Math.sin(i*Math.PI/16),b[2]]);
    svg('polyline',{points:baseRing.map(projectString).join(' '),fill:'#24384b',stroke:'#79b6d6','stroke-width':1.5,'pointer-events':'none'},root);
  }
  drawables.sort((a,b)=>a.depth-b.depth).forEach(item=>item.render());
  if(showRobot){
    const axes=svg('g',{'pointer-events':'none','class':'robot-reference'},root);
    const label=(point,text,color)=>{const [x,y]=project(point);svg('text',{x:x+10,y:y-12,fill:color,'font-size':11,'font-family':'Segoe UI, sans-serif','paint-order':'stroke',stroke:'#0c1420','stroke-width':3},axes).textContent=text;};
    label(base.position,`БАЗА ${robotName()}`,'#99c9e1');label(tip.position,state?.robot_profile?.tip||'tool0','#8ce8c9');
    const a=[base.position[0],base.position[1],ground+.003],b=[objectCenter[0],objectCenter[1],ground+.003];
    svg('polyline',{points:[a,b].map(projectString).join(' '),fill:'none',stroke:'#85b6d2','stroke-width':1.2,'stroke-dasharray':'4 5','opacity':.7},axes);
    const q=tip.orientation,[x,y,z,w]=q,n=x*x+y*y+z*z+w*w;
    const basis=[[1-2*(y*y+z*z)/n,2*(x*y+z*w)/n,2*(x*z-y*w)/n],
      [2*(x*y-z*w)/n,1-2*(x*x+z*z)/n,2*(y*z+x*w)/n],
      [2*(x*z+y*w)/n,2*(y*z-x*w)/n,1-2*(x*x+y*y)/n]];
    basis.forEach((v,i)=>svg('polyline',{points:[tip.position,tip.position.map((p,j)=>p+.1*v[j])].map(projectString).join(' '),fill:'none',stroke:['#ef7b7b','#8ed3a1','#82aefa'][i],'stroke-width':2},axes));
  }
  for(const edge of geometry.edges){
    const selected=selection==='edge'&&edge.id===s.edge;
    svg('polyline',{points:(edge.vertices||[edge.start,edge.end]).map(projectString).join(' '),fill:'none',stroke:selected?'#ffc58a':'#7a91aa','stroke-width':selected?'4':'1','stroke-opacity':selected?'1':'.35'},root);
    if(selection==='edge'){
      const hit=svg('polyline',{points:(edge.vertices||[edge.start,edge.end]).map(projectString).join(' '),fill:'none',stroke:'transparent','stroke-width':'18',tabindex:'0',role:'button','aria-label':'Ребро '+edge.id},root);
      const pick=()=>{if(dragMoved||isLocked())return;$('edge').value=edge.id;changed();};
      hit.addEventListener('click',pick);hit.addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();pick();}});svg('title',{},hit).textContent=edge.id;
    }
  }
  svg('polyline',{points:geometry.seam.map(projectString).join(' '),fill:'none',stroke:'#ffc58a','stroke-width':'3','stroke-linecap':'round','stroke-linejoin':'round','pointer-events':'none'},root);
  svg('polyline',{points:geometry.tool.map(projectString).join(' '),fill:'none',stroke:'#79e5c5','stroke-width':'2.5','stroke-dasharray':'7 5','stroke-linejoin':'round','pointer-events':'none'},root);
  const first=project(geometry.tool[0]);svg('circle',{cx:first[0],cy:first[1],r:5,fill:'#76e6c2',stroke:'#143d34','stroke-width':'3','pointer-events':'none'},root);
  const end=project(geometry.tool[geometry.tool.length-1]);svg('circle',{cx:end[0],cy:end[1],r:3,fill:'#d4ffee','pointer-events':'none'},root);
  const firstLabel=svg('text',{x:first[0]+12,y:first[1]-10,fill:'#95c7ba','font-size':'10','font-family':'Segoe UI, sans-serif','pointer-events':'none'},root);firstLabel.textContent='СТАРТ';
  const caption=svg('text',{x:400,y:502,fill:'#647f98','font-size':'10','text-anchor':'middle','font-family':'Segoe UI, sans-serif','letter-spacing':'1','pointer-events':'none'},root);
  caption.textContent=`${size.map(v=>Math.round(v*1000)).join(' × ')} ММ`;
  const origin=svg('text',{x:400,y:523,fill:'#425d76','font-size':'9','text-anchor':'middle','font-family':'Consolas, monospace','pointer-events':'none'},root);
  origin.textContent=`ЦЕНТР ДЕТАЛИ · WORLD  X ${objectCenter[0].toFixed(2)}  Y ${objectCenter[1].toFixed(2)}  Z ${objectCenter[2].toFixed(2)}`;
}
function isRunning(){return state?.job?.state==='running';}
function isLocked(){return isRunning()&&state.job.mode!=='preview';}
function updateControls(){
  const h=state?.health||{},objectReady=(h.objects||[]).some(o=>o.id===$('object').value);
  const real=Boolean(state?.robot_profile?.real);
  document.title=`Weld Studio · ${robotName()}`;
  $('robot-mode').textContent=real?'REAL':'SIM';
  $('robot-name').textContent=robotName();
  $('robot-tip').textContent=`6 осей · ${state?.robot_profile?.tip||'tool0'}`;
  $('source-name').textContent=real?'JAKA SDK':'Isaac Sim';
  $('controller-name').textContent=`Контроллер ${robotName()}`;
  $('return-note').textContent=real?'После прохода — возврат в позу начала операции или заданную конфигурацию.':'После прохода — возврат в сохранённую позу.';
  const canPreview=serverConnected&&h.moveit&&objectReady&&!isLocked()&&!busy&&validSettings();
  $('preview').disabled=!canPreview;$('plan').disabled=!(canPreview&&h.isaac&&h.feedback);$('execute').disabled=!(canPreview&&h.isaac&&h.feedback&&h.controller);
  $('stop').disabled=!isRunning()||Boolean(state?.job?.stopping)||busy;
  $('scene-start').disabled=real||busy||Boolean(state?.scene?.owned)||Boolean(h.moveit);
  $('scene-start').textContent=real?'JAKA: отдельный запуск run_real.sh':state?.scene?.owned?(h.moveit?'Сцена запущена':'Запуск сцены…'):h.moveit?'Подключено к сцене':'＋ Запустить сцену с кубом';
  $('scene-stop').hidden=!state?.scene?.owned;$('scene-stop').disabled=isRunning()||busy||state?.scene?.stopping;
  for(const element of $('settings-form').querySelectorAll('input,select,button'))element.disabled=isLocked();
  $('workpiece-shape').disabled=$('workpiece-apply').disabled=!serverConnected||!h.moveit||isLocked()||busy;
  $('workpiece-apply').textContent=busy?'Подождите…':'Применить в RViz';
  const currentObject=(h.objects||[]).find(o=>o.id===$('object').value);
  if(currentObject&&!busy&&currentObject.shape){
    $('workpiece-shape').value=currentObject.shape==='mesh'?'prism':currentObject.shape;
    $('object-shape').textContent=currentObject.shape.toUpperCase();
  }
  $('reset').disabled=isLocked();
  const healthy=[h.isaac&&h.feedback,h.moveit,h.controller,objectReady];
  for(const [i,name]of ['isaac','moveit','controller','object'].entries()){$('health-'+name).textContent=healthy[i]?'✓':'○';$('health-'+name).classList.toggle('good',Boolean(healthy[i]));$('dot-'+name).classList.toggle('good',Boolean(healthy[i]));}
  $('health-count').textContent=`${healthy.filter(Boolean).length} / 4`;
  $('health-object-label').textContent=objectReady?'Объект загружен в MoveIt':'Ожидает загрузки';
  const ready=healthy.every(Boolean)&&serverConnected;
  $('connection').classList.toggle('online',serverConnected);$('connection').replaceChildren(Object.assign(document.createElement('i')),document.createTextNode(serverConnected?'Локальная связь':'Нет связи с панелью'));
  $('ready-dot').classList.toggle('good',ready&&!isRunning());
  $('ready-title').textContent=!serverConnected?'Нет связи с сервером':isRunning()?state.job.phase:!validSettings()?'Проверьте настройки':ready?'Система готова к работе':h.moveit?'Ожидание робота или заготовки':'Ожидание сцены';
  $('ready-description').textContent=isRunning()?'Состояние и журнал обновляются автоматически':ready?'Предпросмотр → планирование → выполнение':real?'Запустите драйвер JAKA и добавьте заготовку в MoveIt':'Запустите сцену и включите Play в Isaac Sim';
  $('connection-hint').textContent=ready?'Все компоненты подключены. Можно построить план.':real?'Ожидаем драйвер JAKA, состояния joint_1…joint_6 и заготовку в MoveIt.':h.moveit&&!h.isaac?'Включите Play в Isaac Sim для получения состояний суставов.':'Запустите сцену и включите воспроизведение в Isaac Sim.';
}
function renderJob(){
  const job=state?.job,active=isRunning();
  $('operation-type').textContent=job?{preview:'ПРЕДПРОСМОТР',plan:'ПЛАНИРОВАНИЕ',execute:'ВЫПОЛНЕНИЕ'}[job.mode]:'ОЖИДАНИЕ';
  $('operation-phase').textContent=job?job.phase:'Всё начинается с выбора участка';
  $('operation-light').classList.toggle('active',active);$('activity-track').classList.toggle('running',active);$('activity-track').classList.toggle('done',job?.state==='succeeded');
  $('operation-subtitle').textContent=job?.stopping?'Запрошена отмена':active?'Процесс выполняется':job?.state==='failed'?'Смотрите причину в журнале':job?.state==='cancelled'?'Операция отменена':job?'Операция завершена':'Процесс не запущен';
  $('report-link').hidden=!job?.report;if(job?.report){$('report-link').href='/api/report/'+job.id;$('report-link').download='welding-'+job.id+'.json';}
  $('step-select').classList.toggle('active',!active||job?.mode==='preview');$('step-plan').classList.toggle('active',active&&job?.mode==='plan');$('step-execute').classList.toggle('active',active&&job?.mode==='execute');
  const list=$('history-list');list.replaceChildren();
  for(const item of state?.history||[]){const row=document.createElement('div');row.className='history-row';const text=document.createElement('div');text.textContent={preview:'Предпросмотр',plan:'План',execute:'Выполнение'}[item.mode];const detail=document.createElement('small');detail.textContent=`${new Date(item.started*1000).toLocaleTimeString('ru-RU')} · ${item.phase}`;text.append(detail);row.append(text);if(item.report){const a=document.createElement('a');a.href='/api/report/'+item.id;a.download='welding-'+item.id+'.json';a.textContent='↓ JSON';row.append(a);}list.append(row);}
}
function addLogs(entries){
  if(!entries.length)return;const log=$('logs');log.querySelector('.log-empty')?.remove();
  for(const entry of entries){const row=document.createElement('div');row.className='log-row'+(/ERROR|failed|Ошибка|Traceback/i.test(entry.text)?' error':/WARN/i.test(entry.text)?' warning':'');
    for(const [cls,text]of [['log-time',entry.time],['log-source',entry.source],['log-text',entry.text]]){const cell=document.createElement('span');cell.className=cls;cell.textContent=text;row.append(cell);}log.append(row);logLines++;}
  while(log.children.length>700)log.firstChild.remove();$('log-count').textContent=logLines;if($('follow-log').checked)log.scrollTop=log.scrollHeight;
}
async function poll(){
  if(polling)return;polling=true;
  try{const data=await api('/api/state?since='+lastSequence);const oldJob=state?.job;state=data;token=data.token;serverConnected=true;
    const available=Boolean(data.health.robot?.available);
    if(available!==robotAvailable){sceneFraming=null;robotAvailable=available;}
    const robotSignature=JSON.stringify(data.health.robot||null);
    if(robotSignature!==lastRobotSignature){lastRobotSignature=robotSignature;if(geometry)draw();}
    addLogs(data.logs);lastSequence=data.sequence;
    const sig=JSON.stringify(data.health.objects);
    if(sig!==objectSignature||!geometry){objectSignature=sig;const previous=$('object').value;const objects=data.health.objects;
      if(objects.length){$('object').replaceChildren(...objects.map(o=>{const option=document.createElement('option');option.value=o.id;option.textContent=`${o.id} · ${{box:'Параллелепипед',cylinder:'Цилиндр',mesh:'Mesh'}[o.shape]||'BOX'}${o.dimensions.length?' · '+o.dimensions.map(v=>Math.round(v*1000)).join(' × ')+' мм':''}`;return option;}));if(objects.some(o=>o.id===previous))$('object').value=previous;}
      $('object-id').textContent=$('object').value;refreshGeometry();}
    if(oldJob?.state==='running'&&data.job?.id===oldJob.id&&data.job.state!=='running')notify(data.job.state==='failed'?(data.job.error||'Операция завершилась с ошибкой'):'Операция завершена',data.job.state==='failed');
    renderJob();updateControls();
  }catch{serverConnected=false;if(geometry)draw();updateControls();}finally{polling=false;}
}
async function start(mode){
  if(!validSettings()){notify('Проверьте значения параметров и начало / конец участка',true);return;}
  busy=true;updateControls();
  try{await api('/api/start',{mode,settings:readSettings()});await poll();}catch(error){notify(error.message,true);}finally{busy=false;updateControls();}
}
async function sceneAction(action){busy=true;updateControls();try{await api('/api/scene',{action,rviz:$('rviz').checked});await poll();}catch(error){notify(error.message,true);}finally{busy=false;updateControls();}}

async function applyWorkpiece(){
  if(busy||isLocked())return;
  const shape=$('workpiece-shape').value,object=$('object').value;
  busy=true;updateControls();
  $('workpiece-hint').textContent='Обновляем заготовку в сцене MoveIt/RViz…';
  try{
    await api('/api/workpiece',{object,shape});
    geometry=null;objectSignature=null;sceneFraming=null;
    await poll();
    notify('Заготовка заменена в RViz. Выберите грань или ребро.');
  }catch(error){notify(error.message,true);}
  finally{
    busy=false;
    $('workpiece-hint').textContent='Выбор формы заменяет текущую деталь в MoveIt/RViz. Центр: X −1 · Y 0 · Z 0,1 м.';
    updateControls();
  }
}
$('workpiece-shape').addEventListener('change',applyWorkpiece);
$('workpiece-apply').onclick=applyWorkpiece;

for(const a of ['minus','plus'])for(const b of ['minus','plus'])for(const pair of [['x','y'],['x','z'],['y','z']]){const id=`${pair[0]}_${a}__${pair[1]}_${b}`,option=document.createElement('option');option.value=id;option.textContent=`Ребро ${axisName(pair[0]+'_'+a)} / ${axisName(pair[1]+'_'+b)}`;$('edge').append(option);}
$('settings-form').addEventListener('submit',e=>e.preventDefault());
for(const id of ['speed','offset','clearance','step','scaling','roll','face','edge','object'])$(id).addEventListener('input',changed);
for(const key of ['begin','end']){for(const suffix of ['','-number'])$(key+suffix).addEventListener('input',()=>{$(key+(suffix?'':'-number')).value=$(key+suffix).value;changed();});}
$('mode-face').onclick=()=>{selection='face';changed();};$('mode-edge').onclick=()=>{selection='edge';changed();};
$('full-range').onclick=()=>{for(const id of ['begin','begin-number'])$(id).value=0;for(const id of ['end','end-number'])$(id).value=100;changed();};
$('reset').onclick=()=>applySettings(defaults);
$('preview').onclick=()=>start('preview');$('plan').onclick=()=>start('plan');$('execute').onclick=()=>start('execute');
$('stop').onclick=async()=>{busy=true;updateControls();try{await api('/api/stop',{});await poll();}catch(error){notify(error.message,true);}finally{busy=false;updateControls();}};
$('scene-start').onclick=()=>sceneAction('start');$('scene-stop').onclick=()=>sceneAction('stop');
$('history-toggle').onclick=()=>{$('history-panel').hidden=!$('history-panel').hidden;};
$('clear-log').onclick=()=>{$('logs').replaceChildren();logLines=0;$('log-count').textContent='0';};
$('export-settings').onclick=()=>{const blob=new Blob([JSON.stringify(readSettings(),null,2)],{type:'application/json'}),url=URL.createObjectURL(blob),a=document.createElement('a');a.href=url;a.download='welding-settings.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);};
document.querySelectorAll('[data-view]').forEach(button=>button.onclick=()=>{document.querySelectorAll('[data-view]').forEach(b=>b.classList.toggle('selected',b===button));yaw=button.dataset.view==='top'?0:.75;pitch=button.dataset.view==='top'?Math.PI/2:.48;zoom=1;draw();});
$('fit-view').onclick=()=>{yaw=.75;pitch=.48;zoom=1;sceneFraming=null;draw();};
$('show-robot').addEventListener('change',()=>{zoom=1;sceneFraming=null;draw();});
$('scene-svg').addEventListener('pointerdown',event=>{dragging={x:event.clientX,y:event.clientY,yaw,pitch};dragMoved=false;});
window.addEventListener('pointermove',event=>{if(!dragging)return;const dx=event.clientX-dragging.x,dy=event.clientY-dragging.y;if(Math.abs(dx)+Math.abs(dy)<5&&!dragMoved)return;dragMoved=true;$('scene-svg').classList.add('dragging');yaw=dragging.yaw+dx*.009;pitch=Math.min(1.55,Math.max(-.7,dragging.pitch+dy*.008));draw();});
window.addEventListener('pointerup',()=>{dragging=null;$('scene-svg').classList.remove('dragging');setTimeout(()=>dragMoved=false,0);});
$('scene-svg').addEventListener('wheel',event=>{event.preventDefault();zoom=Math.max(.45,Math.min(1.7,zoom*(event.deltaY>0?.92:1.08)));draw();},{passive:false});
setInterval(()=>{const job=state?.job;const seconds=job?Math.max(0,Math.floor(((job.finished||Date.now()/1000)-job.started))):0;$('elapsed').textContent=`${String(Math.floor(seconds/60)).padStart(2,'0')}:${String(seconds%60).padStart(2,'0')}`;},1000);
let saved={};try{saved=JSON.parse(localStorage.getItem('weld-studio-settings')||'{}');}catch{}
applySettings(saved);poll();setInterval(poll,250);
