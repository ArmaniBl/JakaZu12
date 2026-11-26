'use strict';
// Informational only: neither scan age nor availability controls job readiness.
const scanStamp=document.createElement('div');scanStamp.className='scan-update';
scanStamp.innerHTML='<span>Последнее обновление скана</span><time id="scan-last-update">Ещё не обновлялся</time>';
document.querySelector('.viewport-panel .panel-heading').after(scanStamp);
const scanProgress=document.createElement('div');scanProgress.className='scan-progress';
scanProgress.innerHTML='<div><span id="scan-phase">Ожидание сканера</span><span id="scan-duration"></span></div><progress id="scan-progress" max="100" aria-label="Прогресс сканирования"></progress>';
scanStamp.after(scanProgress);
function renderScanProgress(){
  const data=state?.scan_progress,bar=$('scan-progress'),label=$('scan-phase'),duration=$('scan-duration');
  updateScannerView();
  bar.removeAttribute('value');
  if(!scannerAvailable){bar.hidden=true;return;}
  const phase=data.state;
  const labels={PAUSED_MOTION:'Пауза до конца операции робота',WAITING:'Ожидание сканирования',CAPTURING:'Захват кадров',DECODING:'Восстановление облака',COMPLETE:'Сканирование завершено',COMPLETED:'Сканирование завершено',FAILED:'Ошибка сканирования',CANCELLED:'Сканирование отменено',PLANNING:'Сканер занят: планирование',COMPARING:'Сканер занят: сравнение'};
  label.textContent=labels[phase]||'Сканер готов';
  bar.hidden=!['CAPTURING','DECODING'].includes(phase);
  if(phase==='CAPTURING'&&Number.isFinite(data.percent)){
    bar.value=data.percent;label.textContent+=` · ${data.completed_frames}/${data.total_frames} · ${data.percent}%`;
  }
  duration.textContent='';
  if(Number.isFinite(data.started_unix)){
    const end=Number.isFinite(data.finished_unix)?data.finished_unix:Date.now()/1000;
    const seconds=Math.max(0,Math.floor(end-data.started_unix));
    duration.textContent=`${Math.floor(seconds/60)} мин ${String(seconds%60).padStart(2,'0')} с`;
  }
}
function renderScanTime(){
  const timestamp=state?.scan_updated_unix,element=$('scan-last-update');
  if(typeof timestamp!=='number'||!Number.isFinite(timestamp)||timestamp<=0){element.textContent='Ещё не обновлялся';element.removeAttribute('datetime');return;}
  const date=new Date(timestamp*1000),age=Math.max(0,Math.floor(Date.now()/1000-timestamp));
  const elapsed=age<60?`${age} с назад`:age<3600?`${Math.floor(age/60)} мин назад`:age<86400?`${Math.floor(age/3600)} ч назад`:`${Math.floor(age/86400)} дн. назад`;
  const clock=date.toLocaleString('ru-RU',{timeZone:'Europe/Moscow',day:'2-digit',month:'2-digit',hour:'2-digit',minute:'2-digit',second:'2-digit'});
  element.textContent=`${clock} МСК · ${elapsed}`;element.dateTime=date.toISOString();
}
function renderScanner(){
  renderScanProgress();
  if(scannerAvailable)renderScanTime();
}
renderScanner();
setInterval(renderScanner,1000);
