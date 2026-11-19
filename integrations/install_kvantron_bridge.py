#!/usr/bin/env python3
"""Install the read-only endpoint into an existing kVantron scanner checkout."""
import argparse
from pathlib import Path
import shutil

parser = argparse.ArgumentParser()
parser.add_argument('scanner_root', type=Path)
args = parser.parse_args()
target = args.scanner_root/'scanner_app/spectrum.py'
source = target.read_text(encoding='utf-8')
marker = "    @bp.get('/api/spectrum/state')"
endpoint = '''    @bp.get('/api/spectrum/moveit-export')
    def moveit_export():
        from .moveit_export import export_bundle
        from adaptive_weld.cell_calibration import reference_frame
        with guard:
            if busy():raise ValueError('Завершите сканирование/операцию перед экспортом в MoveIt')
            if not fresh_capture():raise ValueError('Нужен новый реальный скан текущего задания')
            part=task_part()
            context=bp.calibration_cloud()
            if context['unit']=='grid_pitch' and not setup_api.public_state().get('ready'):
                raise ValueError('Сначала примите измеренную калибровку сканера')
            frame,matrix,reusable=reference_frame(context)
            return jsonify(export_bundle(context,part,frame,matrix,reusable,active_task['id'],active_task['capture_id']))

'''
if "'/api/spectrum/moveit-export'" not in source:
    if source.count(marker) != 1:
        raise SystemExit('Unsupported scanner source: expected unique state endpoint')
    backup = target.with_suffix('.py.before-moveit')
    if not backup.exists():
        shutil.copy2(target, backup)
    target.write_text(source.replace(marker, endpoint+marker), encoding='utf-8')
shutil.copy2(Path(__file__).with_name('kvantron_export.py'), target.with_name('moveit_export.py'))
# Windows readers can briefly hold the projector command without delete sharing.
capture = args.scanner_root/'сканер/scripts/structured_capture.py'
capture_text = capture.read_text(encoding='utf-8')
old_command = "temporary=sequence/'command.tmp';temporary.write_text(str(i));temporary.replace(sequence/'command')"
new_command = "temporary=sequence/'command.tmp';temporary.write_text(str(i));replace_when_available(temporary,sequence/'command')"
if old_command in capture_text:
    backup = capture.with_suffix('.py.before-command-retry')
    if not backup.exists():shutil.copy2(capture,backup)
    capture_text = capture_text.replace('import argparse\n','import argparse\nfrom scanner_atomic import replace_when_available\n',1)
    capture.write_text(capture_text.replace(old_command,new_command),encoding='utf-8')
elif new_command not in capture_text:
    raise SystemExit('Unsupported projector command writer: expected unique command replacement')
shutil.copy2(Path(__file__).with_name('scanner_atomic.py'),capture.with_name('scanner_atomic.py'))
capture_text=capture.read_text(encoding='utf-8')
capture_text=capture_text.replace('from scanner_atomic import replace_when_available\n',
                                'from scanner_atomic import replace_when_available, select_projector\n')
old_display="    display = next((d for d in displays if d['id']==args.display),None)\n    if not display or display['builtin'] or display['main'] or display['mirrors']:\n        raise ValueError('Choose an independent external projector')"
new_display="    display = select_projector(displays,args.display)\n    args.display = display['id']"
if old_display in capture_text:capture_text=capture_text.replace(old_display,new_display)
elif new_display not in capture_text:raise SystemExit('Unsupported projector selection block')
capture.write_text(capture_text,encoding='utf-8')
capture_text=capture.read_text(encoding='utf-8')
capture_text=capture_text.replace('from scanner_atomic import replace_when_available, select_projector\n',
    'from scanner_atomic import replace_when_available, select_projector, read_text_when_available\n')
capture_text=capture_text.replace("(sequence/'ready').read_text()","read_text_when_available(sequence/'ready')")
capture.write_text(capture_text,encoding='utf-8')
hardware=capture.with_name('windows_hardware.py')
hardware_text=hardware.read_text(encoding='utf-8')
if 'from scanner_atomic import replace_when_available' not in hardware_text:
    hardware_text=hardware_text.replace('from contextlib import ExitStack\n',
        'from contextlib import ExitStack\nfrom scanner_atomic import replace_when_available, read_text_when_available\n',1)
hardware_text=hardware_text.replace("temporary.replace(directory/'ready')","replace_when_available(temporary,directory/'ready')")
hardware_text=hardware_text.replace("(directory/'command').read_text()","read_text_when_available(directory/'command')")
hardware.write_text(hardware_text,encoding='utf-8')
capture_text=capture.read_text(encoding='utf-8')
exposure_import='from scanner_atomic import capture_exposure\n'
if exposure_import not in capture_text:
    capture_text=exposure_import+capture_text
exposure_hook="    args.exposure = capture_exposure(args.exposure, ROOT.parent/'runtime/scanner/capture-settings.json')\n"
if exposure_hook not in capture_text:
    marker='def capture(args):\n'
    if capture_text.count(marker)!=1:raise SystemExit('Unsupported capture entry point')
    capture_text=capture_text.replace(marker,marker+exposure_hook)
capture.write_text(capture_text,encoding='utf-8')
reconstruct=capture.with_name('dense_reconstruct.py')
reconstruct_text=reconstruct.read_text(encoding='utf-8')
refinement_hook='''    preview_refinement = None
    preview_refinement_error = None
    preview_settings = ROOT.parent/'runtime/scanner/capture-settings.json'
    if (os.environ.get('KVANTRON_MOVEIT_BRIDGE') == '1' and preview_settings.is_file()
            and json.loads(preview_settings.read_text(encoding='utf-8')).get('preview_stereo_refinement') is True
            and response > .2):
        from stereo_refinement import triangulate, refine_preview_rotation
        _, raw_positive, raw_error = triangulate(pixels, fit['cameras'], rotation, translation)
        if np.mean(raw_positive & (raw_error <= 2.)) < .5:
            try:
                rotation, preview_refinement = refine_preview_rotation(pixels, fit['cameras'], rotation, translation)
            except ValueError as error:
                preview_refinement_error = str(error)
'''
if '    preview_refinement = None\n' not in reconstruct_text:
    marker="    rotation,translation=np.array(fit['stereo_R']),np.array(fit['stereo_T_grid_units'])\n"
    if reconstruct_text.count(marker)!=1:raise SystemExit('Unsupported stereo reconstruction block')
    reconstruct_text=reconstruct_text.replace(marker,marker+refinement_hook)
    marker="    (folder/'reconstruction.json').write_text(json.dumps(report,indent=2)+'\\n')"
    if reconstruct_text.count(marker)!=1:raise SystemExit('Unsupported reconstruction report block')
    reconstruct_text=reconstruct_text.replace(marker,
        "    report.update(preview_stereo_refinement=preview_refinement, preview_refinement_error=preview_refinement_error)\n"+marker)
reconstruct.write_text(reconstruct_text,encoding='utf-8')
shutil.copy2(Path(__file__).with_name('stereo_refinement.py'),reconstruct.with_name('stereo_refinement.py'))
text=target.read_text(encoding='utf-8')
text=text.replace("'--display','3','--exposure'","'--display',capture_env.get('SPECTRUM_PROJECTOR_DISPLAY','0'),'--exposure'")
target.write_text(text,encoding='utf-8')
# Route UI invalidation must not cancel an unrelated ongoing scan/decode.
text=target.read_text(encoding='utf-8')
old='            cancel_requested.set();access_report=None;access_signature=None;route_generation+=1'
new="            if job.get('state')=='PLANNING':cancel_requested.set()\n            access_report=None;access_signature=None;route_generation+=1"
if old in text:
    target.write_text(text.replace(old,new),encoding='utf-8')
app = args.scanner_root/'scanner_app/app.py'
text = app.read_text(encoding='utf-8')
marker = 'def local_access():\n'
guard = '''    # MoveIt owns robot control in the integrated scanner session.
    if os.environ.get('KVANTRON_MOVEIT_BRIDGE')=='1' and request.method=='POST':
        if ((request.path.startswith('/api/jaka/') and request.path not in ('/api/jaka/stop','/api/jaka/disconnect'))
                or request.path in ('/api/workflow/start','/api/workflow/prepare','/api/workflow/heartbeat')):
            return jsonify(error='В объединённом режиме роботом управляет только MoveIt'),409
'''
if "os.environ.get('KVANTRON_MOVEIT_BRIDGE')" not in text:
    if text.count(marker)!=1:raise SystemExit('Unsupported scanner app: unique local_access guard required')
    backup=app.with_suffix('.py.before-moveit')
    if not backup.exists():shutil.copy2(app,backup)
    app.write_text(text.replace(marker,marker+guard),encoding='utf-8')
(args.scanner_root/'04-Scan-For-MoveIt.cmd').write_text(
    '@echo off\nsetlocal\ncd /d "%~dp0"\nset "KVANTRON_MOVEIT_BRIDGE=1"\ncall 02-Start.cmd\n',encoding='ascii')
print('Installed GET /api/spectrum/moveit-export. Restart the scanner service.')
