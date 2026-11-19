"""Read-only Windows station diagnosis. No camera acquisition or robot commands."""
import argparse
import importlib
import json
import os
from pathlib import Path
import platform
import sys


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    report=dict(python=sys.version,python_executable=sys.executable,platform=platform.platform(),
                dependencies={},monitors=[],cameras=[],serial_ports=[],blockers=[])
    for name in ('cadquery','fcl','cv2','flask','harvesters','serial','screeninfo','tkinter','numpy','scipy'):
        try:
            module=importlib.import_module(name)
            report['dependencies'][name]=dict(ok=True,version=str(getattr(module,'__version__','available')))
        except Exception as error:
            report['dependencies'][name]=dict(ok=False,error=str(error))
            report['blockers'].append('Dependency unavailable: '+name)
    if report['dependencies']['screeninfo']['ok']:
        from screeninfo import get_monitors
        report['monitors']=[dict(x=m.x,y=m.y,width=m.width,height=m.height,primary=m.is_primary,name=m.name)
                            for m in get_monitors()]
        if len(report['monitors'])<2:report['blockers'].append('HDMI projector / extended display not detected')
    if report['dependencies']['serial']['ok']:
        from serial.tools.list_ports import comports
        report['serial_ports']=[dict(port=p.device,description=p.description) for p in comports()]
    cti=os.environ.get('SPECTRUM_GENTL_CTI','')
    report['cti']=cti
    if not cti or not Path(cti).is_file():
        report['blockers'].append('Vendor x64 GenTL CTI is not configured')
    elif report['dependencies']['harvesters']['ok']:
        handles=[]
        try:
            handles.append(os.add_dll_directory(str(Path(cti).parent)))
            for folder in os.environ.get('SPECTRUM_SDK_DLL_DIRS','').split(';'):
                if folder:handles.append(os.add_dll_directory(folder))
            from harvesters.core import Harvester
            with Harvester() as h:
                h.add_file(cti,check_existence=True,check_validity=True);h.update()
                report['cameras']=[dict(model=d.model,serial=d.serial_number,vendor=d.vendor) for d in h.device_info_list]
            if len([d for d in report['cameras'] if d['model']=='VEN-505-36U3C-M01'])!=2:
                report['blockers'].append('Two matching Spectrum VENUS cameras not detected')
        except Exception as error:
            report['blockers'].append('GenTL enumeration failed: '+str(error))
        finally:
            for handle in handles:handle.close()
    if not os.environ.get('SPECTRUM_CAMERA_0_SERIAL') or not os.environ.get('SPECTRUM_CAMERA_1_SERIAL'):
        report['blockers'].append('Camera order/serial numbers are not configured')
    report['software_ready']=all(v['ok'] for v in report['dependencies'].values())
    report['hardware_ready']=not report['blockers']
    path=Path(args.output);path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(report,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
