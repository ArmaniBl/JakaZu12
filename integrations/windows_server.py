"""Foreground loopback scanner service for the prepared MoveIt station."""
import argparse
import os
from pathlib import Path
import re
import sys
import webbrowser

# UTF-8 must be selected at interpreter startup, not after file I/O begins.
if not sys.flags.utf8_mode:
    os.execve(sys.executable,[sys.executable,'-X','utf8',*sys.argv],dict(os.environ,PYTHONUTF8='1'))

parser=argparse.ArgumentParser()
parser.add_argument('--scanner-root',required=True)
parser.add_argument('--open-browser',action='store_true')
args=parser.parse_args()
root=Path(args.scanner_root).resolve()
sys.path.insert(0,str(root))
os.chdir(root)
config=root/'config.cmd'
if config.exists():
    for line in config.read_text(encoding='utf-8-sig').splitlines():
        match=re.fullmatch(r'\s*@?set\s+"(SPECTRUM_[A-Z0-9_]+)=(.*)"\s*',line,re.IGNORECASE)
        if match:os.environ[match[1].upper()]=os.path.expandvars(match[2])
os.environ.update(SCANNER_ROOT=str(root),SCANNER_RUNTIME=str(root/'runtime/scanner'),
                  SCANNER_INITIAL_SCENE='empty',KVANTRON_MOVEIT_BRIDGE='1',PYTHONUTF8='1')
from scanner_app.app import app
from werkzeug.serving import make_server
print('Scanner for MoveIt: http://127.0.0.1:8158/; Ctrl+C to stop.',flush=True)
try:
    server=make_server('127.0.0.1',8158,app,threaded=True)
    if args.open_browser:webbrowser.open('http://127.0.0.1:8158/')
    server.serve_forever()
except KeyboardInterrupt:
    pass
