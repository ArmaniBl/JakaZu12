"""Print target matching setup_fit's six-marker orientation signature."""
import argparse
from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm

LARGE={(1,2),(1,3),(3,1),(3,3),(3,6),(4,3)}
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
args.output.parent.mkdir(parents=True,exist_ok=True)
c=canvas.Canvas(str(args.output),pagesize=(210*mm,297*mm))
c.setTitle('Calibration target 7x7 - pitch 20mm - print actual size')
for row in range(7):
    for col in range(7):
        c.circle((45+col*20)*mm,(210-row*20)*mm,(5 if (row,col) in LARGE else 2.5)*mm,stroke=0,fill=1)
c.showPage()
c.setFont('Helvetica-Bold',14)
c.drawString(20*mm,275*mm,'PRINT PAGE 1 AT ACTUAL SIZE / 100%')
c.setFont('Helvetica',11)
for i,line in enumerate([
    'No fit-to-page, scaling, stretching or borderless enlargement.',
    'Grid: 7 x 7 centres. Nominal pitch: 20 mm in both directions.',
    'First-to-last centre distance: 120 mm horizontally and vertically.',
    'Small circles: diameter 5 mm. Six large circles: diameter 10 mm.',
    'Large circles, row/column from upper left, counting from 1:',
    '(2,3), (2,4), (4,2), (4,4), (4,7), (5,4).',
    'Mount on a rigid, flat, matte board. Do not bend the paper.',
    'Measure centre spans on several rows and columns after printing.',
    'Measured pitch = first-to-last centre span / 6.',
    'If X/Y scale differs or spacing is uneven, reject the target.',
    'This print is not a certified metrology artefact.',
    'Validate scale/pose on an independent measured object before robot motion.',
]):c.drawString(20*mm,(250-i*10)*mm,line)
c.save()
print(args.output)
