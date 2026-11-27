"""Generate a portable animated preview of actual rendered simulation frames."""
import json
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def make_preview(run):
    run=Path(run)
    frames=[json.loads(line) for line in (run/'telemetry.jsonl').read_text().splitlines()]
    stride=max(1,len(frames)//100)
    previews=[]
    font_path='/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
    try:
        font=ImageFont.truetype(font_path,16)
        title=ImageFont.truetype(font_path,28)
    except OSError:
        font=ImageFont.load_default();title=font
    for frame in frames[::stride]:
        canvas=Image.new('RGB',(960,470),'#101925')
        canvas.paste(Image.open(run/frame['overview']).resize((640,360)),(0,65))
        canvas.paste(Image.open(run/frame['rgb']).resize((320,180)),(640,65))
        draw=ImageDraw.Draw(canvas)
        draw.text((20,15),'ULTRON / SITE SAFETY PATROL',font=title,fill='#8bd4e6')
        draw.text((655,265),frame['mission_id'],font=font,fill='white')
        draw.text((655,295),frame['state'],font=font,fill='#edc06d')
        draw.text((655,325),f"Simulation time: {frame['time']:.1f} s",font=font,fill='white')
        draw.text((655,355),'B2 joint model + A* route',font=font,fill='#8bd4e6')
        draw.text((20,435),'SIMULATION / kinematic motion / oracle perception / human sign-off pending',font=font,fill='#e9c992')
        previews.append(canvas)
    if previews:
        previews[0].save(run/'preview.gif',save_all=True,append_images=previews[1:],duration=220,loop=0,optimize=False)
