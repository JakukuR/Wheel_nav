#!/usr/bin/env python3
"""Export sparse indexed Image samples to full-size lossless grayscale PNG."""
import argparse
from bisect import bisect_left
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import cv2
from check_recording_quality import load_image,TOPICS


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('quality_directory',type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--period',type=float,default=2.)
    args=parser.parse_args()
    if args.period<=0:parser.error('period must be positive')
    root=args.quality_directory;out=args.output
    out.mkdir(parents=True,exist_ok=False)
    indexed=json.loads((root/'image_index.json').read_text())
    report=json.loads((root/'report.json').read_text())
    stamps={eye:[r['stamp'] for r in indexed[eye]] for eye in TOPICS}
    start=max(v[0] for v in stamps.values());end=min(v[-1] for v in stamps.values())
    rows=[];groups=[];used={}
    for target in range(start+500_000_000,end,int(args.period*1e9)):
        group={}
        for eye in TOPICS:
            values=stamps[eye];i=bisect_left(values,target)
            i=min((j for j in (i-1,i) if 0<=j<len(values)),key=lambda j:abs(values[j]-target))
            key=(eye,i)
            if key not in used:
                row=dict(indexed[eye][i],eye=eye,index=i)
                used[key]=f"{eye}_{row['stamp']}.png";rows.append(row)
            group[eye]=used[key]
        groups.append(group)
    cv2.setNumThreads(1)
    def export(row):
        image=cv2.cvtColor(load_image(row),cv2.COLOR_BGR2GRAY)
        name=used[(row['eye'],row['index'])]
        if not cv2.imwrite(str(out/name),image,[cv2.IMWRITE_PNG_COMPRESSION,2]):
            raise RuntimeError('PNG write failed')
        return dict(file=name,eye=row['eye'],stamp=row['stamp'],index=row['index'],
            width=image.shape[1],height=image.shape[0],grayscale_sha256=hashlib.sha256(image.tobytes()).hexdigest())
    with ThreadPoolExecutor(max_workers=2) as pool:
        frames=list(pool.map(export,rows))
    manifest=dict(frames=frames,groups=groups,start_stamp=start,period_s=args.period,
        conversion='BGR8 -> grayscale with OpenCV; full size, no crop/resize; PNG lossless',source_bag=report['bag'])
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps({'frames':len(frames),'png_bytes':sum(p.stat().st_size for p in out.glob('*.png'))}),flush=True)


if __name__=='__main__':main()
