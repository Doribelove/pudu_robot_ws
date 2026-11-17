# SPDX-License-Identifier: Apache-2.0
from pathlib import Path


def make_demo(directory,case):
    """New synthetic map; no benchmark/private map payload is distributed."""
    p=Path(directory);p.mkdir(parents=True,exist_ok=False)
    n=160 if case=='small_budget' else 240
    data=bytearray([254])*(n*n)
    for r in range(n):
        for c in range(n):
            if min(r,c,n-1-r,n-1-c)<4 or (case=='obstacle' and 145<=r<175 and 65<=c<75):data[r*n+c]=0
    (p/'map.pgm').write_bytes(f'P5\n{n} {n}\n255\n'.encode()+data)
    path=p/'map.yaml'
    path.write_text('image: map.pgm\nresolution: 0.05\norigin: [0.0, 0.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    return path
