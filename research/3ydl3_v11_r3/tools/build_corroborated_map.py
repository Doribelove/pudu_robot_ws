"""Build the confirmed V5-r3 effective map from supplied map layers.

This does not modify the user-confirmed optemap PGM carrier.  It emits a new
PNG/YAML pair and an audit record.  A PGM-unknown cell is opened only when both
companion maps mark the same pixel free and the manual semantic annotation
marks it as a lane, junction, or parking area.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import Image
import yaml

from arena_3d_v1.semantic_world import SemanticWorld, json_write
from three_yd_v5.topology import V5Topology


ROOT=Path(__file__).resolve().parents[1]


def file_sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    source_yaml=ROOT/'inputs/map.yaml'
    source_png=ROOT/'inputs/original_grid.png'
    companion_a=ROOT/'inputs/optemap.png'
    companion_b=ROOT/'inputs/static_map.png'
    output_png=ROOT/'inputs/corroborated_grid.png'
    output_yaml=ROOT/'inputs/corroborated_map.yaml'
    audit_path=ROOT/'report/corroborated_map_build.json'

    config=yaml.safe_load(source_yaml.read_text())
    source=np.asarray(Image.open(source_png).convert('L'))
    a=np.asarray(Image.open(companion_a).convert('L'))
    b=np.asarray(Image.open(companion_b).convert('L'))
    if source.shape != a.shape or source.shape != b.shape:
        raise ValueError('MAP_LAYER_SHAPE_MISMATCH')

    world=SemanticWorld(source_yaml,ROOT/'inputs/semantic.json',
                        cache_root=ROOT/'report/corroborated_map_world')
    topology=V5Topology(world,ROOT/'inputs/topology.json',
                        ROOT/'report/corroborated_map_graph')
    labels=topology.subregions
    physical_ids=[rid for rid,desc in topology.regions.items()
                  if desc['kind'] in ('lane','junction_area','parking_area')]
    semantic_drivable=np.isin(labels,physical_ids)

    occupied_cut=round((1-float(config['occupied_thresh']))*255)
    free_cut=round((1-float(config['free_thresh']))*255)
    unknown=(source>occupied_cut)&(source<free_cut)
    # Keep the large, canonical gray-128 unknown field closed.  Only consider
    # the near-free tail immediately below the YAML free threshold.  The lower
    # bound 190 is fixed in the confirmed contract and recorded in the audit; it covers
    # the observed 190/202 seam pixels without opening generic unknown space.
    near_free_unknown=unknown&(source>=190)
    corroborated=near_free_unknown&(a==255)&(b==255)&semantic_drivable
    candidate=source.copy();candidate[corroborated]=255
    Image.fromarray(candidate).save(output_png)
    candidate_config=dict(config);candidate_config['image']=output_png.name
    output_yaml.write_text(yaml.safe_dump(candidate_config,sort_keys=False))

    values,counts=np.unique(source[corroborated],return_counts=True)
    json_write(audit_path,{
        'schema':'3YD-V5-r3-effective-map-v1',
        'status':'confirmed_map_loading_contract_20260921; source PGM retained and effective map generated separately',
        'rule':'PGM near-free unknown gray 190..204 AND optemap.png free AND static_map.png free AND manual semantic drivable',
        'unknown_is_collision_outside_corroborated_cells':True,
        'source_yaml':str(source_yaml),'source_image':str(source_png),
        'source_image_sha256':file_sha(source_png),
        'companion_images':[str(companion_a),str(companion_b)],
        'companion_sha256':[file_sha(companion_a),file_sha(companion_b)],
        'output_yaml':str(output_yaml),'output_image':str(output_png),
        'output_image_sha256':file_sha(output_png),
        'shape':list(source.shape),'occupied_cut_max_gray':occupied_cut,
        'free_cut_min_gray':free_cut,'candidate_unknown_min_gray':190,
        'source_unknown_cells':int(unknown.sum()),
        'source_near_free_unknown_cells':int(near_free_unknown.sum()),
        'semantic_drivable_cells':int(semantic_drivable.sum()),
        'corroborated_cells_opened':int(corroborated.sum()),
        'opened_source_gray_histogram':{str(int(v)):int(n) for v,n in zip(values,counts)},
        'real_robot_execution':False,
    })
    print(json.dumps(json.loads(audit_path.read_text()),indent=2,ensure_ascii=False))


if __name__=='__main__':
    main()
