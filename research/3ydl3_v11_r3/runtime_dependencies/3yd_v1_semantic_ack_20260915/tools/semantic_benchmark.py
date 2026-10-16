"""Same frozen query/geometry pipeline with selectable publication ablations."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
from native_snapshot import configure
configure()
import argparse,sys
import static_benchmark
from arena_3d_v1.stable_session import StableSmacSession
from arena_3d_v1.semantic_ack_session import SemanticAckSession
from arena_3d_v1.curvature_repair import TurnaroundRecovery
p=argparse.ArgumentParser(add_help=False);p.add_argument('--publication',choices=['baseline','observed_baseline','paced','repair','combined'],default='combined')
a,rest=p.parse_known_args();sys.argv=[sys.argv[0],*rest]
if a.publication=='baseline':static_benchmark.ReferenceSmacSession=StableSmacSession
else:
 class ConfiguredSession(SemanticAckSession):
  def __init__(self,*args,**kwargs):
   super().__init__(*args,tile_interval_s=.015 if a.publication in ['paced','combined'] else 0.,enable_semantic_repair=a.publication in ['repair','combined'],**kwargs)
 static_benchmark.ReferenceSmacSession=ConfiguredSession
static_benchmark.CandidateRecovery=TurnaroundRecovery
if __name__=='__main__':static_benchmark.main()
