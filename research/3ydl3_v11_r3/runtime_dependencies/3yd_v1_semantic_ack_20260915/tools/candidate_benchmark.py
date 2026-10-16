"""r1 plus bounded curvature repair and stable ACK session."""
if __name__=='__main__':
 from owned_process import supervised_entry
 supervised_entry()
import static_benchmark
from arena_3d_v1.stable_session import StableSmacSession
from arena_3d_v1.curvature_repair import TurnaroundRecovery
static_benchmark.ReferenceSmacSession=StableSmacSession
static_benchmark.CandidateRecovery=TurnaroundRecovery
if __name__=='__main__':static_benchmark.main()
