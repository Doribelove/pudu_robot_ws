"""Run the unchanged r1 algorithm with raw-path and ACK observations."""
import static_benchmark
from arena_3d_v1.diagnostic_session import DiagnosticSession

static_benchmark.ReferenceSmacSession = DiagnosticSession
if __name__ == '__main__':
    static_benchmark.main()
