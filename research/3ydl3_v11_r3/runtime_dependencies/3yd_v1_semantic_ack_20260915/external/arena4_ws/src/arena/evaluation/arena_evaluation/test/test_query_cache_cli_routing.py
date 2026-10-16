import ast
import inspect
from types import SimpleNamespace
from arena_evaluation import two_layer_v1_r3_benchmark as runner
from arena_evaluation.roi_backend_r3 import RoiExactAckSmacSession


def test_cli_roi_configuration_is_accepted_by_the_constructor_chain():
    # Execute the actual CLI's session-keyword expression with distinct cache
    # settings. This catches parser options accidentally forwarded into ROS.
    tree=ast.parse(inspect.getsource(runner.main))
    expressions=[node.value for node in ast.walk(tree) if isinstance(node,ast.Assign)
        and any(isinstance(t,ast.Name) and t.id=='session_kwargs' for t in node.targets)
        and isinstance(node.value,ast.Dict) and node.value.keys]
    assert len(expressions)==1
    args=SimpleNamespace(roi_max_cells=80*1024**2,ready_context_max_cells=32*1024**2,
                         query_cache_max_bytes=2*1024**3,corridor_straight_half_width_m=1.2)
    kwargs=eval(compile(ast.Expression(expressions[0]),'<actual CLI session kwargs>','eval'),{'args':args})
    accepted=set()
    for cls in RoiExactAckSmacSession.__mro__:
        accepted.update(name for name,param in inspect.signature(cls.__init__).parameters.items()
                        if param.kind not in {param.VAR_KEYWORD,param.VAR_POSITIONAL})
    assert set(kwargs)<=accepted, f'Unknown ROS session constructor parameters: {set(kwargs)-accepted}'
    assert kwargs['max_cells']==args.roi_max_cells
    assert kwargs['ready_context_max_cells']==args.ready_context_max_cells
