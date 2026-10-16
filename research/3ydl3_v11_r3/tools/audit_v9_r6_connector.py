"""Verify the only connector proposal difference is the registered filter.

Normalization must reproduce the original function AST including every curve,
body, semantic, steering, ambiguity and budget statement. No rule exemption.
"""
import ast
from copy import deepcopy


def function(path, name):
    return next(n for n in ast.parse(path.read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name == name)


def audit_connector(root, frozen):
    current=root/'src/three_yd_v5/progress_connector.py'
    original=frozen/'src/three_yd_v5/progress_connector.py'
    assert ast.dump(function(current,'quintic')) == ast.dump(function(original,'quintic'))
    p=deepcopy(function(current,'propose'))
    expected=ast.parse('limit=heading_limit_degrees()').body[0]
    assert ast.dump(p.body[0]) == ast.dump(expected)
    p.body.pop(0)
    class Normalize(ast.NodeTransformer):
        count_filter=0;count_log=0
        def visit_Call(self,n):
            if (isinstance(n.func,ast.Attribute) and isinstance(n.func.value,ast.Name)
                    and n.func.value.id=='np' and n.func.attr=='deg2rad'
                    and len(n.args)==1 and isinstance(n.args[0],ast.Name) and n.args[0].id=='limit'):
                n.args[0]=ast.Constant(value=10);self.count_filter+=1
            return self.generic_visit(n)
        def visit_Dict(self,n):
            for i,k in enumerate(n.keys):
                if isinstance(k,ast.Constant) and k.value=='candidate_heading_limit_deg':
                    assert isinstance(n.values[i],ast.Name) and n.values[i].id=='limit'
                    n.keys.pop(i);n.values.pop(i);self.count_log+=1;break
            return self.generic_visit(n)
    x=Normalize();p=x.visit(p)
    assert x.count_filter==1 and x.count_log==1
    assert ast.dump(p)==ast.dump(function(original,'propose'))
    return {'original_quintic_AST_unchanged':True,'original_propose_AST_after_filter_normalization_identical':True,
            'only_candidate_heading_filter_10_or_15_deg':True,'original_hard_audits_and_budget_preserved':True}
