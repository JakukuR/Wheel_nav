import ast
from pathlib import Path

def test_follow_response_preserves_identity_sections():
    source=Path(__file__).resolve().parents[2]/'wla_r680_navigation/wla_r680_navigation_py/debug_viewer.py'
    tree=ast.parse(source.read_text())
    assignment=next(n for n in ast.walk(tree) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Subscript) and isinstance(t.value,ast.Name) and t.value.id=='result' and isinstance(t.slice,ast.Constant) and t.slice.value=='follow' for t in n.targets))
    status=dict(geometry_binding={'state':'lost'},identity_supervision={'state':'searching'},state='waiting')
    scope=dict(status=status,status_fresh=True,result={})
    exec(compile(ast.Module(body=[assignment],type_ignores=[]),str(source),'exec'),scope)
    assert scope['result']['follow']['geometry_binding']==status['geometry_binding']
    assert scope['result']['follow']['identity_supervision']==status['identity_supervision']
