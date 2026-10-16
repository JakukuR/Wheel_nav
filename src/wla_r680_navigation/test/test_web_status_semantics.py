"""Exercise the real JSON projection without ROS, HTTP, or a camera."""
import ast
import json
from pathlib import Path
import threading
import time

source = Path(__file__).resolve().parents[1] / 'scripts/web_gateway'
tree = ast.parse(source.read_text(encoding='utf-8'))
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'GatewayNode')
method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'state_json')
namespace = dict(json=json, time=time)
exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)

class Stub:
    _lock = threading.Lock()
    _state = dict(map={}, navigation_ready=False)
    _received = dict(odom=time.monotonic(), robot=time.monotonic())

node = Stub()
project = namespace['state_json']
def state():
    return json.loads(project(node))['connected']

assert state()['odom'] and state()['robot'] and not state()['navigation']
node._state['navigation_ready'] = True
node._received['navigation_ready'] = time.monotonic()
assert state()['navigation']
node._received['navigation_ready'] -= 1.
assert state()['odom'] and not state()['navigation']
node._received['navigation_ready'] = time.monotonic()
node._state['navigation_ready'] = False
assert not state()['navigation']
print('PASS odom/TF availability does not imply navigation readiness; stale/false readiness closes display')
