"""Compressed frames must keep draining when decoded frames are rate limited."""
import threading
from types import SimpleNamespace
from unittest.mock import patch
from wla_uvc_camera.node import Camera

class Capture:
    def __init__(self):
        self.count=0; self.decoded=0
    def grab(self):
        self.count+=1
        return True
    def retrieve(self):
        self.decoded+=1
        return True,SimpleNamespace(shape=(720,2560,3))
    def read(self):
        self.grab()
        return self.retrieve()

class Closed:
    def __init__(self,capture):self.capture=capture
    def is_set(self):return self.capture.count>=8

def run_reader(rate):
    capture=Capture()
    node=SimpleNamespace(args=SimpleNamespace(decode_fps=rate,height=720,width=2560),capture=capture,closed=Closed(capture),lock=threading.Lock(),sequence=0,latest=None)
    node.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=capture.count*10000000))
    with patch('wla_uvc_camera.node.time.monotonic',side_effect=lambda:capture.count*.01):Camera.read(node)
    return node,capture

def test_decode_limit_drains_and_keeps_newest_decoded_pair():
    node,capture=run_reader(20)
    assert capture.count==8
    assert capture.decoded==2
    assert node.sequence==2
    assert node.latest[2]>=60000000

def test_default_decodes_every_frame():
    node,capture=run_reader(0)
    assert capture.count==capture.decoded==node.sequence==8
