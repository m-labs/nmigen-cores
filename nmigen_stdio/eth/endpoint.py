# TODO: Replace layout with 2 components: "endpoint" layout & "payload" layout
from nmigen import *
from nmigen.hdl.rec import Direction


__all__ = ["PHYEndpoint"]


class _EndpointBase(Record):
	def init_record(self, payload_layout, param_layout=[]):
		full_layout = [
	        ("stb"    , 1, Direction.FANOUT),   # Externally driven: Data transfer is enabled?
	        ("rdy"    , 1, Direction.FANIN),    # Internally driven: Transfer can start?
	        ("eop"    , 1, Direction.FANOUT),   # Last data received?
	        ("payload", payload_layout),		# nested layout of the payload
	        ("param"  , param_layout)			# nested layout of the params, if any
		]
		Record.__init__(self, full_layout)


class PHYEndpoint(_EndpointBase):
	def __init__(self, *, data_width):
		payload_layout = [
	        ("data"   ,    data_width, Direction.FANOUT),
	        ("last_be", data_width//8, Direction.FANOUT),
	        ("error"  , data_width//8, Direction.FANOUT)
		]
		self.init_record(payload_layout)
