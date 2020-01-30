# TODO: Replace layout with 2 components: "endpoint" layout & "payload" layout
from nmigen import *
from nmigen.hdl.rec import Direction


__all__ = ["Endpoint", "PHYEndpoint"]


class _EndpointBase(Record):
	def init_record(self, payload_layout, param_layout=[]):
		full_layout = [
	        ("stb"    , 1, Direction.FANOUT),   # Data transfer is valid?
	        ("rdy"    , 1, Direction.FANIN),    # Transfer can start?
	        ("eop"    , 1, Direction.FANOUT),   # Last data received?
	        ("payload", payload_layout),		# nested layout of the payload
	        ("param"  , param_layout)			# nested layout of the params, if any
		]
		Record.__init__(self, full_layout)


class Endpoint(_EndpointBase):
	def __init__(self, payload_layout, param_layout=[]):
		self.init_record(payload_layout, param_layout)


class PHYEndpoint(_EndpointBase):
	"""A Record representation of an endpoint accepting an Ethernet packet.

	Attributes
	----------
	stb : Signal()
		Output to indicate whether or not the data transfer is valid.
	rdy : Signal()
		Input to indicate whether or not the endpoint is ready for RX/TX.
	eop : Signal()
		Output to indicate whether or not this is the last packet.
	payload : :class:`Record`
		A Record representation of the packet payload. See below for its attributes.
	payload.data : Signal(data_width)
		Packet data.
	payload.last_be : Signal(data_width//2)
		Last word byte enable.
	payload.error : Signal(data_width//2):
		Packet error.
	"""
	def __init__(self, *, data_width):
		payload_layout = [
	        ("data"   ,    data_width, Direction.FANOUT),
	        ("last_be", data_width//8, Direction.FANOUT),
	        ("error"  , data_width//8, Direction.FANOUT)
		]
		self.init_record(payload_layout)
