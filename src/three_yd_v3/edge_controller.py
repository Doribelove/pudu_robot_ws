from .channel_controller import ChannelController
from .edge_channel import EdgeChannel


class EdgeController(ChannelController):
    channel_class=EdgeChannel
