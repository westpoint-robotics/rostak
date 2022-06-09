import asyncio
import pytak
import rospy
import urllib
from std_msgs.msg import String

class RosTakBridge:
    """
    Proxy CoT messages (xml strings) between ROS and TAK agents.
    """

    def __init__(self):
        rospy.init_node("rostak_bridge")

    async def run(self):

        # connect to tak server
        tak_url_param = rospy.get_param('~url', "")
        try:
            tak_url = urllib.parse.urlparse(tak_url_param)
            rx_proto, tx_proto = await pytak.protocol_factory(tak_url)
        except:
            raise ValueError('TAK url must be valid')

        # bridge objects
        tx_queue = asyncio.Queue()
        rx_queue = asyncio.Queue()
        reader = RosTakReceiver(rx_queue, rx_proto)
        writer = pytak.EventTransmitter(tx_queue, tx_proto)
        listener = RosCotListener(tx_queue, {})
        
        # start workers, restart on error
        while True:
            done, pending = await asyncio.wait(
                {listener.run(), reader.run(), writer.run()},
                return_when=asyncio.FIRST_COMPLETED
            )

            for task in done:
                rospy.loginfo(f"Task completed: {task}")
    
class RosCotListener(pytak.MessageWorker):
    """
    listen for CoT from ROS and enqueue for TAK
    """
    def __init__(self, event_queue: asyncio.Queue,  config: dict) -> None:
        super().__init__(event_queue)
        self.config = config
        self.aio = asyncio.get_event_loop()

    def queue_cotmsg(self, cotmsg):
        self.aio.create_task(
            self._put_event_queue(cotmsg.data.encode('utf-8'))
        )
    
    async def run(self):
        rospy.Subscriber("tak_tx", String, self.queue_cotmsg)
        # TODO: better way to keep async worker running so ROS callback threading continues
        while True:
            await asyncio.sleep(0.25)
    
class RosTakReceiver(pytak.EventReceiver):
    """
    receive CoT from TAK and publish to ROS
    """
    def __init__(self, event_queue: asyncio.Queue, reader: asyncio.Protocol) -> None:
        super().__init__(event_queue, reader)

    async def run(self):
        pub = rospy.Publisher('tak_rx', String, queue_size=10)
        msg = String()
        rospy.loginfo(" *** Publishing on tak_rx ***")
        while True:
            msg.data = await self.event_queue.get()
            rospy.loginfo(msg.data)
            pub.publish(msg)

if __name__ == '__main__':
    bridge = RosTakBridge()
    asyncio.run(bridge.run())