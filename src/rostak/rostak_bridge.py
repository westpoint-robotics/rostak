import asyncio
import takproto
import pytak
import rospy
import json
from std_msgs.msg import String, UInt8MultiArray, MultiArrayDimension
from configparser import ConfigParser

class RosTakBridge:
    """
    Proxy CoT messages (xml strings) between ROS and TAK agents.
    """

    def __init__(self):
        rospy.init_node("rostak_bridge")
        self.config = ConfigParser()['DEFAULT']
        self.config["COT_URL"] = rospy.get_param('~COT_URL', "")
        self.config["PYTAK_TLS_CLIENT_CERT"] = rospy.get_param('~PYTAK_TLS_CLIENT_CERT', "")
        self.config["PYTAK_TLS_CLIENT_KEY"] = rospy.get_param('~PYTAK_TLS_CLIENT_KEY', "")
        self.config["PYTAK_TLS_CLIENT_CAFILE"] = rospy.get_param('~PYTAK_TLS_CLIENT_CAFILE', "")
        self.config["PYTAK_TLS_CLIENT_CIPHERS"] = rospy.get_param('~PYTAK_TLS_CLIENT_CIPHERS', "")
        self.config["PYTAK_TLS_DONT_VERIFY"] = rospy.get_param('~PYTAK_TLS_DONT_VERIFY', "")
        self.config["PYTAK_TLS_DONT_CHECK_HOSTNAME"] = rospy.get_param('~PYTAK_TLS_DONT_CHECK_HOSTNAME', "")

    async def run(self):

        # connect to tak server
        try:
            rospy.loginfo(self.config["COT_URL"])
            rx_protocol, tx_protocol = await pytak.protocol_factory(self.config)
        except:
            raise ValueError('TAK url must be valid')

        # bridge objects
        tasks = []
        if tx_protocol:
            tx_queue = asyncio.Queue()
            tasks.append(Ros_to_Tak_Worker(tx_queue, self.config).run())
            tasks.append(pytak.TXWorker(tx_queue, self.config, tx_protocol).run())

        if rx_protocol:
            rx_queue = asyncio.Queue()
            tasks.append(CustomRXWorker(rx_queue, self.config, rx_protocol).run())
            tasks.append(Tak_to_Ros_Worker(rx_queue, self.config).run())

        done, pending = await asyncio.wait(
            tasks,
            return_when=asyncio.FIRST_COMPLETED
        )

        for task in done:
            rospy.loginfo(f"Task completed: {task}")

class Ros_to_Tak_Worker(pytak.QueueWorker):
    """
    listen for CoT from ROS and enqueue for TAK
    """
    def __init__(self, queue: asyncio.Queue, config: dict) -> None:
        super().__init__(queue, config)
        self.aio = asyncio.get_event_loop()

    def queue_cotmsg(self, cotmsg):
        self.aio.create_task(
            self.put_queue(cotmsg.data.encode('utf-8'))
        )

    async def run(self):
        rospy.loginfo(" *** Subscribing to tak_tx ***")
        rospy.Subscriber("tak_tx", String, self.queue_cotmsg)
        # TODO: better blocking so ROS subscription continues
        while True:
            await asyncio.sleep(0.25)

class Tak_to_Ros_Worker(pytak.Worker):
    """
    publish received tak messages to ros
    """
    def __init__(self, queue: asyncio.Queue, config: dict) -> None:
        super().__init__(queue, config)
        rospy.loginfo(" *** Publishing on tak_rx ***")
        self.pub = rospy.Publisher('tak_rx', UInt8MultiArray, queue_size=10)
        self.msg = UInt8MultiArray()
        self.msg.layout.dim.append(MultiArrayDimension())
        self.msg.layout.dim[0].label = "length"

    async def handle_data(self, data: bytes) -> None:
        self.msg.layout.dim[0].size = len(data)
        self.msg.layout.dim[0].stride = len(data)
        self.msg.data = data
        self.pub.publish(self.msg)

class CustomRXWorker(pytak.RXWorker):
    """
    handle tak protocol payload v0 and v1 (xml, protobuf)
    """
    async def readcot(self):
        if hasattr(self.reader, 'readuntil'):
            xml = await self.reader.readuntil("</event>".encode("UTF-8"))
            cot = bytes(takproto.xml2proto(xml.decode()))
        elif hasattr(self.reader, 'recv'):
            cot, _ = await self.reader.recv()
        return cot

if __name__ == '__main__':
    bridge = RosTakBridge()
    asyncio.run(bridge.run())
