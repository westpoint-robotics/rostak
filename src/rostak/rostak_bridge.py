import asyncio
import pytak
import rospy
from std_msgs.msg import String
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
            rx_proto, tx_proto = await pytak.protocol_factory(self.config)
        except:
            raise ValueError('TAK url must be valid')

        # bridge objects
        tasks = []
        if tx_proto:
            tx_queue = asyncio.Queue()
            tasks.append(pytak.TXWorker(tx_queue, self.config, tx_proto).run())
            tasks.append(RosCotWorker(tx_queue, self.config).run())

        if rx_proto:
            rx_queue = asyncio.Queue()
            tasks.append(RosTakReceiver(rx_queue, self.config, rx_proto).run())

        # start workers, restart on error
        while True:
            done, pending = await asyncio.wait(
                tasks,
                return_when=asyncio.FIRST_COMPLETED
            )

            for task in done:
                rospy.loginfo(f"Task completed: {task}")
    
class RosCotWorker(pytak.QueueWorker):
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
        # TODO: better way to keep async worker running so ROS callback threading continues
        while True:
            await asyncio.sleep(0.25)
    
class RosTakReceiver(pytak.RXWorker):
    """
    receive CoT from TAK and publish to ROS
    """
    def __init__(self, queue: asyncio.Queue, config: dict, reader: asyncio.Protocol) -> None:
        super().__init__(queue, config, reader)

    async def run(self):
        pub = rospy.Publisher('tak_rx', String, queue_size=10)
        msg = String()
        rospy.loginfo(" *** Publishing on tak_rx ***")
        while True:
            cot = await self.reader.readuntil(separator=b'</event>')
            msg.data = cot.decode()
            pub.publish(msg)

if __name__ == '__main__':
    bridge = RosTakBridge()
    asyncio.run(bridge.run())