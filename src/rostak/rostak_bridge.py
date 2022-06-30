import asyncio
import os
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
            rospy.loginfo(self.config["COT_URL"])
            rx_proto, tx_proto = await pytak.protocol_factory(self.config)
        except:
            raise ValueError('TAK url must be valid')

        self.r_pipe, self.w_pipe = os.pipe()

        rospy.Subscriber("tak_tx", String, self.queue_cotmsg)

        # bridge objects
        tasks = []
        if tx_proto:
            self.tx_queue = asyncio.Queue()
            tasks.append(pytak.TXWorker(self.tx_queue, self.config, tx_proto).run())
            tasks.append(RosCotWorker(self.tx_queue, self.config, self.r_pipe).run())

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
    
    def queue_cotmsg(self, cotmsg):
        rospy.loginfo("writing to pipe")
        os.write(self.w_pipe, (cotmsg.data + '\n').encode('utf-8'))
        # self.tx_queue.put(
        #     cotmsg.data.encode('utf-8')
        # )

class RosCotWorker(pytak.QueueWorker):
    """
    listen for CoT from ROS and enqueue for TAK
    """
    def __init__(self, queue: asyncio.Queue, config: dict, r_pipe) -> None:
        super().__init__(queue, config)
        self.r_pipe = r_pipe
        # self.aio = asyncio.get_event_loop()

    # def queue_cotmsg(self, cotmsg):
    #     self.aio.create_task(
    #         self.put_queue(cotmsg.data.encode('utf-8'))
    #     )
    
    async def run(self):
        r = os.fdopen(self.r_pipe)
        while True:
            rospy.loginfo("reading pipe")
            cot = r.read()
            rospy.loginfo(cot)
            await self.put_queue(cot)
        # rospy.loginfo(" *** Subscribing to tak_tx ***")
        # rospy.Subscriber("tak_tx", String, self.queue_cotmsg)
        # # TODO: better way to keep async worker running so ROS callback threading continues
        # while True:
        #     await asyncio.sleep(0.1)
    
class RosTakReceiver():
    """
    receive CoT from TAK and publish to ROS
    """
    def __init__(self, queue: asyncio.Queue, config: dict, reader: asyncio.Protocol) -> None:
        self.reader = reader
        
    async def run(self):
        pub = rospy.Publisher('tak_rx', String, queue_size=10)
        msg = String()
        rospy.loginfo(" *** Publishing on tak_rx ***")
        while True:
            try:
                cot = await self.reader.readuntil(separator=b'</event>')
                msg.data = cot.decode()
                pub.publish(msg)
            except asyncio.exceptions.IncompleteReadError as e:
                print(e)
                rospy.sleep(1)

if __name__ == '__main__':
    bridge = RosTakBridge()
    asyncio.run(bridge.run())
