from datetime import datetime, timezone, timedelta
import uuid
import xml.etree.ElementTree as ET
import yaml

ISO_8601_UTC = "%Y-%m-%dT%H:%M:%S.%fZ"

class CotUtility:

    def __init__(self, config_path: str):
        
        with open(config_path) as config:
            self.cfg = yaml.safe_load(config)
        
        self.cfg["uid"] = self.cfg["uid"].replace("callsign", self.cfg["callsign"])
        
        self.fix = {
            "latitude": "0.0",
            "longitude": "0.0",
            "altitude": "0.0"
        }

    def new_cot(self, stale_in = 60) -> ET.Element:
        cot = ET.Element("event", attrib=self.header(stale_in))
        
        ET.SubElement(cot, "point", attrib=self.current_point())
        ET.SubElement(cot, "detail")
        
        return cot

    def new_status_msg(self, stale_in = 60) -> str:
        return ET.tostring(
            self.new_status(stale_in)
        ).decode()
    
    def new_chat_msg(self, stale_in = 60) -> str:
        return ET.tostring(
            self.new_chat(stale_in)
        ).decode()
    
    def new_status(self, stale_in = 60) -> ET.Element:

        cot = self.new_cot(stale_in)
        
        detail = cot.find("detail")
        
        ET.SubElement(detail, "contact", attrib={
            "callsign": self.cfg['callsign'],
            "endpoint": "*:-1:stcp"
        })
        
        ET.SubElement(detail, "precisionlocation", attrib={
            "geopointsrc": "GPS",
            "altsrc": "GPS"
        })
        
        ET.SubElement(detail, "__group", attrib={
            "name": self.cfg['team'],
            "role": self.cfg["role"]
        })
        
        ET.SubElement(detail, "takv", attrib={
            "platform": "westpointrobotics/rostak"
        })
        
        return cot

    def new_chat(self, text, stale_in = 84600):
        msg_id = str(uuid.uuid4())
        
        cot = self.new_cot(stale_in)
        cot.set("type", "b-t-f")

        detail = cot.find("detail")
        
        chat = ET.SubElement(detail, "__chat", {
            "parent": "",
            "groupOwner": "false",
            "messageId": msg_id,
            "chatroom": self.cfg["team"],
            "id": self.cfg['team'],
            "senderCallsign": self.cfg['callsign']
        })
        
        remarks = ET.SubElement(detail, "remarks", {
            "source": "",
            "time": ""
        })
        remarks.text = text
        
        ET.SubElement(detail, "link", {
            "uid": self.cfg['uid'],
            "type": self.cfg['type'],
            "relation": "p-p"
        })
        
        return cot

    def set_point(self, fix: dict):
        self.fix = fix
    
    def current_point(self):
        return {
            "lat": str(self.fix.latitude),
            "lon": str(self.fix.longitude),
            "hae": str(self.fix.altitude),
            "ce": str(self.cfg['radius']),
            "le": str(self.cfg['height'])
        }
    
    def header(self, stale_in):
        time = datetime.now(timezone.utc)
        return {
            "version": "2.0",
            "uid": self.cfg['uid'],
            "type": self.cfg['type'],
            "how": "m-g",
            "time": time.strftime(ISO_8601_UTC),
            "start": time.strftime(ISO_8601_UTC),
            "stale": (time + timedelta(seconds=stale_in)).strftime(ISO_8601_UTC)
        }

    def get_config(self):
        return self.cfg
