from datetime import datetime, timezone, timedelta
from math import cos, asin, sqrt, pi
import re
import uuid
import xml.etree.ElementTree as ET
import yaml

ISO_8601_UTC = "%Y-%m-%dT%H:%M:%S.%fZ"

class CotUtility:

    def __init__(self, config_path: str):
        
        with open(config_path) as config:
            self.cfg = yaml.safe_load(config)
        
        self.cfg["uid"] = self.cfg["uid"].replace("callsign", self.cfg["callsign"])
        self.atoms = {}
        self.course = 0.0
        self.speed = 0.0
        self.battery = 0
        self.fixtime = datetime.now(timezone.utc)
        self.fix = {
            "latitude": 0.0,
            "longitude": 0.0,
            "altitude": 0.0
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
    
    def new_chat_msg(self, text, stale_in = 60) -> str:
        return ET.tostring(
            self.new_chat(text, stale_in)
        ).decode()
    
    def new_status(self, stale_in = 60) -> ET.Element:

        cot = self.new_cot(stale_in)
        
        detail = cot.find("detail")

        ET.SubElement(detail, "contact", attrib={
            "callsign": self.cfg['callsign'],
            "endpoint": "*:-1:stcp"
        })
        
        ET.SubElement(detail, "__group", attrib={
            "name": self.cfg['team'],
            "role": self.cfg["role"]
        })

        ET.SubElement(detail, "track", attrib={
            "course": str(self.course),
            "speed": str(self.speed)
        })

        ET.SubElement(detail, "status", attrib={
            "battery": str(self.battery)
        })

        ET.SubElement(detail, "precisionlocation", attrib={
            "geopointsrc": "GPS",
            "altsrc": "GPS"
        })
        
        ET.SubElement(detail, "takv", attrib={
            "device": "robot_payload",
            "os": "os",
            "platform": "westpointrobotics/rostak",
            "version": "1.0"
        })
        
        return cot

    def new_chat(self, text, stale_in = 84600) -> ET.Element:
        msg_id = str(uuid.uuid4())
        
        # if reply-to:
        match = re.search(r"reply-to:([^\s]+)", text)
        recipient = match.group(1) if match else self.cfg["team"]
        text = text.replace(match.group(0), "").strip()
        
        # todo: lookup recipient's callsign
        # todo: lookup teammates

        cot = self.new_cot(stale_in)
        cot.set("uid", "GeoChat." + self.cfg["uid"] + "." + recipient + "." + msg_id)
        cot.set("type", "b-t-f")

        detail = cot.find("detail")
        
        if recipient == self.cfg["team"]:
            chat = ET.SubElement(detail, "__chat", {
                "parent": "TeamGroups" if recipient == self.cfg["team"] else "RootContactGroup",
                "groupOwner": "false",
                "messageId": msg_id,
                "chatroom": self.cfg["team"],
                "id": self.cfg['team'],
                "senderCallsign": self.cfg['callsign']
            })
        else:
            chat = ET.SubElement(detail, "__chat", {
                "parent": "RootContactGroup",
                "groupOwner": "false",
                "messageId": msg_id,
                "chatroom": recipient_info["callsign"],
                "id": recipient_info["uid"],
                "senderCallsign": self.cfg['callsign']
            })
        
        remarks = ET.SubElement(detail, "remarks", {
            "source": "BAO.F.ATAK." + self.cfg["uid"],
            "time": cot.get("time")
        })
        remarks.text = text
        
        ET.SubElement(detail, "link", {
            "uid": self.cfg['uid'],
            "type": self.cfg['type'],
            "relation": "p-p"
        })
        
        return cot

    def set_point(self, fix: dict):
        ts = datetime.now(timezone.utc)
        sec = (ts - self.fixtime).total_seconds()
        m = self.distance(
            self.fix['latitude'], self.fix['longitude'], self.fix['altitude'],
            fix.latitude, fix.longitude, fix.altitude
        )
        self.speed = m / sec
        self.fix = {
            "latitude": fix.latitude,
            "longitude": fix.longitude,
            "altitude": fix.altitude
        }
        self.fixtime = ts
    
    def set_speed(self, speed):
        self.speed = speed
        
    def set_course(self, course):
        self.course = course

    def current_point(self):
        return {
            "lat": str(self.fix['latitude']),
            "lon": str(self.fix['longitude']),
            "hae": str(self.fix['altitude']),
            "ce": str(self.cfg['radius']),
            "le": str(self.cfg['height'])
        }
    
    def is_moving(self):
        return self.speed > 0.5

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

    def extract_cmd(self, cot):
        remarks = cot.find("./detail/remarks")
        if remarks is None:
            return ""
        text = remarks.text if "#!" in remarks.text else ""
        print(text)
        return text.lower()

    def extract_callsign(self, cot):
        contact = cot.find("./detail/contact")
        if contact is None:
            return ""
        callsign = contact.get("callsign")
        return callsign.replace(" ", "").lower()

    def extract_point(self, cot):
        point = cot.find("./point")
        if point is None:
            return ""
        return point.get("lat", "0.0") + "," + point.get("lon", "0.0") + "," + point.get("hae", "0.0")

    def extract_route(self, cot):
        links = [x.attrib["point"] for x in cot.findall("./detail/link")]
        delimiter = " "
        return delimiter.join(links)

    def resolve_target(self, item):
        if item in self.atoms.keys():
            return self.atoms[item]
        found = [x for x in self.atoms.values() if x["callsign"] == item]
        return found[0]["data"] if len(found) > 0 else None

    def process_cot(self, xml):
        # print(xml)
        cot = ET.fromstring(xml)
        uid=cot.get("uid").lower()
        callsign = self.extract_callsign(cot)
        point = self.extract_point(cot)
        code = cot.get("type")
        
        ''' record atoms '''
        if code.startswith("a-"):
            self.atoms[uid] = {
                "uid": uid,
                "code": code,
                "callsign": callsign,
                "data": point
            }
            print("updated " + callsign + " at " + point)
        
        ''' record route '''
        if code.startswith("b-m-r"):
            route = self.extract_route(cot)
            self.atoms[uid] = {
                "uid": uid,
                "code": code,
                "callsign": callsign,
                "data": route
            }
        
        ''' if chat msg has command, resolve target callsign '''
        if code.startswith("b-t-f"):
            cmd = self.extract_cmd(cot)
            parts = cmd.split(" ")
            if len(parts) < 2:
                return cmd 
            data = self.resolve_target(parts[1])
            if parts[1] == "me":
                data = point
            return cmd + " " + data

        return ""

    def distance(self, lat1, lon1, alt1, lat2, lon2, alt2):
        p = pi/180
        a = 0.5 - cos((lat2-lat1)*p)/2 + cos(lat1*p) * cos(lat2*p) * (1-cos((lon2-lon1)*p))/2
        h = 12742 * asin(sqrt(a)) #2*R*asin...
        v = sqrt(h**2 + abs(alt1 - alt2)**2)
        return h + v
