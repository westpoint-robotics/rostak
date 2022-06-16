from datetime import datetime, timezone, timedelta
import uuid
import xml.etree.ElementTree as ET
import yaml

ISO_8601_UTC = "%Y-%m-%dT%H:%M:%S.%fZ"

class CotUtility:

    def __init__(self, config_path: str, **kwargs):

        with open(config_path) as config:
            self.cfg = yaml.safe_load(config)

        self.uid = self.cfg["uid"].replace("callsign", self.cfg["callsign"])
        self.cot_type = 'a-n-x'
        self.callsign = self.cfg["callsign"]
        self.team = self.cfg['team']
        self.fix = {"latitude": 0.0, "longitude": 0.0, "altitude":0.0}

        try:
            self.object_detect_str = kwargs['object_str']
            self.parse_str()
        except KeyError:
            self.cot_type = self.cfg["type"]


    def parse_str(self): 
        detection_list = self.object_detect_str.split(',')
        hfu = 'n'

        for cot_info in detection_list:
            cot_info = cot_info.replace(' ','')
            if list(cot_info)[0] == 'a' and list(cot_info)[1] == '-':
                self.cot_type = cot_info
            elif cot_info.lower() == 'hostile':
                hfu = 'h'
                self.team = 'red'
            elif cot_info.lower() == 'friendly':
                hfu = 'f'
                self.team = 'green'
            elif cot_info.lower() == 'unknown':
                hfu = 'u'
                self.team = 'orange'
            elif cot_info.lower() == 'neutral':
                selg.team = 'blue'
                continue
            else:
                self.callsign = cot_info

            self.uid = self.callsign
            temp = list(self.cot_type)
            temp[2] = hfu
            self.cot_type = ''.join(temp)

    def set_object_str(self, obj_str: str):
        self.object_detect_str = obj_str
        self.parse_str()

    def set_point(self, point: tuple):
        self.fix = {'latitude': point[0], 'longitude': point[1], 'altitude':point[2]}


    def new_status_msg(self, stale_in = 60) -> str:
        return ET.tostring(
            self.new_status(stale_in)
        ).decode()


    def new_status(self, stale_in = 60) -> ET.Element:
        cot = self.new_cot(stale_in)

        detail = cot.find("detail")

        ET.SubElement(detail, "contact", attrib={
            "callsign": self.callsign,
            "endpoint": "*:-1:stcp"
        })
        
        ET.SubElement(detail, "precisionlocation", attrib={
            "geopointsrc": "GPS",
            "altsrc": "GPS"
        })
        
        ET.SubElement(detail, "__group", attrib={
            "name": self.team,
            "role": self.cfg["role"]
        })
        
        ET.SubElement(detail, "takv", attrib={
            "platform": "westpointrobotics/rostak"
        })
        
        return cot


    def new_cot(self, stale_in = 60) -> ET.Element:
        cot = ET.Element("event", attrib=self.header(stale_in))

        ET.SubElement(cot, "point", attrib=self.current_point())
        ET.SubElement(cot, "detail")

        return cot


    def header(self, stale_in):
        time = datetime.now(timezone.utc)
        return {
            "version": "2.0",
            "uid": self.uid,
            "type": self.cot_type,
            "how": "m-g",
            "time": time.strftime(ISO_8601_UTC),
            "start": time.strftime(ISO_8601_UTC),
            "stale": (time + timedelta(seconds=stale_in)).strftime(ISO_8601_UTC)
        }


    def current_point(self):
        return {
            "lat": str(self.fix['latitude']),
            "lon": str(self.fix['longitude']),
            "hae": str(self.fix['altitude']),
            "ce": str(self.cfg['radius']),
            "le": str(self.cfg['height'])
        }


    def get_config(self):
        return self.cfg

    def get_zone(self):
        return self.cfg['utm_zone']
