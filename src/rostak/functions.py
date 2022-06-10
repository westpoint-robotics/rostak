from datetime import datetime, timezone, timedelta
import xml.etree.ElementTree as ET
from pytak.constants import ISO_8601_UTC

def new_cot(config, stale_in = 60) -> ET.Element:

    time = datetime.now(timezone.utc)

    root = ET.Element("event", attrib={
        "version": "2.0",
        "uid": config['uid'],
        "type": config['type'],
        "how": "m-g",
        "time": time.strftime(ISO_8601_UTC),
        "start": time.strftime(ISO_8601_UTC),
        "stale": (time + timedelta(seconds=stale_in)).strftime(ISO_8601_UTC)
    })
    
    ET.SubElement(root, "point", attrib={
        "ce": str(config['radius']),
        "le": str(config['height'])
    })
    
    detail = ET.SubElement(root, "detail")
    
    ET.SubElement(detail, "contact", attrib={
        "callsign": config['callsign'],
        "endpoint": "*:-1:stcp"
    })
    
    ET.SubElement(detail, "precisionlocation", attrib={
        "geopointsrc": "GPS",
        "altsrc": "GPS"
    })
    
    ET.SubElement(detail, "__group", attrib={
        "name": config['team'],
        "role": config["role"]
    })
    
    ET.SubElement(detail, "takv", attrib={
        "platform": "westpointrobotics/rostak"
    })
    
    return root

