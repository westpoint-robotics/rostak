from datetime import datetime, timezone, timedelta
import xml.etree.ElementTree as ET

ISO_8601_UTC = "%Y-%m-%dT%H:%M:%S.%fZ"

def new_cot(config, stale_in = 5) -> ET.Element:

    time = datetime.now(timezone.utc)

    root = ET.Element("event", attrib={
        "version": "2.0",
        "uid": config['uid'],
        "type": config['type'],
        "how": "m-g",
        "time": time.strftime(ISO_8601_UTC),
        "start": time.strftime(ISO_8601_UTC),
        "stale": (time + timedelta(minutes=stale_in)).strftime(ISO_8601_UTC)
    })
    
    ET.SubElement(root, "point", attrib={
        "ce": str(config['radius']),
        "le": str(config['height'])
    })
    
    detail = ET.SubElement(root, "detail")
    
    ET.SubElement(detail, "contact", attrib={
        "callsign": config['callsign']
    })
    
    ET.SubElement(detail, "takv", attrib={
        "platform": "westpointrobotics/rostak_bridge"
    })
    
    return root
