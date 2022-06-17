# rostak

Proxy CoT messages (XML strings) between ROS and a TAK environment

## Config

Install pytak v5 using one of the [installation methods](https://github.com/ampledata/pytak#installation).
This requires Python 3.6 or greater.

See the example launch file.  Just needs a url for the TAK endpoint.
If using a TLS transport, some environment variables need to be set.
See pytak's readme for more information.

## Published Topics
These topics will be published by rostak when the user of tak queries the proper functions. 

- `rostak_goto_ll`: publishes latitude and longitude coordinates specified by the RRC-ATAK GOTO plugin on the PoseStamped message type where `x` is latitude `y` is longitude.
- `rostak_goto_utm`: publishes utm coordinates specified by the RRC-ATAK GOTO plugin on the PoseStamped message type where `x` is easting and `y` is northing.
- `rostak_path_ll`: publishes latitude and longitude coorsinates specified by the path feature in tak. The coordinates are published on the Path message type where `x` is latitude and `y` is longitude.
- `rostak_path_utm`: publishes utm coordinates specificed by the Path feature in tak. The coordinates are published on the Path message type where `x` is easting and `y` is northing.
- `rostak_path_reamrks`: publishes what is in the remarks sections of the tak path messages sent to the robot. The string is published on a String message type. 

## Subscribed Topics 
These topics are designed to have the robot publish on them such that the information will be transferred to a CoT message and sent to tak server by rostak.

- `fix`: subscribes to the robots location on PointStamped message type. Set the `frame_id` to utm if you are using utm coordinates, the default is latitude and longitude. 
- `obj`: subscribes to the robots object detections on MarkerArray message type. Set the `frame_id` of ech sub Marker message to utm if you are using utm coordinates, the default is latitude and longitude. Within the sub Marker message set the objects location in the Pose message, and infomation about the object in the String `text` message, where you can provide the objects unique ID, (i.e. vehicle01), whether is friendly, neutral, hostile or unknown, and they CoT type, seperated by commas. The unique ID is necessary to set but the others are optional. Example string: "vehicle01,neutral,a-n-G-E-V".

## Internal Topics

- `tak_tx`: transmits CoT xml strings on this topic to TAK
- `tak_rx`: receives CoT xml strings from TAK and publishes on this topic

For other CoT messages you'd like to send, just publish the xml string to `tak_tx`.
For receiving messages, subscribe to `tak_rx` and process the CoT xml string as desired.
You will likely want to do these outside of this repo, as much of the handling will be 
autonomy stack dependent and not generically applicable to all consumers of the rostak nodes.

## Secrets

TLS connections require a certficate and private key.
Secrets should not be included in source control.
The example.launch file provides a recommendation for 
settings paths in a tak_params file. (Paths loaded by
rosparam don't get macros resolved, so if wanting to 
use `..._CERT="$(find some_pkg)/config/client.pem"`, 
you'll need to pass them into launch file as individual
args.)

## Acknowledgements

Based on the great work of [pytak](https://github.com/ampledata/pytak).
