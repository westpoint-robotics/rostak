# rostak

Proxy CoT messages (XML strings) between ROS and a TAK environment

## Config

Install pytak v5 using one of the [installation methods](https://github.com/ampledata/pytak#installation).

See the example launch file.  Just needs a url for the TAK endpoint.
If using a TLS transport, some environment variables need to be set.
See pytak's readme for more information.

## Topics

- `tak_tx`: transmits CoT xml strings on this topic to TAK
- `tak_rx`: receives CoT xml strings from TAK and publishes on this topic

There is a built-in listener for `/fix`, so CoT status messages automatically
get sent to TAK.

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
