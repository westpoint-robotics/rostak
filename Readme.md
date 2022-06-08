# rostak_bridge

Proxy CoT messages (XML strings) between ROS and a TAK environment

## Config

Install pytak using one of the [installation methods](https://github.com/ampledata/pytak#installation).

See the example launch file.  Just needs a url for the TAK endpoint.
If using a TLS transport, some environment variables need to be set.
See pytak's readme for more information.

## Topics

- `tak_tx`: transmits CoT xml strings on this topic to TAK
- `tak_rx`: receives CoT xml strings from TAK and publishes on this topic

## Secrets

TLS connections require a certficate and private key.
Secrets should not be included in source control.
The example.launch file provides a recommendation for passing in
the file paths as arguments, from which env vars are set.

## Acknowledgements

Based on the great work of [pytak](https://github.com/ampledata/pytak).
