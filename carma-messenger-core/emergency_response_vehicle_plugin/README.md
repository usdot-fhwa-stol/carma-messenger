# emergency_response_vehicle_plugin
The Emergency Response Vehicle (ERV) Plugin is a standalone ROS2 node in CARMA Messenger and is intended to be run on an ERV. The primary role of this node is to collect relevant ERV data (vehicle position, siren and lights status, etc.), assemble the collected data into a BSM, and publish it at a configurable rate. The node collects latitude, longitude, and velocity data from the Torc Pinpoint Driver Node, and the ego vehicle’s emergency sirens and lights status from a UDP socket listening to incoming messages from a connected Raspberry Pi. Additionally, this node collects the ERV’s route destination points from a route CSV file stored on the host PC based on interactions with the CARMA Messenger Emergency Response Web UI widget.

Link to detailed design document on Confluence: [Click Here](https://usdot-carma.atlassian.net/wiki/spaces/CRMMSG/pages/2371321873/Detailed+Design+-+Emergency+Response+Vehicle+Plugin)

