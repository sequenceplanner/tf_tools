# tf_tools

This suite enables manipulation of the /tf tree. It sonsists of:
1. `tf_lookup`: This package has a server that looks up /tf and maintains a buffer. Clients now don't have to lookup /tf as they can now request a lookup from this node. P.S. This probably only works for infrequent calls.
2. `tf_broadcast`: This package maintains active and static transforms in the /tf tree. Being the `main` node that holds the state, it loads the transforms from initial sscenario files, but also enables manipulation of the state through a service call.
3. `tf_scene`: This package holds the initial `scenario` files that are loaded when a scenario is launched.
4. `tf_bringup`: Launches scenarios, tools and examples.
5. `tf_sms`: The Scene Manipulation Service serves as the gateway to other tools, as it makes requests to change state, lookup tf, etc. Manipulating the /tf tree is done through this service.
6. `tf_tools_msgs`: Some messages for communication.
