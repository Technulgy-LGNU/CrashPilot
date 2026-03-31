# CrashPilot Protocol

The CrashPilot will send a unicast message to each robot with
the task the robot should perform.

Path planning should be done on the robot and implementation for
each task.


The packets from the robot to the CrashPilot are not that important,
but I would like for all of us to try to implement them.

## Robot IPs
Please configure static ips on your robots in the following
subnet (only important for production, you can choose different
ips for testing):

```bash
# Team Faabs
10.0.64.101 - 10.0.64.120

# Team LNX
10.0.64.121 - 10.0.64.140

# Team ZG24
10.0.64.141 - 10.0.64.160


# Subnet Mask
255.255.255.0

# Gateway
10.0.64.1
```

For personal computers, there is a DHCP Server for 54 clients, and
we provide 6 additional LAN Ports for each team.

You can configure the ips the CrashPilot uses in the C`P_IPs.toml` file (WIP).
