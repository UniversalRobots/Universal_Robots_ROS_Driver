# Check common firewalls on Linux

When using this driver with a robot it is important that the robot can establish a connection to the
machine running this driver on a couple of ports. In case there is a firewall active without a
special configuration it will most likely block this connection.

In order for this driver to work correctly, the firewall has to allow connections to the ports
**50001, 50002 and 50003**. Obviously, as soon as you change the port arrangement manually, any firewall rules
have to be changed accordingly.

**NOTE: The steps shown in this tutorial will get your machine running with a robot connecting to
it. It is not a full guide to Linux firewalls. Changing your firewall settings is potentially
opening security holes in your local machine. In case of doubt, ask your local system administrator
for advice before altering your firewall settings.**

Changing your firewall settings will most likely require root (sudo) access to your local machine.
If you don't have this, you'll have to talk to your local system administrator.

## UFW
Probably the most common firewall on Ubuntu systems is the [Uncomplicated Firewall (UFW)](https://help.ubuntu.com/community/UFW)

To check whether it is active, run

```
sudo ufw status
```

In case the firewall is active, you will get the output

```
Status: active

# possible list of added rules
```

If it is inactive, you will get `Status: inactive` or even `ufw: command not found` if it isn't
installed, at all.


### Add rules for driver
To add rules for the `ur_robot_driver`, run

```bash
ROBOT_IP=192.168.56.101 # adapt to your particular robot_ip
sudo ufw allow from $ROBOT_IP to any port 50001
sudo ufw allow from $ROBOT_IP to any port 50002
sudo ufw allow from $ROBOT_IP to any port 50003
```

If you want to change your robot's IP address regularly, you can skip the IP address and simply run

```bash
sudo ufw allow 50001
sudo ufw allow 50002
sudo ufw allow 50003
```

## firewalld
Another common firewall on Linux is `firewalld`. An overview of its status (if installed) can be
seen using

```
sudo firewall-cmd --list-all
public (active)
  target: default
  icmp-block-inversion: no
  interfaces: eth0
  sources:
  services: dhcpv6-client http https mysql ssh
  ports:
  protocols:
  masquerade: no
  forward-ports:
  source-ports:
  icmp-blocks:
  rich rules:
```

To allow connections to the driver use

```
sudo firewall-cmd --permanent --zone=public --add-port=50001/tcp
sudo firewall-cmd --permanent --zone=public --add-port=50002/tcp
sudo firewall-cmd --permanent --zone=public --add-port=50003/tcp
```

Note: `firewalld` uses the concept of different trust zones. In the example above we modified the
default `public` zone. Depending on your local setup it might make more sense to use a different
zone.


## iptables
Probably also installed on most Ubuntu systems is `iptables`. It's configuration is not as simple as
for example UFW, which is why we only link to the [upstream
documentation](https://linux.die.net/man/8/iptables) here. If you have problems establishing a
connection, it might be worth checking your iptables setup.
