


import socket
from zeroconf import Zeroconf, ServiceInfo

zeroconf = Zeroconf()

service_type = "_http._tcp.local."
service_name = "My Python Service._http._tcp.local."

ip = socket.inet_aton("192.168.0.109")  # change to your IP

info = ServiceInfo(
    service_type,
    service_name,
    addresses=[ip],
    port=80,
    properties={"path": "/hello"},
    # server="my-python.local."
    server="my-python.local."
)

zeroconf.register_service(info)

print("Service registered. Press enter to exit...")
input()

zeroconf.unregister_service(info)
zeroconf.close()