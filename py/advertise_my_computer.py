#! python

import socket
from zeroconf import Zeroconf, ServiceInfo

zeroconf = Zeroconf()

service_type = "_http._tcp.local."
service_name = "My Python Service._http._tcp.local."

ip = socket.inet_aton("192.168.0.188")

info = ServiceInfo(
    type_=service_type,
    name=service_name,
    addresses=[ip],
    port=8080,
    properties={"path": "/"},
    server="tom.local.",
)

zeroconf.register_service(info)

print("Service registered. Open: http://192.168.0.188:8080")
print("If hostname resolves, also try: http://tom.local:8080")
input("Press enter to exit...\n")

zeroconf.unregister_service(info)
zeroconf.close()
