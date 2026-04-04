from zeroconf import Zeroconf, ServiceBrowser

class MyListener:
    def add_service(self, zeroconf, service_type, name):
        print(f"[+] Service added: {name}")
        info = zeroconf.get_service_info(service_type, name)
        if info:
            print(f"    Address: {info.parsed_addresses()}")
            print(f"    Port: {info.port}")
            print(f"    Properties: {info.properties}")

    def remove_service(self, zeroconf, service_type, name):
        print(f"[-] Service removed: {name}")

    def update_service(self, zeroconf, service_type, name):
        print(f"[*] Service updated: {name}")

zeroconf = Zeroconf()
listener = MyListener()

# Browse HTTP services
browser = ServiceBrowser(zeroconf, "_http._tcp.local.", listener)

input("Press enter to exit...\n")
zeroconf.close()
