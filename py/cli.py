#! python


import typer
from zeroconf import Zeroconf, ServiceBrowser, ServiceInfo
import socket
import netifaces

cli = typer.Typer()




@cli.command()
def my_ip():
    """Get the computer's hostname and IP address."""

    hostname = socket.gethostname()
    IPAddr = socket.gethostbyname(hostname)

    print("Your Computer Name is:", hostname)
    print("Your Computer IP Address is:", IPAddr)




def _neti():
    for iface in netifaces.interfaces():
        print(iface)
        if 'wi-fi' in iface.lower() or 'wlan' in iface.lower():
            addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET)
            if addrs:
                print("Your Wireless NIC IP Address is:", addrs[0]['addr'])
                # break
                return addrs[0]['addr']
    else:
        print("Wireless IP not found.")

@cli.command()
def neti():
    """ Get the computer's wireless network interface IP address. """
    ip = _neti()
    if ip:
        print(f"Your Wireless NIC IP Address is: {ip}")

@cli.command()
def advertise_my_computer(ip_to_advertise: str):
    zeroconf = Zeroconf()

    service_type = "_http._tcp.local."
    service_name = "My Python Service._http._tcp.local."

    ip = socket.inet_aton(ip_to_advertise)

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


@cli.command()
def list_of_mdns_services():
    """" Show all mDNS services on the local network. """
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



if __name__ == "__main__":
    cli()




