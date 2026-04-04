#! python


import typer


cli = typer.Typer()




@cli.command()
def my_ip():
    import socket

    hostname = socket.gethostname()
    IPAddr = socket.gethostbyname(hostname)

    print("Your Computer Name is:", hostname)
    print("Your Computer IP Address is:", IPAddr)






@cli.command()
def neti():

    import netifaces

    for iface in netifaces.interfaces():
        if 'wi-fi' in iface.lower() or 'wlan' in iface.lower():
            addrs = netifaces.ifaddresses(iface).get(netifaces.AF_INET)
            if addrs:
                print("Your Wireless NIC IP Address is:", addrs[0]['addr'])
                break
    else:
        print("Wireless IP not found.")



if __name__ == "__main__":
    cli()



