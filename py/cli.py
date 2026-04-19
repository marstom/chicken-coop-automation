#! python


import time

import typer
from zeroconf import Zeroconf, ServiceBrowser, ServiceInfo
import socket
import netifaces
import asyncio
import aiomqtt

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

@cli.command()
def sub(
    broker: str = "raspberrypi.local",
    port: int = 1883,
    topic: str = "coop/bme280/#",
    username: str | None = 'admin',
    password: str | None = 'admin',
):
    import paho.mqtt.client as mqtt
    from urllib.parse import urlparse

    # paho-mqtt expects a hostname/IP, not a URL.
    parsed_broker = urlparse(broker)
    mqtt_host = parsed_broker.hostname or broker.rstrip("/")

    def on_connect(client, userdata, flags, reason_code, properties=None):
        print("Connected with result code", reason_code)
        reason_text = str(reason_code).lower()
        is_auth_error = (
            reason_text == "not authorized"
            or getattr(reason_code, "value", None) == 5
            or reason_code == 5
        )
        if is_auth_error:
            typer.echo(
                "MQTT authentication failed: bad username or password.",
                err=True,
            )
            client.disconnect()
            return
        client.subscribe(topic)

    def on_message(client, userdata, msg):
        print(f"[{msg.topic}] {msg.payload.decode()}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    if username:
        client.username_pw_set(username, password)

    client.on_connect = on_connect
    client.on_message = on_message

    print(f"Connecting to MQTT broker {mqtt_host}:{port}, topic: {topic}")
    try:
        client.connect(mqtt_host, port, 60)
    except Exception as exc:
        typer.echo(
            "MQTT connection failed. Use a hostname/IP like "
            "'raspberrypi.local' or '192.168.x.x', not 'http://...'. "
            f"Original error: {exc}",
            err=True,
        )
        raise typer.Exit(code=1)

    client.loop_forever()


@cli.command()
def mqtt_send():
    """
    Send message to a topic
    """

    # file: send_mqtt.py
    import paho.mqtt.client as mqtt

    BROKER = "raspberrypi.local"
    # BROKER = "192.168.0.103"
    PORT = 1883
    TOPIC = "coop/relay/1/set"
    MESSAGE = "Hello world"

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set("admin", "admin")
    client.connect(BROKER, PORT, 60)
    client.publish(TOPIC, MESSAGE)
    client.disconnect()
    print("Sent")


@cli.command()
def coop_receive_temperature():
    import paho.mqtt.client as mqtt

    BROKER = "raspberrypi.local"
    # BROKER = "192.168.0.103"
    PORT = 1883
    TOPIC = "coop/bme280/temperature"

    def on_message(client, userdata, msg):
        print(f"[{msg.topic}] {msg.payload.decode()}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set("admin", "admin")
    client.connect(BROKER, PORT, 60)
    client.subscribe(TOPIC)
    client.on_message = on_message
    client.loop_forever()

@cli.command()
def receive_debug_logs(topic: str = "coop/log/mydebug"):
    """ Debug tools """
    import paho.mqtt.client as mqtt

    BROKER = "raspberrypi.local"
    PORT = 1883

    def on_message(client, userdata, msg):
        print(f"[{msg.topic}] {msg.payload.decode()}")

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.username_pw_set("admin", "admin")
    client.connect(BROKER, PORT, 60)
    client.subscribe(topic)
    client.on_message = on_message
    client.loop_forever()


@cli.command()
def receive_mqtt_async(topic: str = "coop/log/mydebug"):
    async def main():
        async with aiomqtt.Client("raspberrypi.local", username="admin", password="admin") as client:
            await client.subscribe(topic)
            async for message in client.messages:
                print(f"[{message.topic}] {message.payload.decode()}")

    asyncio.run(main())

def topics():
    """
        // MQTT stuff.
    inline constexpr const char *MDNS_HOSTNAME = "chicken";
    inline constexpr const char *THINGNAME = "esp32-c3-coop-temp-amonia-sensor";
    inline constexpr const char *PREFIX = "coop/";
    inline constexpr const char *BME_TEMPERATURE_TOPIC = "coop/bme280/temperature";
    inline constexpr const char *BME_PRESSURE_TOPIC = "coop/bme280/pressure";
    inline constexpr const char *BME_HUMIDITY_TOPIC = "coop/bme280/humidity";
    inline constexpr const char *BME_ALTITUDE_TOPIC = "coop/bme280/altitude";
    inline constexpr const char *AMONIA_SENSOR_TOPIC = "coop/amonia/raw";
    inline constexpr const char *MQTT_LOG_TOPIC = "coop/log/mydebug";
    inline constexpr const char *STATUS_TOPIC = "coop/status/read";
    inline constexpr const char *RELAY_1_SET_TOPIC = "coop/relay/1/set";
    
    """

if __name__ == "__main__":
    cli()
