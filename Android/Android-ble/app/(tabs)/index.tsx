import { useEffect, useState } from "react";

import { SafeAreaView } from "react-native-safe-area-context";

import { Button, PermissionsAndroid, Platform, Text, View } from "react-native";
import { BleManager, Device } from "react-native-ble-plx";

const manager = new BleManager();

export default function HomeScreen() {
  const [devices, setDevices] = useState<Device[]>([]);
  const [connectedDevice, setConnectedDevice] = useState<Device | null>(null);

  useEffect(() => {
    requestPermissions();
    return () => manager.destroy();
  }, []);
  // useEffect(() => {
  // requestPermissions();
  // return () => manager.destroy();
  // }, []);

  const requestPermissions = async () => {
    if (Platform.OS === "android") {
      await PermissionsAndroid.requestMultiple([
        PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
        PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
        PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT,
      ]);
    }
  };

  const scanForDevices = () => {
    setDevices([]);

    manager.startDeviceScan(null, null, (error, device) => {
      if (error) {
        console.log("Scan error:", error);
        return;
      }

      if (device && device.name) {
        setDevices((prev) => {
          if (prev.find((d) => d.id === device.id)) return prev;
          return [...prev, device];
        });
      }
    });

    // stop after 5s
    setTimeout(() => manager.stopDeviceScan(), 5000);
  };

  const connectToDevice = async (device: Device) => {
    try {
      manager.stopDeviceScan();
      const connected = await device.connect();
      await connected.discoverAllServicesAndCharacteristics();
      setConnectedDevice(connected);
      console.log("Connected to", connected.name);
    } catch (e) {
      console.log("Connection error:", e);
    }
  };
  return (
    <SafeAreaView>
      <Text style={{ fontSize: 32 }}>The Home Screen</Text>
      <Text>The Home Screen</Text>
      <View style={styles.fixToText}>
        <Button
          title="Run"
          onPress={() => {
            console.log("Run button pressed");
          }}
        />

        {/* <Button
          title="Connect a"
          color="#ff9422"
          onPress={() => bleConn.readValue()}
        />
        <Button
          title="Disconnect"
          color="#f194ff"
          onPress={() => Alert.alert("Right button pressed")}
        />
        <Button
          title="ReqPerm"
          color="#ff9422"
          onPress={() => bleConn.requestPermissions()}
        /> */}
      </View>
    </SafeAreaView>
  );
}

const styles = StyleSheet.create({
  fixToText: {
    flexDirection: "row",
    justifyContent: "space-between",
    // backgroundColor: "violet",
  },
  titleContainer: {
    flexDirection: "row",
    alignItems: "center",
    gap: 8,
  },
  stepContainer: {
    gap: 8,
    marginBottom: 8,
  },
  reactLogo: {
    height: 178,
    width: 290,
    bottom: 0,
    left: 0,
    position: "absolute",
  },
});
