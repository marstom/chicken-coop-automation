
// ------------- BLE support -------------
// UUIDs
// const char *deviceServiceUuid = "19b10000-e8f2-537e-4f6c-d104768a1214";
// const char *deviceServiceRequestCharacteristicUuid = "19b10001-e8f2-537e-4f6c-d104768a1214";
// const char *deviceServiceResponseCharacteristicUuid = "19b10002-e8f2-537e-4f6c-d104768a1214";
// import { Device } from "react-native-ble-plx";
import { PermissionsAndroid, Platform } from "react-native";
import { Device } from "react-native-ble-plx";


export const connectBle = () => {
//   const [devices, setDevices] = useState<Device[]>([]);
//   const [connectedDevice, setConnectedDevice] = useState<Device | null>(null);
    const SERVICE_UUID = "19b10000-e8f2-537e-4f6c-d104768a1214";
    const CHAR_UUID = "19b10001-e8f2-537e-4f6c-d104768a1214";

    const readValue = async (device: Device) => {
    const characteristic = await device.readCharacteristicForService(
        SERVICE_UUID,
        CHAR_UUID
    );
    console.log("Value:", characteristic.value); // base64
    };

    const writeValue = async (device: Device) => {
        await device.writeCharacteristicWithResponseForService(
            SERVICE_UUID,
            CHAR_UUID,
            Buffer.from("hello").toString("base64")
        );
    }



      const requestPermissions = async () => {
        if (Platform.OS === "android") {

        await PermissionsAndroid.requestMultiple([
            PermissionsAndroid.PERMISSIONS.ACCESS_FINE_LOCATION,
            PermissionsAndroid.PERMISSIONS.BLUETOOTH_SCAN,
            PermissionsAndroid.PERMISSIONS.BLUETOOTH_CONNECT
        ]);
        }
  };

    const scanForDevices = () => {

            // manager.startDeviceScan(null, null, (error, device) => {

            // });
        //     if (error) {
        //         console.log("Scan error:", error);
        //         return;
        //     }

        //     if (device && device.name) {
        //         console.log("Found device:", device.name);
                
        //     }
        // }));
    };

    return {
        readValue,
        writeValue,
        requestPermissions,
        scanForDevices
    };
}