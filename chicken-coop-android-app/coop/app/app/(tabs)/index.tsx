import { useEffect, useRef } from "react";
import { Image } from "expo-image";
import { Button, Pressable, TextInput } from "react-native";
import { BleManager } from "react-native-ble-plx";

import ParallaxScrollView from "@/components/parallax-scroll-view";
import { ThemedText } from "@/components/themed-text";
import { ThemedView } from "@/components/themed-view";

export default function HomeScreen() {
  const manager = useRef(new BleManager()).current;

  useEffect(() => {
    return () => {
      manager.destroy();
    };
  }, [manager]);

  const scanBluetooth = () => {
    console.log("Scanning...");

    manager.startDeviceScan(null, null, (error, device) => {
      if (error) {
        console.log("BLE error:", error);
        return;
      }

      if (device) {
        console.log("Found:", device.name, device.id);
      }
    });
  };

  return (
    <ParallaxScrollView
      headerBackgroundColor={{ light: "#A1CEDC", dark: "#1D3D47" }}
      headerImage={
        <Image
          source={require("@/assets/images/lock.avif")}
          style={{
            height: 178,
            width: 290,
          }}
        />
      }
    >
      <ThemedView
        style={{
          flexDirection: "column",
          gap: 12,
        }}
      >
        <ThemedText type="title">Welcome!</ThemedText>

        <TextInput placeholder="Password" />

        <Button title="asdf" onPress={() => console.log("Button pressed")} />

        <Pressable onPress={scanBluetooth}>
          <ThemedText>Scan Bluetooth</ThemedText>
        </Pressable>
      </ThemedView>
    </ParallaxScrollView>
  );
}
