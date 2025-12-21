import { Alert, Button, StyleSheet, Text, View } from "react-native";

import { SafeAreaView } from "react-native-safe-area-context";
import { connectBle } from "../connectBle";

export default function HomeScreen() {
  const bleConn = connectBle();
  return (
    <SafeAreaView>
      <Text style={{ fontSize: 32 }}>The Home Screen</Text>
      <Text>The Home Screen</Text>
      <Text>The Home Screen</Text>
      <Text>The Home Screen</Text>
      <View style={styles.fixToText}>
        <Button
          title="Run"
          onPress={() => {
            console.log("Run button pressed");
          }}
        />

        <Button
          title="Connect"
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
        />
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
