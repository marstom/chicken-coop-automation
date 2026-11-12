This is libraries common for all the projects in this repository. 
It is intended for project specific (private) libraries.


How to generate compile_commands.json file for ESP32 projects in PlatformIO:
1. Open my project - temperature sensor for example, then `pio run -e esp32-c3-chicken-rmaker -t compiledb`
2. create .vscode/settings.json file in libraries folder with the following content:
```json
{
  "C_Cpp.default.compileCommands": "${workspaceFolder}/../chicken-coop/compile_commands.json"
}
```