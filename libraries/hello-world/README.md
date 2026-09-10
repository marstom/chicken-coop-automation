# HelloWorld

A minimal PlatformIO library that provides a hello-world message.

## Usage

Add the local library to your project's `platformio.ini`:

```ini
lib_deps =
    symlink://../../libraries/hello-world
```

Then use it in your application:

```cpp
#include <hello_world.h>

Serial.println(hello_world::message());
```

The current library version is `0.1.0`.
