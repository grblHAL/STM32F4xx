# Custom PIO Boards
This directory contains custom PlatformIO MCU board definitions which are not present in the official PlatformIO registry. 

Why? The PIO STSTM32 project has not accepted any new boards since at least May 2022. Check out [this PR](https://github.com/platformio/platform-ststm32/pull/890) for example.

See also the [PlatformIO documentation](https://docs.platformio.org/en/latest/boards/custom.html) on how to use custom board definitions in your projects.

In short, you will need to add

```
[platformio]
boards_dir = platformio_boards
```

And then put your JSON board definition files in this directory.