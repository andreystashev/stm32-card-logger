# STM32 Card Logger

An STM32F103C8T6 firmware prototype that receives framed commands over USART2,
appends their binary data to an SD card through SPI and FatFs, and sends a
command-specific reply. The project contains STM32 HAL and FatFs sources plus a
STM32CubeMX configuration.

This is a hardware experiment and a starting point for a custom exchange
protocol. USART1 and USART3 are initialized, but the active command receiver is
USART2; the firmware does not automatically log arbitrary data from all UARTs.

## Hardware and connections

The source configures a 72 MHz system clock using an 8 MHz external crystal.
Use a compatible STM32F103C8T6 board, an ST-Link or other suitable programmer,
a FAT-formatted SD card, an SPI SD adapter, and a UART adapter with compatible
3.3 V logic levels. Connect a common ground.

| Signal | STM32 pin | Notes |
| --- | --- | --- |
| SD chip select | PA4 | GPIO output; defined by `CS_SD_PIN` in `Core/Inc/sd.h` |
| SPI2 SCK | PB13 | Clock |
| SPI2 MISO | PB14 | Card to MCU |
| SPI2 MOSI | PB15 | MCU to card |
| USART2 TX | PA2 | Command replies |
| USART2 RX | PA3 | Command input; do not connect SD chip select here |
| USART1 TX / RX | PA9 / PA10 | Initialized; separate experimental DMA code |
| USART3 TX / RX | PB10 / PB11 | Initialized; no active receive loop |
| Status output | PB1 | Toggles after a successful log write; legacy label `parachute` |
| Error LED | PC13 | Active-low LED used by the SPI driver |

Power the SD adapter according to its own specification. A bare SD card requires
3.3 V; do not assume that every adapter accepts 5 V or includes level shifting.

## Build and flash

The repository includes sources, a startup file, `STM32F103C8TX_FLASH.ld`, and
`sd_card.ioc`. It does **not** include a complete CubeIDE project, Makefile, or a
prebuilt firmware image.

1. Install STM32CubeIDE/STM32CubeMX and the STM32CubeF1 firmware package. The
   configuration records CubeMX 6.6.1 and STM32CubeF1 1.8.4.
2. Create a project for STM32F103C8Tx and use `sd_card.ioc` as the configuration
   reference. Retain the supplied HAL, FatFs, startup, linker, and application
   sources, including `Core/Inc/packet_buffer.h`.
3. Add `Core/Inc`, HAL/CMSIS headers, `FATFS/App`, `FATFS/Target`, and
   `Middlewares/Third_Party/FatFs/src` to the include paths; define
   `USE_HAL_DRIVER` and `STM32F103xB`.
4. Check the board clock, flash/RAM settings, and wiring, then build and flash
   using the IDE and your programmer.

Back up the application before regenerating code: some experimental functions
are outside CubeMX `USER CODE` sections. The `.ioc` also records USART1 at
230400 baud, whereas the checked-in `main.c` initializes all three USARTs at
460800 baud. Review generated changes before replacing these sources.

## Command protocol

Configure the command UART for **460800 baud, 8 data bits, no parity, one stop
bit, no hardware flow control**. Send a six-byte header followed by the payload
and two trailer bytes:

| Byte offset | Meaning in the current implementation |
| --- | --- |
| 0 | Reserved; currently not validated |
| 1–2 | Little-endian address; bit `0x20` must be set for command handling |
| 3 | Command code |
| 4–5 | Little-endian payload length, from 0 to 792 bytes |
| 6 onward | Payload, then two trailer bytes |

The trailer is stored but not checked as a checksum. Unknown commands and
addresses without bit `0x20` receive no reply and are not logged.

| Command | Reply | Data appended to `mywrite.txt` |
| --- | --- | --- |
| `0x01` | `01 02 03 04 05 06 07 08` | Complete request, then reply |
| `0x02` | Complete request echoed back | Complete request once |
| `0x03` | `01 02 03 04 05 06 07 01` | Complete request, then reply |

For example, `00 20 00 01 00 00 00 00` is a zero-payload `0x01` request. It
has no checksum protection and is only an example of the current framing.

The file is created when missing and opened for append for each handled request.
SD writes run in the main loop and finish before sending the reply. Reception
is rearmed before transmission; echo replies use the stored request copy so that
the next receive cannot overwrite bytes being sent. Requests must be paced: send
one complete packet and wait for its reply before sending the next packet.
Reception pauses during SD processing; continuous streams and simultaneous
traffic can lose bytes. UART2 errors reset reception to a new header. There is no timeout or robust stream
resynchronization for incomplete/corrupt frames. After an invalid frame, restart
the exchange from a known packet boundary; a reset may be required.

## Checks and limitations

Run the hardware-independent regression checks with a C99 compiler:

```sh
./tests/run.sh
```

They check packet bounds, request/reply log ordering, atomic rejection when a
buffer is full, SD data-response decoding, initialization failure, and the FatFs
disk adapter's block/byte addressing, multi-sector loops, and error propagation.
The SD/HAL interfaces are mocked; these checks do not verify a physical card,
UART timing, flashing, or a complete ARM firmware build.

The firmware remains a prototype: there is no power-loss recovery, the RTC is
reset at startup and FatFs timestamps return zero, and most fatal errors stop in
`Error_Handler`. Legacy directory/file-transfer helpers are experimental and are
not part of the active command path. In particular, the C stdio transfer helper
has no FatFs-backed `_open` implementation. Adapt and validate the protocol,
card compatibility, and error handling on your own hardware before relying on
recorded data.
