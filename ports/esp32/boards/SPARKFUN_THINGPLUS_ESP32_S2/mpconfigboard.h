// Board and hardware specific configuration

#define MICROPY_HW_BOARD_NAME "SparkFun Thing Plus ESP32-S2"
#define MICROPY_HW_MCU_NAME "ESP32S2"

#define MICROPY_PY_BLUETOOTH (0)
#define MICROPY_HW_ENABLE_SDCARD (0)

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL     (1)

#define MICROPY_HW_I2C0_SCL            (2)
#define MICROPY_HW_I2C0_SDA            (1)

#define MICROPY_HW_SPI0_SCK  (36)
#define MICROPY_HW_SPI0_MOSI (35)
#define MICROPY_HW_SPI0_MISO (37)

#define MICROPY_HW_UART0_TX (33)
#define MICROPY_HW_UART0_RX (34)
