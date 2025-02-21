// Board and hardware specific configuration
#define MICROPY_HW_BOARD_NAME "SparkFun Thing Plus ESP32-S3"
#define MICROPY_HW_MCU_NAME "ESP32S3"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL     (1)

#define MICROPY_HW_I2C0_SCL            (9)
#define MICROPY_HW_I2C0_SDA            (8)

#define MICROPY_HW_SPI0_SCK  (12)
#define MICROPY_HW_SPI0_MOSI (11)
#define MICROPY_HW_SPI0_MISO (13)

#define MICROPY_HW_UART0_TX (43)
#define MICROPY_HW_UART0_RX (44)
