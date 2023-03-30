# ESP32 MCP2515 CAN interface library in C, for ESP-IDF framework.

it's a fork of [MCP2515 CAN interface library in C++ for ESP-IDF](https://github.com/zeroomega/esp32-mcp2515)

CAN-BUS is a common industrial bus because of its long travel distance, medium communication speed and high reliability. It is commonly found on modern machine tools and as an automotive diagnostic bus. This CAN-BUS Shield gives your esp32/esp8266 CAN-BUS capibility. With an OBD-II converter cable added on and the OBD-II library imported, you are ready to build an onboard diagnostic device or data logger.

- Implements CAN V2.0B at up to 1 Mb/s
- SPI Interface up to 10 MHz
- Standard (11 bit) and extended (29 bit) data and remote frames
- Two receive buffers with prioritized message storage

**Contents:**
* [Hardware](#hardware)
   * [CAN Shield](#can-shield)
* [Software Usage](#software-usage)
   * [Initialization](#initialization)
   * [Frame data format](#frame-data-format)
   * [Send Data](#send-data)
   * [Receive Data](#receive-data)
   * [Set Receive Mask and Filter](#set-receive-mask-and-filter)
   * [Examples](#examples)

# Hardware:

## CAN Shield

The following code samples uses the CAN-BUS Shield, wired up as shown:

Component References:
* [MCP2515](https://www.microchip.com/wwwproducts/en/MCP2515) Stand-Alone CAN Controller with SPI Interface
* [MCP2551](https://www.microchip.com/wwwproducts/en/MCP2551) High-speed CAN Transceiver - pictured above, however "not recommended for new designs"
* [MCP2562](https://www.microchip.com/wwwproducts/en/MCP2562) High-speed CAN Transceiver with Standby Mode and VIO Pin - an updated tranceiver since the _MCP2551_ (requires different wiring, read datasheet for example, also [here](https://fragmuffin.github.io/howto-micropython/slides/index.html#/7/5))
* [TJA1055](https://www.nxp.com/docs/en/data-sheet/TJA1055.pdf) Fault-tolerant low speed CAN Transceiver. Mostly used in vehicles.


# Software Usage:

## Initialization

The available modes are listed as follows:
```C
MCP2515_setNormalMode();
MCP2515_setLoopbackMode();
MCP2515_setListenOnlyMode();
```

The available baudrates are listed as follows:
```C
enum CAN_SPEED {
    CAN_5KBPS,
    CAN_10KBPS,
    CAN_20KBPS,
    CAN_31K25BPS,
    CAN_33KBPS,
    CAN_40KBPS,
    CAN_50KBPS,
    CAN_80KBPS,
    CAN_83K3BPS,
    CAN_95KBPS,
    CAN_100KBPS,
    CAN_125KBPS,
    CAN_200KBPS,
    CAN_250KBPS,
    CAN_500KBPS,
    CAN_1000KBPS
};
```


Example of initialization

```C
bool SPI_Init(void)
{
	printf("Hello from SPI_Init!\n\r");
	esp_err_t ret;
	//Configuration for the SPI bus
	spi_bus_config_t bus_cfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz = 0 // no limit
	};

	// Define MCP2515 SPI device configuration
	spi_device_interface_config_t dev_cfg = {
		.mode = 0, // (0,0)
		.clock_speed_hz = 10000000, // 10mhz
		.spics_io_num = PIN_NUM_CS,
		.queue_size = 128
	};

	// Initialize SPI bus
	ret = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
	ESP_ERROR_CHECK(ret);

	// Add MCP2515 SPI device to the bus
	ret = spi_bus_add_device(SPI2_HOST, &dev_cfg, &MCP2515_Object->spi);
	ESP_ERROR_CHECK(ret);

	return true;
}

void CAN_Init(void)
{
	MCP2515_init();
	SPI_Init();
	MCP2515_reset();
	MCP2515_setBitrate(CAN_1000KBPS, MCP_8MHZ);
	MCP2515_setNormalMode();

	xTaskCreatePinnedToCore(CAN_Module_RX_Task_Polling, "CAN_Module_RX_Task_Polling", 16384, NULL, 20, NULL, 1);
}
```

The available clock speeds are listed as follows:

```C
enum CAN_CLOCK {
    MCP_20MHZ,
    MCP_16MHZ,
    MCP_8MHZ
};
```

Default value is MCP_16MHZ, in my example it was 8MHZ.

Note: To transfer data on high speed of CAN interface via UART dont forget to update UART baudrate as necessary.

## Frame data format

Library uses Linux-like structure to store can frames;

```C
typedef struct can_frame {
    canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
} CAN_FRAME_t[1], *CAN_FRAME;
```

For additional information see [SocketCAN](https://www.kernel.org/doc/Documentation/networking/can.txt)

## Send Data

```C
ERROR_t MCP2515_sendMessage(const TXBn_t txbn, const CAN_FRAME frame);
ERROR_t MCP2515_sendMessageAfterCtrlCheck(const CAN_FRAME frame);
```

This is a function to send data onto the bus.

For example, In the 'send' example, we have:

```C
CAN_FRAME frame;
frame->can_id = 0x000;
frame->can_dlc = 4;
frame->data[0] = 0xFF;
frame->data[1] = 0xFF;
frame->data[2] = 0xFF;
frame->data[3] = 0xFF;

/* send out the message to the bus and
tell other devices this is a standard frame from 0x00. */
MCP2515_sendMessage(&frame);
```

```C
CAN_FRAME frame;
frame->can_id = 0x12345678 | CAN_EFF_FLAG;
frame->can_dlc = 2;
frame->data[0] = 0xFF;
frame->data[1] = 0xFF;

/* send out the message to the bus using second TX buffer and
tell other devices this is a extended frame from 0x12345678. */
MCP2515_sendMessage(MCP2515::TXB1, &frame);
```



## Receive Data

The following function is used to receive data on the 'receive' node:

```C++
ERROR_t MCP2515_readMessage(const RXBn_t rxbn, const CAN_FRAME frame);
ERROR_t MCP2515_readMessageAfterStatCheck(const CAN_FRAME frame);
```

In conditions that masks and filters have been set. This function can only get frames that meet the requirements of masks and filters.

You can choise one of two method to receive: interrupt-based and polling

Example of poll read

```C
CAN_FRAME frame;

void loop() {
    if (MCP2515_readMessage(&frame) == ERROR_OK) {
        // frame contains received message
    }
}
```

Example of interrupt based read

```C
bool interrupt = false;
CAN_FRAME frame;

void irqHandler() {
    interrupt = true;
}

void setup() {
    ...
    attachInterrupt(0, irqHandler, FALLING);
}

void loop() {
    if (interrupt) {
        interrupt = false;

        uint8_t irq = MCP2515_getInterrupts();

        if (irq & CANINTF_RX0IF) {
            if (MCP2515_readMessage(RXB0, &frame) == ERROR_OK) {
                // frame contains received from RXB0 message
            }
        }

        if (irq & CANINTF_RX1IF) {
            if (MCP2515_readMessage(RXB1, &frame) == ERROR_OK) {
                // frame contains received from RXB1 message
            }
        }
    }
}
```


## Set Receive Mask and Filter

There are 2 receive mask registers and 5 filter registers on the controller chip that guarantee you get data from the target device. They are useful especially in a large network consisting of numerous nodes.

We provide two functions for you to utilize these mask and filter registers. They are:

```C
ERROR_t MCP2515_setFilterMask(const MASK_t num, const bool ext, const uint32_t ulData);
ERROR_t MCP2515_setFilter(const RXF_t num, const bool ext, const uint32_t ulData);
```

**MASK mask** represents one of two mask **MCP2515::MASK0** or **MCP2515::MASK1**

**RXF num** represents one of six acceptance filters registers from **MCP2515::RXF0** to **MCP2515::RXF5**

**ext** represents the status of the frame. **false** means it's a mask or filter for a standard frame. **true** means it's for a extended frame.

**ulData** represents the content of the mask of filter.



For more information, please refer to [wiki page](http://www.seeedstudio.com/wiki/CAN-BUS_Shield) .

----

This software is written by loovee ([luweicong@seeed.cc](luweicong@seeed.cc "luweicong@seeed.cc")) for seeed studio
Updated by Dmitry ([https://github.com/autowp](https://github.com/autowp "https://github.com/autowp"))
Adapted for use on esp32/esp8266 by dedal.qq ([https://github.com/dedalqq](https://github.com/dedalqq "https://github.com/dedalqq"))
Adapted for use on ESP32 & ESP-IDF by dogualpay ([https://github.com/dogualpay](https://github.com/dogualpay "https://github.com/dogualpay"))
and is licensed under [The MIT License](http://opensource.org/licenses/mit-license.php). Check [LICENSE.md](LICENSE.md) for more information.

Contributing to this software is warmly welcomed. You can do this basically by
[forking](https://help.github.com/articles/fork-a-repo), committing modifications and then
[pulling requests](https://help.github.com/articles/using-pull-requests) (follow the links above
for operating guide). Adding change log and your contact into file header is encouraged.
Thanks for your contribution.

Seeed Studio is an open hardware facilitation company based in Shenzhen, China.
Benefiting from local manufacture power and convenient global logistic system,
we integrate resources to serve new era of innovation. Seeed also works with
global distributors and partners to push open hardware movement.
