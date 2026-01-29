# firmware-common-cpp

A single-header C++ library for embedded systems (STM32, ESP32, etc.), designed to be portable and easy to integrate.

## Structure

*   `include/firmware_common.hpp`: The single header file containing the library code.
*   `tests/`: Unit tests using GoogleTest.

## Requirements

*   CMake 3.14+
*   C++17 compliant compiler

## Building and Testing

To build the tests, run the following commands:

```bash
mkdir build
cd build
cmake ..
make
ctest
```

## Integration

Since this is a header-only library, you can simply include the header file in your project:

1.  Copy `include/firmware_common.hpp` to your project's include directory.
2.  Include it in your source files:
    ```cpp
    #include "firmware_common.hpp"
    ```

Alternatively, if using CMake, you can add this directory as a subdirectory and link against the `firmware_common` interface target:

```cmake
add_subdirectory(path/to/firmware-common-cpp)
target_link_libraries(your_target PRIVATE firmware_common)
```

## Usage Example: Sending a CAN Message

Here is a basic example of how to construct, serialize, and encode a message for transmission over the CAN bus:

```cpp
#include "firmware_common.hpp"

using namespace firmware_common::can_bus;

void send_example() {
    // 1. Construct the message struct
    // Example: AirBrakesControl with 50.5% extension
    AirBrakesControlMessage msg = AirBrakesControlMessage::from_float(0.505f);

    // 2. Get the CAN frame ID
    // Requires node_type and node_id of the sender
    uint32_t frame_id = get_frame_id(msg, 10, 20);

    // 3. Initialize the Multi-Frame Encoder
    CanBusMultiFrameEncoder encoder(msg);

    // 4. Iterate through frames and send
    while (encoder.has_next()) {
        auto frame = encoder.next();
        
        // Send to your CAN hardware driver:
        // your_can_driver_send(frame_id, frame.data, frame.len);
    }
}
```

## Usage Example: Receiving CAN Bus Messages

The `CanBusMultiFrameDecoder` handles both single-frame and multi-frame messages. It reconstructs multi-frame messages and verifies their CRC.

```cpp
#include "firmware_common.hpp"
#include <iostream>

using namespace firmware_common::can_bus;

void receive_example() {
    // 1. Initialize the decoder
    CanBusMultiFrameDecoder decoder;

    // 2. When a frame is received from hardware:
    uint32_t received_id = 0x12345678; // Example ID
    uint8_t received_data[8] = { /* ... data ... */ };
    size_t received_len = 8;
    uint64_t timestamp_us = 1000000;

    // 3. Process the frame
    auto result = decoder.process_frame(received_id, received_data, received_len, timestamp_us);

    // 4. Check if a complete message was reconstructed
    if (result.has_value()) {
        const ReceivedCanBusMessage& msg = result.value();
        
        // Access the reconstructed message variant
        if (std::holds_alternative<NodeStatusMessage>(msg.message)) {
            const auto& node_status = std::get<NodeStatusMessage>(msg.message);
            std::cout << "Received NodeStatus, uptime: " << node_status.uptime_s << "s\n";
        } else if (std::holds_alternative<BaroMeasurementMessage>(msg.message)) {
            const auto& baro = std::get<BaroMeasurementMessage>(msg.message);
            std::cout << "Received Baro: " << baro.pressure() << " Pa\n";
        }
        // ... handle other message types ...
    }
}
```

