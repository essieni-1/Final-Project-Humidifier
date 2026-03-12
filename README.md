# ESP32-S3 Smart Humidifier 

An automated environmental control system built using the **ESP-IDF** framework on the **ESP32-S3**. This project integrates real-time sensors and a multi-tasking architecture to maintain humidity levels based on a user-defined physical interface.

##  Features
- **Three Operational Modes:**
  - `OFF`: System standby.
  - `ON`: Continuous humidification (Manual override).
  - `AUTO`: Intelligent regulation based on a dynamic humidity target.
- **Dynamic Threshold Adjustment:** A physical potentiometer allows real-time adjustment of the humidity target (20% to 80%) without rebooting.
- **Failsafe Water Guard:** A UART-driven water level sensor automatically halts operation and alerts the user if the reservoir is empty.
- **Informative UI:** 16x2 LCD display showing live humidity ($H\%$) and current target ($T\%$).

---

## Hardware Components

| Component | Function | Interface |
| :--- | :--- | :--- |
| **ESP32-S3** | Main Microcontroller | N/A |
| [**DHT20**](https://www.digikey.com/en/products/detail/sparkfun-electronics/18364/14635373) | Humidity & Temperature Sensor | I2C |
| [**Piezo Atomizer**](https://www.bestmodulescorp.com/en/bm52o5221-1.html) | Mist Generation & Water Level Detection | UART |
| **16x2 LCD (HD44780)** | User Interface | GPIO (4-bit mode) |
| **Potentiometer** | Threshold Setting | ADC (Analog) |
| **Push Button** | Mode Selection | GPIO (Pull-up) |

---


---

##  Software Logic

The system utilizes four distinct FreeRTOS tasks to maintain high responsiveness:

1. **`humidity_task`**: Polls the DHT20 sensor over I2C every 2 seconds.
2. **`water_guard_task`**: Monitors the UART feedback from the water sensor. Includes a 3-second fail-safe to prevent atomizer damage if communication is lost.
3. **`button_task`**: Debounces the mode button to cycle between OFF, ON, and AUTO.
4. **`control_task`**: The primary logic loop. It calculates the target threshold from the ADC and manages the physical output and LCD screen using a **Mutex** for thread-safe display access.

### Threshold Calculation
The threshold is mapped from the 12-bit ADC raw value (0-4095) using the following formula:

$$Threshold = MIN + \frac{Raw \times (MAX - MIN)}{4095}$$

---

