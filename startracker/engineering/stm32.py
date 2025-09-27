import serial


def listports():
    import serial.tools.list_ports

    print("Available serial ports:")
    for port in serial.tools.list_ports.comports():
        print(f"  {port.device}")


def read_stm32_serial(port="/dev/ttyACM0", baudrate=115200, timeout=1):
    with serial.Serial(port, baudrate, timeout=timeout) as ser:
        print(f"Connected to {port} at {baudrate} baud.")
        try:
            while True:
                packet = ser.readline()
                if packet:
                    print(packet.decode(errors="replace").strip())
        except KeyboardInterrupt:
            print("Serial reading stopped.")


if __name__ == "__main__":
    listports()
    read_stm32_serial()
