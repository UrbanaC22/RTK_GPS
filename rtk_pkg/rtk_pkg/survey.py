import serial
import threading

def calculate_checksum(data: str) -> str:
    """Calculate XOR checksum for given data (excluding $ and *)."""
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def read_from_gps(ser):
    """Reads data from the GPS module and prints messages that start with $."""
    while True:
        try:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('$'):
                print(f"Received: {line}")
        except Exception as e:
            print(f"Error reading from GPS: {e}")

def main():
    baud_rate = 115200
    
    try:
        ser = serial.Serial("/dev/ttyTHS1", baud_rate, timeout=1)
    except Exception as e:
        print(f"Error opening serial port: {e}")
        return
    
    threading.Thread(target=read_from_gps, args=(ser,), daemon=True).start()
    
    print("Type GPS commands to send. Press Ctrl+C to exit.")
    try:
        while True:
            user_input = input("Send: ").strip()
            if user_input:
                if not user_input.startswith('$'):
                    print("Error: Command must start with '$'")
                    continue
                
                command = user_input[1:]  # Remove leading $
                checksum = calculate_checksum(command)
                full_command = f"${command}*{checksum}\r\n"
                
                ser.write(full_command.encode('utf-8'))
                print(f"Sent: {full_command.strip()}")
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()

if __name__ == "__main__":
    main()



