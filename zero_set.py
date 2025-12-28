import serial
import time

class ST3215:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.003):
        self.ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        print(f"Connected to {port} at {baudrate} baud")

    def ping(self, servo_id: int) -> bool:
        length = 2
        instruction = 0x01
        checksum = (~(servo_id + length + instruction)) & 0xFF
        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, checksum])

        self.ser.reset_input_buffer()
        self.ser.write(packet)

        buffer = bytearray()
        start_time = time.time()

        while time.time() - start_time < 0.02:
            data = self.ser.read(32)
            if data:
                buffer.extend(data)

            if len(buffer) >= 6:
                for i in range(len(buffer) - 1):
                    if buffer[i] == 0xFF and buffer[i+1] == 0xFF:
                        if len(buffer) >= i + 6:
                            packet = buffer[i:i+6]
                            recv_id = packet[2]
                            return recv_id == servo_id
        return False

    def move(self, servo_id: int, position: int, speed: int = 0):
        """
        Move servo to a given position (0-4094) with optional speed (0-1023).
        """
        # Clamp values
        position = max(0, min(4094, position))
        speed = max(0, min(1023, speed))

        # Split into bytes
        pos_low = position & 0xFF
        pos_high = (position >> 8) & 0xFF
        speed_low = speed & 0xFF
        speed_high = (speed >> 8) & 0xFF

        # Packet parameters
        length = 7               # address + 4 bytes of data + instruction + servo_id
        instruction = 0x03       # WRITE_DATA
        address = 0x2A           # Goal position register

        checksum = (~(servo_id + length + instruction + address + pos_low + pos_high + speed_low + speed_high)) & 0xFF

        # Build packet: FF FF ID LEN INST ADDR POS_L POS_H SPD_L SPD_H CHECK
        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction,
                            address, pos_low, pos_high, speed_low, speed_high, checksum])
        self.ser.write(packet)
        self.ser.flush()

    def DefineMiddle(self, servo_id):
        """
        Set the current servo position as the middle (neutral) position (2048) and enable torque.
        """
        middle_pos = 2048

        # Move servo to middle
        self.move(servo_id, middle_pos, speed=0)  # speed=0 = immediate/hold position

        # Enable torque (write 128 to torque switch register 0x28)
        torque_enable = 128
        length = 4
        instruction = 0x03  # WRITE_DATA
        address = 0x28      # Torque switch register
        low_byte = torque_enable & 0xFF
        checksum = (~(servo_id + length + instruction + address + low_byte)) & 0xFF

        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, address, low_byte, checksum])
        self.ser.write(packet)
        self.ser.flush()

        





    def read_position(self, servo_id: int) -> int | None:
        """Read 12-bit current position (0-4094)"""
        length = 4
        instruction = 0x02
        address = 0x38  # Current location low byte
        data_len = 2
        checksum = (~(servo_id + length + instruction + address + data_len)) & 0xFF

        packet = bytearray([0xFF, 0xFF, servo_id, length, instruction, address, data_len, checksum])
        self.ser.reset_input_buffer()
        self.ser.write(packet)

        buffer = bytearray()
        start_time = time.time()

        while time.time() - start_time < 0.05:
            data = self.ser.read(32)
            if data:
                buffer.extend(data)

            for i in range(len(buffer) - 6):
                if buffer[i] == 0xFF and buffer[i+1] == 0xFF and buffer[i+2] == servo_id and buffer[i+3] == 4:
                    pos_low = buffer[i+5]
                    pos_high = buffer[i+6]
                    position = (pos_high << 8) | pos_low
                    return max(0, min(4094, position))
        return None






    def close(self):
        self.ser.close()

if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    controller = ST3215(port)
    servo_ids = [3,4]
    speed = 500
    pos = 1000

    

    try:
        for servo_id in servo_ids:
            if controller.ping(servo_id):
                print(f"✅ Servo {servo_id} is online")

                # Read position before moving
                
                # print(f"First position: {controller.read_position(servo_id)}")
                # controller.move(servo_id, pos, speed)
                # time.sleep(1.5)
                controller.DefineMiddle(servo_id)
                time.sleep(1.1)
                print(f"Current pos: {controller.read_position(servo_id)}")
                # # Move to middle position
                # middle = controller.middle_position()
                # print(f"Moving to middle position: {middle}")
                # controller.move(servo_id, middle, speed)
                # time.sleep(1.5)  # give time to reach position

                # # Read position after moving
                # current = controller.read_position(servo_id)
                # if current is not None:
                #     print(f"Position after move:  {current}")

            else:
                print(f"❌ Servo {servo_id} did not respond")

    finally:
        controller.close()