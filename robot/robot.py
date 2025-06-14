from contextlib import contextmanager
import serial
import time

COM_PORT = 'COM6'
BAUD_RATE = 115200
TIMEOUT = 2

# === Context manager for serial connection ===
@contextmanager
def connect_serial(port=COM_PORT, baud=BAUD_RATE, timeout=TIMEOUT, graceful_exit_flag=True):
    ser = None
    try:
        ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)
        print(f"Connected to {port} at {baud} baud.")
        yield ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        yield None
    finally:
        if ser and ser.is_open:
            if graceful_exit_flag:
                print("Graciously exiting...")
                perform_graceful_exit(ser)
            ser.close()
            print(f"Closed serial port {port}.")

def send_gcode(ser, cmd, wait_for="ok", timeout=10):
    ser.reset_input_buffer()
    ser.write((cmd.strip() + '\n').encode('utf-8'))
    time.sleep(0.05)

    response = []
    start_time = time.time()

    while time.time() - start_time < timeout:
        if ser.in_waiting:
            line = ser.readline().decode('utf-8').strip()
            if line:
                response.append(line)
                if wait_for and line.lower() == wait_for.lower():
                    print_response(response)
                    return True  # Success
    else:
        response.append("(timeout waiting for response)")
        print_response(response)
        return False  # Timed out


# === Print response lines ===
def print_response(response):
    if response:
        print('\n'.join(f"> {line}" for line in response))
    else:
        print("> (no response)")

# === Configure axes ===
def configure_axes(ser, F=200, A=400, J=900, S=15, E=1):
    # send_gcode(ser,"M203 J{J}")
    # send_gcode(ser,"M204 A{A}")
    # send_gcode(ser,"M210 F{F} A{A} J{J} S{S} E{E}")
    template = "M6{axis} D1 E{E} I0 S{S} R5 U8.333 P178 Q0 H45 A{A} J{J} F{F}"
    for axis in range(3):  # axis 0 = X, 1 = Y, 2 = Z
        cmd = template.format(axis=axis, F=F, A=A, J=J, S=S, E=E)
        send_gcode(ser, cmd)

# === Perform graceful shutdown ===
def perform_graceful_exit(ser):
    send_gcode(ser, "G28")
    disable_effector(ser)
    
def enable_effector(ser):
    send_gcode(ser, "G01 W30 U50")  

def disable_effector(ser):
    send_gcode(ser, "G01 W30 U140")  
    
def move_to_position(ser, x=None, y=None, z=None):
    """Move to a specified position in 3D space."""
    cmd="G01"
    if x!=None:
        cmd += f" X{x} "
    if y!=None:
        cmd += f" Y{y} "
    if z!=None:
        cmd += f" Z{z} "
    print(cmd)
    send_gcode(ser, cmd)
    
def pick(ser):
    enable_effector(ser)
    send_gcode(ser, "G04 P3000")
    print("PICK!")
    send_gcode(ser, "G01 Z-960 F30")
    send_gcode(ser, "G04 P1000")
    
    send_gcode(ser, "G01 Z-900 F30")
    
def place(ser):
    enable_effector(ser)
    send_gcode(ser, "G04 P1000")
    send_gcode(ser, "G01 Z-960 0 F30")
    
    print("PLACE!")
    send_gcode(ser, "G04 P3000")
    send_gcode(ser, "G01 Z-900 F30")
