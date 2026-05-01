import time
import serial
import serial.tools.list_ports
import numpy as np

# ================= CONFIG =================
BAUD_RATE    = 115200
SERIAL_PORT  = None       # None = auto-detect, or set e.g. "COM3"

N_AXES       = 3
AXIS_NAMES   = ["X", "Y", "Z"]  # all three sensors present

# PWM sweep — goes negative and positive to get full bidirectional response
PWM_STEPS    = list(range(-255, -29, 15)) + \
               list(range(-30,   31, 5)) + \
               list(range( 30,  256, 15))
SETTLE_MS    = 400         # ms to wait after setting PWM before reading
ZERO_SETTLE  = 800         # ms to wait after zeroing before reading ambient

# ================= SERIAL =================
def find_port():
    ports = serial.tools.list_ports.comports()
    for p in ports:
        if any(x in p.description for x in ["Arduino", "CH340", "USB Serial"]):
            return p.device
    return ports[0].device if ports else None

def send(ser, msg):
    ser.write((msg + "\n").encode("utf-8"))

def wait_ok(ser, timeout=3.0):
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line == "ok":
            return True
        if line:
            print(f"  unexpected: {line}")
    print("  WARNING: no ok received")
    return False

def read_sensors(ser, timeout=5.0):
    """Send read command, wait for data,... line back."""
    send(ser, "read")
    t0 = time.perf_counter()
    while time.perf_counter() - t0 < timeout:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line.startswith("data,"):
            parts = line.split(",")
            if len(parts) == 7:
                try:
                    I0 = float(parts[1])
                    I1 = float(parts[2])
                    I2 = float(parts[3])
                    Bx = float(parts[4])
                    By = float(parts[5])
                    Bz = float(parts[6])
                    return I0, I1, I2, Bx, By, Bz
                except ValueError:
                    pass
    print("  WARNING: sensor read timed out")
    return None

# ================= LINEAR FIT =================
def fit_gain(currents, fields):
    """
    Fit B = G * I + offset using least squares.
    Returns slope G and offset for each field component.
    """
    currents = np.array(currents)
    fields   = np.array(fields)
    results  = {}
    for i, name in enumerate(["Bx", "By", "Bz"]):
        if len(currents) < 2:
            results[name] = {"slope": 0.0, "offset": 0.0}
            continue
        coeffs = np.polyfit(currents, fields[:, i], 1)
        results[name] = {"slope": coeffs[0], "offset": coeffs[1]}
    return results

# ================= MAIN =================
def main():
    port = SERIAL_PORT or find_port()
    if port is None:
        print("No Arduino found")
        return

    print(f"Connecting on {port}...")
    ser = serial.Serial(port, BAUD_RATE, timeout=2.0)
    
    # Wait for READY - modified to catch and print Arduino boot errors
    print("Waiting for READY...")
    t0 = time.perf_counter()
    ready = False
    while time.perf_counter() - t0 < 5.0:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if line == "READY":
            print("Arduino ready.")
            ready = True
            break
        elif line:
            print(f"  Arduino: {line}")
            
    if not ready:
        print("Failed to get READY signal. Halting.")
        ser.close()
        return

    # Read ambient field with everything zeroed
    print("\nReading ambient field (all coils off)...")
    send(ser, "zero")
    wait_ok(ser)
    time.sleep(ZERO_SETTLE / 1000.0)
    ambient = read_sensors(ser)
    if ambient is None:
        print("Failed to read ambient — check magnetometer wiring")
        ser.close()
        return

    I0_amb, I1_amb, I2_amb, Bx_amb, By_amb, Bz_amb = ambient 
    print(f"  Ambient: Bx={Bx_amb:.4f}  By={By_amb:.4f}  Bz={Bz_amb:.4f} uT")

    # Storage for results
    all_fits   = {}
    G_matrix   = np.zeros((3, N_AXES))

    for axis in range(N_AXES):
        print(f"\n{'='*50}")
        print(f"Sweeping axis {axis} ({AXIS_NAMES[axis]} coil)")
        print(f"{'='*50}")

        currents = []
        readings = []

        for pwm in PWM_STEPS:
            # Set PWM on this axis, zero all others
            send(ser, f"pwm,{axis},{pwm}")
            wait_ok(ser)
            time.sleep(SETTLE_MS / 1000.0)

            result = read_sensors(ser)
            if result is None:
                continue

            I0, I1, I2, Bx, By, Bz = result

            currents_list = [I0, I1, I2]
            I_active = currents_list[axis]

            # Subtract ambient field
            Bx_corr = Bx - Bx_amb
            By_corr = By - By_amb
            Bz_corr = Bz - Bz_amb

            currents.append(I_active)
            readings.append([Bx_corr, By_corr, Bz_corr])

            print(f"  pwm={pwm:4d}  I={I_active:7.4f}A  "
                  f"dBx={Bx_corr:8.4f}  dBy={By_corr:8.4f}  dBz={Bz_corr:8.4f} uT")

        # Zero between axes
        send(ser, "zero")
        wait_ok(ser)
        time.sleep(ZERO_SETTLE / 1000.0)

        # Fit lines
        fit = fit_gain(currents, readings)
        all_fits[axis] = {"fit": fit, "currents": currents, "readings": readings}

        print(f"\n  Fit results for {AXIS_NAMES[axis]} coil:")
        for comp, res in fit.items():
            print(f"    d{comp}/dI = {res['slope']:10.4f} uT/A  "
                  f"offset = {res['offset']:8.4f} uT")

        # Store G matrix column
        G_matrix[0, axis] = fit["Bx"]["slope"]
        G_matrix[1, axis] = fit["By"]["slope"]
        G_matrix[2, axis] = fit["Bz"]["slope"]

    # Zero everything at end
    send(ser, "zero")
    wait_ok(ser)

    # ================= RESULTS =================
    print(f"\n{'='*50}")
    print("CALIBRATION COMPLETE")
    print(f"{'='*50}")

    print("\nFull G matrix (uT/A)  [rows=Bx,By,Bz  cols=Ix,Iy,Iz]:")
    comp_names = ["Bx", "By", "Bz"]
    for i, comp in enumerate(comp_names):
        row = "  " + comp + ": "
        for j in range(N_AXES):
            row += f"{G_matrix[i,j]:10.4f}  "
        print(row)

    print("\nDiagonal gains (on-axis, uT/A):")
    for axis in range(N_AXES):
        col       = G_matrix[:, axis]
        max_idx   = np.argmax(np.abs(col))
        max_gain  = col[max_idx]
        print(f"  {AXIS_NAMES[axis]} coil dominant: "
              f"d{comp_names[max_idx]}/dI = {max_gain:.4f} uT/A")

    print("\nCross-axis coupling:")
    for axis in range(N_AXES):
        col      = G_matrix[:, axis]
        max_gain = np.max(np.abs(col))
        for i, comp in enumerate(comp_names):
            pct = abs(col[i]) / max_gain * 100 if max_gain > 0 else 0
            marker = " <-- on-axis" if np.argmax(np.abs(col)) == i else ""
            print(f"  {AXIS_NAMES[axis]} coil → {comp}: "
                  f"{col[i]:8.4f} uT/A  ({pct:.1f}%){marker}")

    print("\nPaste into leo_sim.py:")
    print(f"# 3-axis (X, Y, Z)")
    print(f"# G columns = [X_coil, Y_coil, Z_coil], rows = [Bx, By, Bz]")
    gx = G_matrix[:, 0]
    gy = G_matrix[:, 1]
    gz = G_matrix[:, 2]
    
    print(f"G_DIAG_T_PER_A = np.array([{gx[0]*1e-6:.6f}, "
          f"{gy[1]*1e-6:.6f}, {gz[2]*1e-6:.6f}])  # Bx/Ix, By/Iy, Bz/Iz in T/A")

    ser.close()
    print("\nDone.")

if __name__ == "__main__":
    main()