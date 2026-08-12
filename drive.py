#!/usr/bin/env python3
"""
drive.py -- standalone SmartElex 15D WASD motor test (NO ROS 2).

Tap a key -> sends the 6-char packet and PRINTS how long the serial write()
took. That write time IS your latency. (The SmartElex serial-over-USB interface
accepts one command every ~1.8-2 s -- if you see ~2000 ms here, the delay is the
driver hardware, not ROS/Foxglove/Python.)

Run on the Pi host after SSH (you're in the dialout group + pyserial is installed):
    python3 ~/drive.py
(container alternative: docker exec -it rover_ros2 python3 /root/drive.py)

Hardware: SmartElex 15D in SERIAL MODE (DIP switch = 8), USB -> /dev/ttyACM0
@ 115200. USB powers logic only -- motors need external 7-22 V on VIN/GND.

Keys:
    w = forward       s = back
    a = turn left     d = turn right
    x  or  SPACE = STOP
    q  or  Ctrl-C = quit (sends STOP first)

Packet: * [DIR1][SPD1][DIR2][SPD2] #     DIR 1=fwd / 0=rev,  SPD 0..9 (0=0%, 9=100%)
    forward *1S1S#   back *0S0S#
    left    *0S1S#   right *1S0S#     (ch1=left, ch2=right)
    stop    *1010#

Notes:
  * Last command PERSISTS (no deadman) -- press x / SPACE to stop.
  * If 'w' drives backward, motor polarity is flipped (swap a channel's wires,
    or we invert it in software later). If a/d turn the wrong way, swap them.
  * Make sure the ROS motor node is NOT running (it would contend for the port):
        docker exec rover_ros2 pkill -9 -f l298n_node.py
"""
import sys
import time
import termios
import tty

try:
    import serial
except ImportError:
    sys.exit("pyserial not installed. In the container: apt install -y python3-serial")

PORT = "/dev/ttyACM0"
BAUD = 115200
SPEED = 5          # 0..9 duty level (5 ~= 50%). Lower = slower / safer.

# Build packets (S = SPEED). ch1=left, ch2=right.
FWD   = f"*1{SPEED}1{SPEED}#"
BACK  = f"*0{SPEED}0{SPEED}#"
LEFT  = f"*0{SPEED}1{SPEED}#"      # left rev,  right fwd  -> turn left
RIGHT = f"*1{SPEED}0{SPEED}#"      # left fwd,  right rev  -> turn right
STOP  = "*1010#"

KEYS = {'w': FWD, 's': BACK, 'a': LEFT, 'd': RIGHT, 'x': STOP, ' ': STOP}


def getch():
    """Read one keypress without waiting for Enter."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def send(ser, packet, label):
    t0 = time.monotonic()
    ser.write(packet.encode('ascii'))
    ser.flush()
    dt = (time.monotonic() - t0) * 1000.0
    print(f"  [{label:>5}] {packet}   write = {dt:7.1f} ms", flush=True)


def main():
    print(f"drive.py -- SmartElex 15D WASD test   ({PORT} @ {BAUD}, SPEED={SPEED})")
    print("keys:  w fwd   s back   a left   d right   x/SPACE stop   q quit")
    print("Lift/block the wheels before pressing w/s/a/d.  Last cmd persists.\n")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=5.0)
    except Exception as e:
        sys.exit(f"could not open {PORT}: {e}\nIs the SmartElex USB plugged in?")
    time.sleep(0.5)
    send(ser, STOP, "STOP")
    print("\nready -- press a key.  (watch the 'write =' ms; that's your latency)\n")
    try:
        while True:
            ch = getch()
            if ch in ('q', '\x03'):
                break
            if ch in KEYS:
                send(ser, KEYS[ch], 'STOP' if ch == ' ' else ch.upper())
    except KeyboardInterrupt:
        pass
    finally:
        send(ser, STOP, "STOP")
        ser.close()
        print("\nstopped, port closed. bye.")


if __name__ == "__main__":
    main()
