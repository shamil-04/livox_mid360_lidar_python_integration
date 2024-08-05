# lidar/utils.py
import sys

def error(msg):
    print(f"Error: {msg}", file=sys.stderr)
    sys.exit(1)

def print_buffer(buf):
    length = len(buf)
    for i in range(length):
        if i % 16 == 0:
            print()
        if i % 4 == 0:
            print(" ", end="")
        if buf[i] > 9:
            print(f"{buf[i]:02x}", end="")
        elif buf[i]:
            print(f"0{buf[i]:x}", end="")
        else:
            print("00", end="")
    print()
    return length
