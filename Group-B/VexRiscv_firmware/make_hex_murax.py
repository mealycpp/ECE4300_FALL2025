#!/usr/bin/env python3
import os

BIN_IN  = os.path.join("build", "prog.bin")
HEX_OUT = "prog.hex"

BASE_ADDR = 0x80000000  # Murax IMEM base

def main():
    if not os.path.exists(BIN_IN):
        raise SystemExit(f"Input binary not found: {BIN_IN}")

    with open(BIN_IN, "rb") as f:
        data = f.read()

    with open(HEX_OUT, "w") as o:
        # One @-line at the start, like muraxDemo.hex
        o.write("@%08X\n" % BASE_ADDR)

        # 16 bytes per line, space-separated hex, like the demo
        for i in range(0, len(data), 16):
            chunk = data[i:i+16]
            o.write(" ".join(f"{b:02X}" for b in chunk) + "\n")

    print(f"Wrote {HEX_OUT} ({len(data)} bytes from {BIN_IN})")

if __name__ == "__main__":
    main()
