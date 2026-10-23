#!/usr/bin/env python3
"""Headless GPS capture using GNURadio + osmosdr (LimeSDR or HackRF).

Usage:
  python gps_recv.py [--device DEVICE] [--antenna ANT] [--seconds N] [--output FILE]

Examples:
  python gps_recv.py --device "driver=lime,soapy=0" --antenna LNAW
  python gps_recv.py --device "" --antenna ""   # HackRF (default osmosdr)
"""
import argparse
import signal
import sys
import time

from gnuradio import gr, blocks
import osmosdr


class GPSReceiver(gr.top_block):
    def __init__(self, device_args, antenna, output_file, samp_rate=4.096e6):
        gr.top_block.__init__(self, "GPS Receiver")

        self.source = osmosdr.source(args=device_args)
        self.source.set_sample_rate(samp_rate)
        self.source.set_center_freq(1575.42e6, 0)
        self.source.set_freq_corr(0, 0)

        if "lime" in device_args.lower():
            self.source.set_gain(30, 0)       # LNA
            self.source.set_if_gain(12, 0)    # TIA
            self.source.set_bb_gain(19, 0)    # PGA
            self.source.set_dc_offset_mode(2, 0)  # auto
            self.source.set_bandwidth(5e6, 0)
        else:
            self.source.set_gain(25, 0)       # RF
            self.source.set_if_gain(40, 0)
            self.source.set_bb_gain(40, 0)
            self.source.set_dc_offset_mode(0, 0)

        if antenna:
            self.source.set_antenna(antenna, 0)

        self.source.set_bandwidth(0, 0)

        self.sink = blocks.file_sink(gr.sizeof_gr_complex, output_file, False)
        self.sink.set_unbuffered(False)

        self.connect((self.source, 0), (self.sink, 0))


def main():
    parser = argparse.ArgumentParser(description="GPS IQ capture")
    parser.add_argument("--device", default="driver=lime,soapy=0",
                        help="osmosdr device string")
    parser.add_argument("--antenna", default="LNAW",
                        help="Antenna port (LNAW for LimeSDR, empty for HackRF)")
    parser.add_argument("--seconds", type=int, default=0,
                        help="Capture duration (0 = run until Ctrl+C)")
    parser.add_argument("--output", default="gps_captured.iq",
                        help="Output file path")
    args = parser.parse_args()

    tb = GPSReceiver(args.device, args.antenna, args.output)

    def stop_handler(sig, frame):
        print("\nStopping capture...")
        tb.stop()
        tb.wait()
        sys.exit(0)

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    print(f"Starting GPS capture: device={args.device} antenna={args.antenna}")
    print(f"Output: {args.output}")
    tb.start()

    if args.seconds > 0:
        print(f"Capturing for {args.seconds} seconds...")
        time.sleep(args.seconds)
        tb.stop()
        tb.wait()
        print("Done.")
    else:
        print("Capturing until Ctrl+C...")
        tb.wait()


if __name__ == "__main__":
    main()
