"""Optional offline Ouster adapter; run with a Python containing ouster-sdk 1.0.1.

It has no ROS or YOLO dependency. Live use gets PointCloud2 from the sensor driver.
Raw packets in the supplied recordings have a device uptime clock, not Unix time.
For those packets the host receive midpoint is an explicit approximation, never
silently presented as a hardware-synchronized acquisition timestamp.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from ouster.sdk import core


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag')
    parser.add_argument('output')
    parser.add_argument('--hz', type=float, default=10.0)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    records, incomplete, last = [], 0, None
    with open(args.bag, 'rb') as stream:
        reader = make_reader(stream, decoder_factories=[DecoderFactory()])
        _, _, _, metadata = next(reader.iter_decoded_messages(topics=['/ouster/metadata']))
        info = core.SensorInfo(metadata.data)
        lut, pf = core.XYZLut(info), core.PacketFormat(info)
        batch, scan = core.FrameBatcher(info), core.LidarFrame(info)
        for _, _, msg, decoded in reader.iter_decoded_messages(topics=['/ouster/lidar_packets']):
            packet = core.LidarPacket(pf)
            packet.buf[:] = np.frombuffer(decoded.buf, dtype=np.uint8)
            packet.host_timestamp = msg.log_time
            if not batch.batch(packet, scan):
                continue
            coverage = float(np.count_nonzero(scan.status & 1) / scan.w)
            if coverage >= .98:
                host = np.sort(np.asarray(scan.packet_timestamp)[scan.packet_timestamp > 0])
                stamp = int(host[len(host) // 2])
                if last is None or stamp - last >= 1e9 / args.hz * .95:
                    valid = (scan.field(core.ChanField.RANGE) > 0) & ((scan.status & 1)[None, :] > 0)
                    xyz = lut(scan)[valid].astype(np.float32)
                    name = f'{len(records):05d}.npz'
                    np.savez(out / name, xyz=xyz)
                    records.append({'file': name, 'stamp_ns': stamp, 'log_ns': msg.log_time,
                                    'frame': 'os_sensor', 'coverage': coverage})
                    last = stamp
            else:
                incomplete += 1
            scan = core.LidarFrame(info)
    (out / 'scans.json').write_text(json.dumps({'scans': records, 'incomplete': incomplete,
        'time_basis': 'packet_host_receive_midpoint_approximate'}))
    print(f'Ouster: {len(records)} scans, {incomplete} incomplete scans skipped', flush=True)


if __name__ == '__main__':
    main()
