"""
XM10/H10 USB-CDC multi-channel live monitor.

Purpose
-------
Reads PhAI V2.2 COBS-framed packets from the board and supports a variable
number of float channels, including EMG + encoder + IMU streams.

Recommended 12-float payload order for H10 experiments
-------------------------------------------------------
0  EMG_PF3
1  EMG_PF4
2  EMG_PF5
3  EMG_PF6
4  HIP_MOTOR_ANGLE_R
5  HIP_MOTOR_VEL_R
6  IMU_ACC_X
7  IMU_ACC_Y
8  IMU_ACC_Z
9  IMU_GYRO_X
10 IMU_GYRO_Y
11 IMU_GYRO_Z

The parser itself is not limited to 12 floats. It decodes every float contained
in the selected module packet.

Examples
--------
python cdc_h10_live_monitor.py --port COM6
python cdc_h10_live_monitor.py --cli --port COM6
python cdc_h10_live_monitor.py --list-ports

Custom labels:
python cdc_h10_live_monitor.py --port COM6 --labels RF,BF,MG,TA,HipAngle,HipVel,Ax,Ay,Az,Gx,Gy,Gz

Dependencies
------------
python -m pip install pyserial pyqt5 pyqtgraph
"""

import argparse
from collections import deque
import csv
from datetime import datetime
import os
import struct
import sys
import time

import serial
import serial.tools.list_ports


PHAI_SOF = 0xAA
PHAI_HEADER_SIZE = 6
PHAI_CRC_SIZE = 2
DEFAULT_BAUD = 921600
DEFAULT_MODULE_ID = 0xF0

DEFAULT_6_LABELS = [
    "Raw_RH",
    "Raw_LH",
    "Env_RH",
    "Env_LH",
    "Tau_RH",
    "Tau_LH",
]

DEFAULT_12_LABELS = [
    "EMG_PF3",
    "EMG_PF4",
    "EMG_PF5",
    "EMG_PF6",
    "HIP_MOTOR_ANGLE_R",
    "HIP_MOTOR_VEL_R",
    "IMU_ACC_X",
    "IMU_ACC_Y",
    "IMU_ACC_Z",
    "IMU_GYRO_X",
    "IMU_GYRO_Y",
    "IMU_GYRO_Z",
]

DEFAULT_14_LABELS = [
    "EMG_PF3",
    "EMG_PF4",
    "EMG_PF5",
    "EMG_PF6",
    "HIP_ANGLE_R",
    "HIP_ANGLE_L",
    "HIP_VEL_R",
    "HIP_VEL_L",
    "IMU_ACC_X",
    "IMU_ACC_Y",
    "IMU_ACC_Z",
    "IMU_GYRO_X",
    "IMU_GYRO_Y",
    "IMU_GYRO_Z",
]


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT, poly 0x1021, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_decode(encoded: bytes) -> bytes:
    """Decode one COBS frame without the trailing 0x00 delimiter."""
    out = bytearray()
    idx = 0

    while idx < len(encoded):
        code = encoded[idx]
        idx += 1

        if code == 0:
            raise ValueError("zero byte inside COBS frame")

        end = idx + code - 1
        if end > len(encoded):
            raise ValueError("truncated COBS frame")

        out.extend(encoded[idx:end])
        idx = end

        if code < 0xFF and idx < len(encoded):
            out.append(0)

    return bytes(out)


def parse_phai_frame(frame: bytes):
    """
    Return:
        seq_id, module_id, status, tuple(float, ...)
    """
    decoded = cobs_decode(frame)

    min_size = PHAI_HEADER_SIZE + PHAI_CRC_SIZE
    if len(decoded) < min_size:
        return None

    if decoded[0] != PHAI_SOF:
        return None

    len_units = decoded[1]
    total_size = PHAI_HEADER_SIZE + len_units * 4 + PHAI_CRC_SIZE

    if len_units == 0 or len(decoded) < total_size:
        return None

    payload_end = PHAI_HEADER_SIZE + len_units * 4

    crc_rx = decoded[payload_end] | (decoded[payload_end + 1] << 8)
    crc_calc = crc16_ccitt(decoded[:payload_end])

    if crc_rx != crc_calc:
        return None

    seq_id = decoded[2] | (decoded[3] << 8)
    module_id = decoded[4]
    status = decoded[5]

    payload = decoded[PHAI_HEADER_SIZE:payload_end]

    # IMPORTANT:
    # This automatically unpacks ALL float channels in the packet.
    values = struct.unpack(f"<{len_units}f", payload)

    return seq_id, module_id, status, values


def list_ports() -> list[str]:
    return [p.device for p in serial.tools.list_ports.comports()]


def guess_port() -> str | None:
    ports = list_ports()

    if len(ports) == 1:
        return ports[0]

    preferred = [
        p
        for p in ports
        if "usbmodem" in p.lower()
        or "usbserial" in p.lower()
        or p.upper().startswith("COM")
    ]

    if len(preferred) == 1:
        return preferred[0]

    return None


def format_module(module_id: int) -> str:
    return f"0x{module_id:02X}"


def choose_labels(channel_count: int, user_labels: list[str] | None) -> list[str]:
    if user_labels:
        if len(user_labels) != channel_count:
            print(
                f"[WARN] packet has {channel_count} floats but "
                f"--labels has {len(user_labels)} names."
            )

            # Keep user names and fill/truncate automatically.
            labels = list(user_labels[:channel_count])
            while len(labels) < channel_count:
                labels.append(f"CH{len(labels)}")
            return labels

        return list(user_labels)

    if channel_count == 6:
        return list(DEFAULT_6_LABELS)

    if channel_count == 12:
        return list(DEFAULT_12_LABELS)

    if channel_count == 14:
        return list(DEFAULT_14_LABELS)

    return [f"CH{i}" for i in range(channel_count)]


def values_to_dict(labels, values):
    """
    Convenient access point for future Minecraft / EMG control code.

    Example:
        data = values_to_dict(labels, values)
        hip_angle = data.get("HIP_MOTOR_ANGLE_R")
        ax = data.get("IMU_ACC_X")
    """
    return dict(zip(labels, values))


def print_live(labels, values, seq_id, packet_count, rate_hz, module_id):
    pairs = "  ".join(
        f"{label}:{value:8.4f}"
        for label, value in zip(labels, values)
    )

    line = (
        f"\rmodule:{format_module(module_id)}  "
        f"seq:{seq_id:5d}  "
        f"pkts:{packet_count:7d}  "
        f"rate:{rate_hz:6.1f} Hz  "
        f"{pairs}"
    )

    print(line, end="", flush=True)


def run_cli(
    port: str,
    baud: int,
    module_id: int,
    user_labels: list[str] | None,
    print_hz: float,
):
    print(f"Opening {port} @ {baud}")
    print(f"Listening for module {format_module(module_id)}")
    print("Channel count will be detected automatically.")
    print("Press Ctrl+C to stop.\n")

    ser = serial.Serial(port, baud, timeout=0.02)
    ser.dtr = True
    ser.rts = True
    ser.reset_input_buffer()
    ser.write(b"AGRB MON START\r\n")

    wire_buf = bytearray()

    packet_count = 0
    matched_count = 0
    error_count = 0

    labels = None
    channel_count = None

    t0 = time.perf_counter()
    last_print = 0.0

    try:
        while True:
            chunk = ser.read(4096)

            if not chunk:
                continue

            wire_buf.extend(chunk)

            while True:
                delim = wire_buf.find(b"\x00")

                if delim < 0:
                    break

                raw_frame = bytes(wire_buf[:delim])
                del wire_buf[: delim + 1]

                if not raw_frame:
                    continue

                try:
                    packet = parse_phai_frame(raw_frame)
                except ValueError:
                    packet = None

                if packet is None:
                    error_count += 1
                    continue

                seq_id, mid, _status, values = packet
                packet_count += 1

                if mid != module_id:
                    continue

                matched_count += 1

                if channel_count != len(values):
                    channel_count = len(values)
                    labels = choose_labels(channel_count, user_labels)

                    print(
                        f"\nDetected {channel_count} float channels: "
                        + ", ".join(labels)
                    )

                now = time.perf_counter()

                if now - last_print >= 1.0 / print_hz:
                    elapsed = max(now - t0, 1e-6)
                    rate_hz = matched_count / elapsed

                    print_live(
                        labels,
                        values,
                        seq_id,
                        matched_count,
                        rate_hz,
                        mid,
                    )

                    last_print = now

    except KeyboardInterrupt:
        print("\n\nStopped.")
        print(f"Decoded packets: {packet_count}")
        print(f"Matched packets: {matched_count}")
        print(f"Decode errors:   {error_count}")

    finally:
        ser.close()


def run_gui(
    port: str,
    baud: int,
    module_id: int,
    user_labels: list[str] | None,
    window_sec: float,
    plot_hz: float,
):
    try:
        from PyQt5 import QtCore, QtWidgets
        import pyqtgraph as pg

    except ImportError as exc:
        missing = exc.name or "PyQt5/pyqtgraph"

        print(
            f"Missing GUI dependency: {missing}",
            file=sys.stderr,
        )
        print(
            "Install with: python -m pip install pyserial pyqt5 pyqtgraph",
            file=sys.stderr,
        )
        sys.exit(2)

    class SerialWorker(QtCore.QThread):
        # t, seq_id, values, rate_hz, error_count
        sample = QtCore.pyqtSignal(float, int, object, float, int)
        status = QtCore.pyqtSignal(str)

        def __init__(self):
            super().__init__()
            self._running = True

        def stop(self):
            self._running = False

        def run(self):
            try:
                ser = serial.Serial(
                    port,
                    baud,
                    timeout=0.02,
                )

                ser.dtr = True
                ser.rts = True

                ser.reset_input_buffer()
                ser.write(b"AGRB MON START\r\n")

            except Exception as exc:
                self.status.emit(
                    f"open failed: {exc}"
                )
                return

            self.status.emit(
                f"connected: {port} @ {baud}"
            )

            wire_buf = bytearray()

            rx_bytes = 0
            decoded_count = 0
            matched_count = 0
            error_count = 0

            t0 = time.perf_counter()

            last_status = 0.0
            last_keepalive = 0.0

            try:
                while self._running:
                    try:
                        chunk = ser.read(4096)

                    except serial.SerialException as exc:
                        self.status.emit(
                            f"serial error: {exc}"
                        )
                        break

                    now_abs = time.perf_counter()

                    if not chunk:
                        if now_abs - last_keepalive >= 1.0:
                            try:
                                ser.write(
                                    b"AGRB MON START\r\n"
                                )

                            except serial.SerialException as exc:
                                self.status.emit(
                                    f"serial error: {exc}"
                                )
                                break

                            self.status.emit(
                                f"waiting for module "
                                f"{format_module(module_id)} | "
                                f"decoded:{decoded_count} "
                                f"matched:{matched_count} "
                                f"errors:{error_count}"
                            )

                            last_keepalive = now_abs

                        continue

                    rx_bytes += len(chunk)
                    wire_buf.extend(chunk)

                    if now_abs - last_status >= 0.5:
                        self.status.emit(
                            f"connected: {port} @ {baud} | "
                            f"rx:{rx_bytes}B "
                            f"decoded:{decoded_count} "
                            f"matched:{matched_count} "
                            f"errors:{error_count}"
                        )

                        last_status = now_abs

                    while self._running:
                        delim = wire_buf.find(b"\x00")

                        if delim < 0:
                            break

                        raw_frame = bytes(wire_buf[:delim])
                        del wire_buf[: delim + 1]

                        if not raw_frame:
                            continue

                        try:
                            packet = parse_phai_frame(
                                raw_frame
                            )

                        except ValueError:
                            packet = None

                        if packet is None:
                            error_count += 1
                            continue

                        seq_id, mid, _status, values = packet
                        decoded_count += 1

                        if mid != module_id:
                            continue

                        matched_count += 1

                        now = now_abs - t0
                        rate_hz = (
                            matched_count
                            / max(now, 1e-6)
                        )

                        self.sample.emit(
                            now,
                            seq_id,
                            tuple(values),
                            rate_hz,
                            error_count,
                        )

            finally:
                ser.close()
                self.status.emit("disconnected")

    class MainWindow(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()

            self.setWindowTitle(
                f"XM10/H10 Multi-channel Monitor - "
                f"module {format_module(module_id)}"
            )

            self.worker = SerialWorker()

            self.labels = None
            self.channel_count = 0

            self.times = deque()
            self.values = []
            self.curves = []

            self.last_plot_update = 0.0

            self.record_file = None
            self.record_writer = None
            self.record_path = None
            self.record_count = 0

            self.latest_values = {}
            self.last_seq_id = None

            central = QtWidgets.QWidget()
            layout = QtWidgets.QVBoxLayout(central)

            layout.setContentsMargins(
                10, 10, 10, 10
            )
            layout.setSpacing(8)

            top_bar = QtWidgets.QHBoxLayout()

            self.status_label = QtWidgets.QLabel(
                "opening serial port..."
            )
            self.status_label.setMinimumHeight(24)

            top_bar.addWidget(
                self.status_label,
                1,
            )

            self.record_button = QtWidgets.QPushButton(
                "Record"
            )
            self.record_button.setCheckable(True)
            self.record_button.clicked.connect(
                self.on_record_clicked
            )

            top_bar.addWidget(
                self.record_button
            )

            layout.addLayout(top_bar)

            self.plot = pg.PlotWidget()
            self.plot.setBackground("w")
            self.plot.showGrid(
                x=True,
                y=True,
                alpha=0.25,
            )
            self.plot.setLabel(
                "bottom",
                "time",
                units="s",
            )
            self.plot.setLabel(
                "left",
                "value",
            )
            self.plot.addLegend(
                offset=(10, 10)
            )
            self.plot.setXRange(
                0,
                window_sec,
            )

            layout.addWidget(
                self.plot,
                1,
            )

            self.value_label = QtWidgets.QLabel(
                "waiting for first packet..."
            )
            self.value_label.setWordWrap(True)
            self.value_label.setMinimumHeight(42)

            layout.addWidget(
                self.value_label
            )

            self.record_label = QtWidgets.QLabel(
                "recording: off"
            )
            self.record_label.setMinimumHeight(22)

            layout.addWidget(
                self.record_label
            )

            self.setCentralWidget(central)
            self.resize(1250, 720)

            self.worker.sample.connect(
                self.on_sample
            )
            self.worker.status.connect(
                self.status_label.setText
            )
            self.worker.start()

        def rebuild_channels(
            self,
            channel_count: int,
        ):
            self.channel_count = channel_count

            self.labels = choose_labels(
                channel_count,
                user_labels,
            )

            self.times.clear()

            self.values = [
                deque()
                for _ in range(channel_count)
            ]

            self.plot.clear()
            self.plot.addLegend(
                offset=(10, 10)
            )

            self.curves = []

            for idx, label in enumerate(self.labels):
                pen = pg.mkPen(
                    pg.intColor(
                        idx,
                        hues=max(
                            channel_count,
                            1,
                        ),
                    ),
                    width=2,
                )

                curve = self.plot.plot(
                    [],
                    [],
                    pen=pen,
                    name=label,
                )

                self.curves.append(curve)

            self.status_label.setText(
                f"Detected {channel_count} channels: "
                + ", ".join(self.labels)
            )

        def on_sample(
            self,
            t,
            seq_id,
            values,
            rate_hz,
            error_count,
        ):
            if (
                self.channel_count
                != len(values)
            ):
                self.rebuild_channels(
                    len(values)
                )

            # ---------------------------------------------
            # IMPORTANT FOR FUTURE EMG / MINECRAFT CONTROL
            # ---------------------------------------------
            self.latest_values = values_to_dict(
                self.labels,
                values,
            )
            self.last_seq_id = seq_id

            # Examples when using the recommended 12-channel layout:
            #
            # emg_pf3 = self.latest_values.get("EMG_PF3")
            # hip_angle = self.latest_values.get("HIP_MOTOR_ANGLE_R")
            # hip_velocity = self.latest_values.get("HIP_MOTOR_VEL_R")
            # ax = self.latest_values.get("IMU_ACC_X")
            # gx = self.latest_values.get("IMU_GYRO_X")
            #
            # This is the point where a future EMG classifier or
            # Minecraft controller can consume the live sensor values.

            self.write_record_row(
                t,
                seq_id,
                values,
            )

            now_abs = time.perf_counter()

            if (
                now_abs
                - self.last_plot_update
                < 1.0 / plot_hz
            ):
                return

            self.last_plot_update = now_abs

            self.times.append(t)

            for idx, value in enumerate(values):
                self.values[idx].append(
                    value
                )

            cutoff = t - window_sec

            while (
                self.times
                and self.times[0] < cutoff
            ):
                self.times.popleft()

                for series in self.values:
                    series.popleft()

            xs = list(self.times)

            for curve, series in zip(
                self.curves,
                self.values,
            ):
                curve.setData(
                    xs,
                    list(series),
                )

            if t > window_sec:
                self.plot.setXRange(
                    t - window_sec,
                    t,
                    padding=0,
                )
            else:
                self.plot.setXRange(
                    0,
                    window_sec,
                    padding=0,
                )

            pairs = "    ".join(
                f"{label}: {value:.4f}"
                for label, value in zip(
                    self.labels,
                    values,
                )
            )

            self.value_label.setText(
                f"seq:{seq_id}    "
                f"rate:{rate_hz:.1f} Hz    "
                f"errors:{error_count}    "
                f"{pairs}"
            )

        def on_record_clicked(
            self,
            checked,
        ):
            if checked:
                if not self.start_recording():
                    self.record_button.setChecked(
                        False
                    )
            else:
                self.stop_recording()

        def start_recording(self):
            if not self.labels:
                QtWidgets.QMessageBox.warning(
                    self,
                    "Recording",
                    "No sensor packet received yet.",
                )
                return False

            default_dir = os.path.join(
                os.path.dirname(__file__),
                "data",
            )

            os.makedirs(
                default_dir,
                exist_ok=True,
            )

            default_name = (
                "h10_stream_"
                + datetime.now().strftime(
                    "%Y%m%d_%H%M%S"
                )
                + ".csv"
            )

            default_path = os.path.join(
                default_dir,
                default_name,
            )

            path, _ = (
                QtWidgets.QFileDialog
                .getSaveFileName(
                    self,
                    "Save CSV Recording",
                    default_path,
                    "CSV Files (*.csv);;All Files (*)",
                )
            )

            if not path:
                return False

            try:
                self.record_file = open(
                    path,
                    "w",
                    newline="",
                    encoding="utf-8",
                )

            except OSError as exc:
                QtWidgets.QMessageBox.critical(
                    self,
                    "Recording Error",
                    str(exc),
                )
                return False

            self.record_writer = csv.writer(
                self.record_file
            )

            self.record_writer.writerow(
                [
                    "time_s",
                    "seq_id",
                    *self.labels,
                ]
            )

            self.record_path = path
            self.record_count = 0

            self.record_button.setText(
                "Stop"
            )

            self.record_label.setText(
                f"recording: 0 rows -> {path}"
            )

            return True

        def stop_recording(self):
            if self.record_file:
                self.record_file.flush()
                self.record_file.close()

            saved_path = self.record_path
            saved_count = self.record_count

            self.record_file = None
            self.record_writer = None
            self.record_path = None

            self.record_button.setText(
                "Record"
            )

            if saved_path:
                self.record_label.setText(
                    f"saved: {saved_count} rows "
                    f"-> {saved_path}"
                )
            else:
                self.record_label.setText(
                    "recording: off"
                )

        def write_record_row(
            self,
            t,
            seq_id,
            values,
        ):
            if not self.record_writer:
                return

            self.record_writer.writerow(
                [
                    f"{t:.6f}",
                    seq_id,
                    *(
                        f"{v:.8f}"
                        for v in values
                    ),
                ]
            )

            self.record_count += 1

            if (
                self.record_count
                % 200
                == 0
            ):
                self.record_file.flush()

                self.record_label.setText(
                    f"recording: "
                    f"{self.record_count} rows "
                    f"-> {self.record_path}"
                )

        def closeEvent(
            self,
            event,
        ):
            if self.record_file:
                self.stop_recording()

            self.worker.stop()
            self.worker.wait(1000)

            event.accept()

    app = QtWidgets.QApplication(
        sys.argv
    )

    pg.setConfigOptions(
        antialias=True
    )

    window = MainWindow()
    window.show()

    sys.exit(
        app.exec_()
    )


def parse_module_id(text: str) -> int:
    return int(text, 0)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "XM10/H10 USB-CDC multi-channel live monitor"
        )
    )

    parser.add_argument(
        "--port",
        help="Serial port, e.g. COM6",
    )

    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
    )

    parser.add_argument(
        "--module",
        type=parse_module_id,
        default=DEFAULT_MODULE_ID,
    )

    parser.add_argument(
        "--labels",
        default=None,
        help=(
            "Optional comma-separated channel names. "
            "Example: RF,BF,MG,TA,HipAngle,HipVel,Ax,Ay,Az,Gx,Gy,Gz"
        ),
    )

    parser.add_argument(
        "--cli",
        action="store_true",
        help="Terminal output instead of GUI",
    )

    parser.add_argument(
        "--print-hz",
        type=float,
        default=20.0,
    )

    parser.add_argument(
        "--plot-hz",
        type=float,
        default=60.0,
    )

    parser.add_argument(
        "--window",
        type=float,
        default=10.0,
        help="GUI time window in seconds",
    )

    parser.add_argument(
        "--list-ports",
        action="store_true",
    )

    args = parser.parse_args()

    if args.list_ports:
        for port in list_ports():
            print(port)
        return

    user_labels = None

    if args.labels:
        user_labels = [
            item.strip()
            for item in args.labels.split(",")
            if item.strip()
        ]

    port = (
        args.port
        or guess_port()
    )

    if not port:
        print(
            "Could not auto-detect a serial port. "
            "Available ports:",
            file=sys.stderr,
        )

        for item in list_ports():
            print(
                f"  {item}",
                file=sys.stderr,
            )

        print(
            "Run again with --port <PORT>.",
            file=sys.stderr,
        )

        sys.exit(2)

    if args.cli:
        run_cli(
            port,
            args.baud,
            args.module,
            user_labels,
            args.print_hz,
        )
    else:
        run_gui(
            port,
            args.baud,
            args.module,
            user_labels,
            args.window,
            args.plot_hz,
        )


if __name__ == "__main__":
    main()
