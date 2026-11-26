#!/usr/bin/env python3
"""Unified product/debug monitor for the eight-edge optical film.

This program deliberately contains no signal-processing or positioning
algorithm. It validates and decodes firmware frames, converts sensor indices
to display coordinates, and draws firmware-provided values unchanged.

The UI receives only the normal board output after smoothing, baseline
subtraction and gating. It does not process that waveform.
"""
from __future__ import annotations

import os
os.environ.setdefault("PYQTGRAPH_QT_LIB", "PyQt5")

import math
import json
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Dict, Optional

# Keep shared capture/analysis geometry in the project root.  This makes the
# UI self-locating when it is launched from the UI directory or a shortcut.
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

try:
    import numpy as np
    import pyqtgraph as pg
    from PyQt5.QtCore import QDateTime, QRectF, QSettings, Qt, QTimer
    from PyQt5.QtGui import QFont
    from PyQt5.QtWidgets import (
        QApplication, QCheckBox, QComboBox, QGroupBox,
        QDoubleSpinBox, QFileDialog, QHBoxLayout, QHeaderView, QLabel, QMainWindow, QPushButton, QSpinBox,
        QGraphicsRectItem, QMessageBox, QStatusBar, QTableWidget, QTableWidgetItem, QTabWidget,
        QVBoxLayout, QWidget,
    )
except ImportError as exc:
    raise SystemExit(
        "缺少综合UI运行依赖。请安装: numpy pyserial PyQt5 pyqtgraph\n"
        f"原始错误: {exc}"
    ) from exc

from board_layout import EDGE_ORDER, EDGE_SENSORS, SEG_BOARDS

try:
    import serial
    from serial.tools import list_ports
    SERIAL_OK = True
except ImportError:
    serial = None
    list_ports = None
    SERIAL_OK = False

try:
    import pyautogui
    MOUSE_OK = True
except ImportError:
    pyautogui = None
    MOUSE_OK = False

SCREEN_W = 599.0
SCREEN_H = 352.0
OUTER_OFFSET = 25.0
ASSEMBLY_W = SCREEN_W + 2.0 * OUTER_OFFSET
ASSEMBLY_H = SCREEN_H + 2.0 * OUTER_OFFSET
# Coordinate origin is the lower-left corner of the 599 x 352 mm active
# display.  Detector PCB origins are independent: in the extended-groove
# layout all four long-edge PCB assemblies begin about 25 mm to the left of
# x=0 and end about 25 mm to the right of x=599.
BAUD_DEFAULT = "460800"

DIAG_HEADER = b"\xaa\x56"
LEGACY_HEADER = b"\xaa\x55"
FOOTER = b"\xfe\xfd"
DIAG_VERSION = 4
DIAG_TYPE_SNAPSHOT = 1
CMD_VERSION = 2
DIAG_FIXED_HEADER = 34
POS_FRAME_LEN = 14
MAX_DIAG_FRAME = 65535

FLAG_BEFORE = 1 << 0
FLAG_PROCESSED = 1 << 1
FLAG_CENTROIDS = 1 << 2
FLAG_POSITION = 1 << 3
FLAG_POSITION_DEBUG = 1 << 4
FLAG_CANDIDATES = 1 << 5
FLAG_PARAMS = 1 << 6
MAX_CANDIDATES = 4
CALIB_MAX_POINTS = 16
DETECTOR_PITCH_MM = 6.0
PCB_NOMINAL_MM = {50: 300.0, 58: 350.0}
PCB_FIRST_DETECTOR_MM = {50: 3.0, 58: 4.0}

# Standard 7 x 3 subset of the original 9 x 5 grid.  No new coordinates are
# introduced: endpoints and the two-PCB seam neighbourhood are retained.
CALIB_GRID_X = (30.0, 95.0, 225.0, 290.0, 355.0, 485.0, 550.0)
CALIB_GRID_Y = (30.0, 160.0, 290.0)

EDGE_LABELS = {
    "up": "外上 UP", "down": "外下 DOWN",
    "left": "外左 LEFT", "right": "外右 RIGHT",
    "in_up": "内上 IN_UP", "in_down": "内下 IN_DOWN",
    "in_left": "内左 IN_LEFT", "in_right": "内右 IN_RIGHT",
}

EDGE_COLORS = {
    "up": "#29b6f6", "down": "#26c6da", "left": "#ab47bc", "right": "#5c6bc0",
    "in_up": "#ffca28", "in_down": "#ff7043", "in_left": "#ec407a", "in_right": "#9ccc65",
}


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


@dataclass(frozen=True)
class CentroidValue:
    position: float
    peak: float
    valid: bool


@dataclass(frozen=True)
class PositionDebug:
    x_line: float
    y_line: float
    have_x: bool
    have_y: bool
    x_cross_out: float
    y_cross_out: float
    cross_out_valid: bool
    cross_source: int
    cross_lines: int
    cross_conf: float
    cross_diff: float
    lines: tuple[tuple[float, float, float, float], ...] = ()
    line_valid_mask: int = 0


@dataclass(frozen=True)
class CandidateValue:
    position: float
    peak: float
    width: float
    quality: float
    valid: bool


@dataclass(frozen=True)
class SelectionValue:
    selected: tuple[int, ...]
    mode: int
    support_mask: int
    confidence: float
    score: float
    residual: float
    x: float
    y: float


@dataclass(frozen=True)
class RuntimeValues:
    centroid_filter: bool
    position_filter: bool
    centroid_threshold: float
    position_outlier_mm: float
    cross_min_conf: float
    position_jump_mm: float
    edge_min: tuple[float, ...] = ()
    edge_max: tuple[float, ...] = ()
    calibration_samples: tuple[int, ...] = ()
    calibration_state: int = 0
    calibration_valid_mask: int = 0
    calibration_point_count: tuple[int, ...] = ()
    calibration_last_samples: tuple[int, ...] = ()
    calibration_pending: int = 0
    calibration_requested: int = 0
    calibration_rms: tuple[float, ...] = ()
    calibration_raw: tuple[tuple[float, ...], ...] = ()
    calibration_mm: tuple[tuple[float, ...], ...] = ()


@dataclass(frozen=True)
class Snapshot:
    received_at: float
    sequence: int
    tick_ms: int
    flags: int
    before: Dict[str, np.ndarray]
    processed: Dict[str, np.ndarray]
    centroids: Dict[str, CentroidValue]
    mapped_mm: Dict[str, float]
    position: tuple[float, float, bool]
    debug: Optional[PositionDebug]
    candidates: Dict[str, tuple[CandidateValue, ...]]
    selection: Optional[SelectionValue]
    params: Optional[RuntimeValues]
    edge_seen_mask: int
    frame_bytes: int


def split_layer(values: np.ndarray, lengths: list[int]) -> Dict[str, np.ndarray]:
    result: Dict[str, np.ndarray] = {}
    offset = 0
    for edge, count in zip(EDGE_ORDER, lengths):
        result[edge] = values[offset:offset + count].copy()
        offset += count
    return result


def decode_diagnostic(frame: bytes) -> Snapshot:
    if len(frame) < DIAG_FIXED_HEADER + 4 or frame[:2] != DIAG_HEADER or frame[-2:] != FOOTER:
        raise ValueError("diagnostic framing")
    version, kind, payload_len = struct.unpack_from("<BBH", frame, 2)
    if version != DIAG_VERSION or kind != DIAG_TYPE_SNAPSHOT:
        raise ValueError(f"unsupported diagnostic frame v{version}/type{kind}")
    expected = DIAG_FIXED_HEADER + payload_len + 4
    if len(frame) != expected:
        raise ValueError("diagnostic length")
    stored_crc = struct.unpack_from("<H", frame, len(frame) - 4)[0]
    if crc16_ccitt(frame[2:-4]) != stored_crc:
        raise ValueError("diagnostic CRC")

    sequence, tick_ms, flags = struct.unpack_from("<IIH", frame, 6)
    edge_count = frame[16]
    edge_seen_mask = frame[17]
    if edge_count != len(EDGE_ORDER):
        raise ValueError(f"edge count {edge_count}")
    lengths = list(struct.unpack_from("<8H", frame, 18))
    if any(n <= 0 or n > 512 for n in lengths):
        raise ValueError("edge length")

    offset = DIAG_FIXED_HEADER
    total = sum(lengths)
    before: Dict[str, np.ndarray] = {}
    processed: Dict[str, np.ndarray] = {}
    centroids: Dict[str, CentroidValue] = {}
    mapped_mm: Dict[str, float] = {}
    position = (0.0, 0.0, False)
    debug: Optional[PositionDebug] = None
    candidates: Dict[str, tuple[CandidateValue, ...]] = {}
    selection: Optional[SelectionValue] = None
    params: Optional[RuntimeValues] = None

    if flags & FLAG_BEFORE:
        before_values = np.frombuffer(frame, dtype="<u2", count=total, offset=offset)
        before = split_layer(before_values, lengths)
        offset += total * 2
    if flags & FLAG_PROCESSED:
        processed_values = np.frombuffer(frame, dtype="<u2", count=total, offset=offset)
        processed = split_layer(processed_values, lengths)
        offset += total * 2
    if flags & FLAG_CENTROIDS:
        for edge in EDGE_ORDER:
            pos, peak, valid = struct.unpack_from("<ffB", frame, offset)
            centroids[edge] = CentroidValue(pos, peak, bool(valid))
            offset += 9
        mapped_values = struct.unpack_from("<8f", frame, offset)
        offset += 8 * 4
        mapped_mm = {edge: mapped_values[i] for i, edge in enumerate(EDGE_ORDER)}
    if flags & FLAG_POSITION:
        x, y, valid = struct.unpack_from("<ffB", frame, offset)
        position = (x, y, bool(valid))
        offset += 9
    if flags & FLAG_POSITION_DEBUG:
        values = struct.unpack_from("<ffBBffBBBff", frame, offset)
        offset += 29
        line_values = struct.unpack_from("<16f", frame, offset)
        offset += 64
        line_mask = frame[offset]
        offset += 1
        lines = tuple(tuple(line_values[i:i + 4]) for i in range(0, 16, 4))
        debug = PositionDebug(
            values[0], values[1], bool(values[2]), bool(values[3]),
            values[4], values[5], bool(values[6]), values[7], values[8], values[9], values[10],
            lines, line_mask,
        )
    if flags & FLAG_CANDIDATES:
        counts = frame[offset:offset + len(EDGE_ORDER)]
        offset += len(EDGE_ORDER)
        all_candidates = {}
        for edge_index, edge in enumerate(EDGE_ORDER):
            values = []
            for candidate_index in range(MAX_CANDIDATES):
                pos, peak, width, quality, valid = struct.unpack_from("<ffffB", frame, offset)
                offset += 17
                if candidate_index < counts[edge_index]:
                    values.append(CandidateValue(pos, peak, width, quality, bool(valid)))
            all_candidates[edge] = tuple(values)
        candidates = all_candidates
        selected = tuple(frame[offset:offset + len(EDGE_ORDER)])
        offset += len(EDGE_ORDER)
        mode, support_mask, confidence, score, residual, sx, sy = struct.unpack_from("<BBfffff", frame, offset)
        offset += 22
        selection = SelectionValue(selected, mode, support_mask, confidence, score, residual, sx, sy)
    if flags & FLAG_PARAMS:
        cf, pf, calibration_state, calibration_valid_mask, threshold, outlier, confidence, jump = struct.unpack_from("<BBBBffff", frame, offset)
        edge_min = struct.unpack_from("<8f", frame, offset + 20)
        edge_max = struct.unpack_from("<8f", frame, offset + 52)
        samples = tuple(frame[offset + 84:offset + 92])
        point_count = tuple(frame[offset + 92:offset + 100])
        if any(n > CALIB_MAX_POINTS for n in point_count):
            raise ValueError("calibration point count")
        last_samples = tuple(frame[offset + 100:offset + 108])
        pending, requested = struct.unpack_from("<BB", frame, offset + 108)
        calibration_rms = struct.unpack_from("<8f", frame, offset + 112)
        raw_flat = struct.unpack_from(f"<{8 * CALIB_MAX_POINTS}f", frame, offset + 144)
        mm_flat = struct.unpack_from(f"<{8 * CALIB_MAX_POINTS}f", frame, offset + 656)
        raw_nodes = tuple(tuple(raw_flat[e * CALIB_MAX_POINTS:e * CALIB_MAX_POINTS + point_count[e]])
                          for e in range(len(EDGE_ORDER)))
        mm_nodes = tuple(tuple(mm_flat[e * CALIB_MAX_POINTS:e * CALIB_MAX_POINTS + point_count[e]])
                         for e in range(len(EDGE_ORDER)))
        offset += 1168
        params = RuntimeValues(bool(cf), bool(pf), threshold, outlier, confidence, jump,
                               edge_min, edge_max, samples, calibration_state,
                               calibration_valid_mask, point_count, last_samples,
                               pending, requested, calibration_rms, raw_nodes, mm_nodes)
    if offset != len(frame) - 4:
        raise ValueError("diagnostic payload layout")

    return Snapshot(time.time(), sequence, tick_ms, flags, before, processed,
                    centroids, mapped_mm, position, debug, candidates, selection, params,
                    edge_seen_mask, len(frame))


def decode_position(frame: bytes) -> Snapshot:
    if len(frame) != POS_FRAME_LEN or frame[:2] != LEGACY_HEADER or frame[-2:] != FOOTER:
        raise ValueError("position framing")
    checksum = 0
    for value in frame[2:11]:
        checksum ^= value
    if checksum != frame[11]:
        raise ValueError("position checksum")
    x, y, valid = struct.unpack_from("<ffB", frame, 2)
    return Snapshot(time.time(), 0, 0, FLAG_POSITION, {}, {}, {}, {},
                    (x, y, bool(valid)), None, {}, None, None, 0, len(frame))


class SerialReceiver(threading.Thread):
    def __init__(self, port: str, baud: int):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.running = True
        self.lock = threading.Lock()
        self.buffer = bytearray()
        self.latest: Optional[Snapshot] = None
        # Preserve every received product coordinate even when waveform drawing
        # is slower than serial reception. No interpolation or positioning here.
        self.output_queue = deque(maxlen=2048)
        self.last_error = ""
        self.port_opened = False
        self.opened_at = 0.0
        self.last_byte_at = 0.0
        self.last_frame_at = 0.0
        self.frames_total = 0
        self.crc_errors = 0
        self.dropped_sequences = 0
        self._last_sequence: Optional[int] = None
        self.bytes_total = 0
        self.fps = 0.0
        self.byte_rate = 0.0
        self._window_frames = 0
        self._window_bytes = 0
        self._window_time = time.monotonic()

    def run(self) -> None:
        try:
            port = serial.Serial(self.port, self.baud, timeout=0.02,
                                 write_timeout=0.5)
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
            return
        with self.lock:
            self.port_opened = True
            self.opened_at = time.monotonic()
        try:
            while self.running:
                with self.lock:
                    pending = bytes(getattr(self, "_pending_write", b""))
                    if pending:
                        self._pending_write.clear()
                if pending:
                    port.write(pending)
                count = port.in_waiting
                chunk = port.read(count if count > 0 else 1)
                if not chunk:
                    with self.lock:
                        self._update_rates_locked()
                    continue
                with self.lock:
                    self.last_byte_at = time.monotonic()
                    self.buffer.extend(chunk)
                    self.bytes_total += len(chunk)
                    self._window_bytes += len(chunk)
                    self._parse_locked()
                    self._update_rates_locked()
        except Exception as exc:
            with self.lock:
                self.last_error = str(exc)
        finally:
            try:
                port.close()
            finally:
                with self.lock:
                    self.port_opened = False

    def _update_rates_locked(self) -> None:
        now = time.monotonic()
        elapsed = now - self._window_time
        if elapsed >= 1.0:
            self.fps = self._window_frames / elapsed
            self.byte_rate = self._window_bytes / elapsed
            self._window_frames = 0
            self._window_bytes = 0
            self._window_time = now

    def _accept_locked(self, snapshot: Snapshot) -> None:
        if snapshot.flags & (FLAG_BEFORE | FLAG_PROCESSED):
            if self._last_sequence is not None:
                delta = (snapshot.sequence - self._last_sequence) & 0xFFFFFFFF
                if 1 < delta < 0x80000000:
                    self.dropped_sequences += delta - 1
            self._last_sequence = snapshot.sequence
        self.latest = snapshot
        self.output_queue.append((snapshot.received_at, snapshot.sequence,
                                  snapshot.tick_ms, snapshot.position))
        self.last_frame_at = time.monotonic()
        self.frames_total += 1
        self._window_frames += 1

    def _parse_locked(self) -> None:
        while len(self.buffer) >= 2:
            diag_i = self.buffer.find(DIAG_HEADER)
            pos_i = self.buffer.find(LEGACY_HEADER)
            candidates = [i for i in (diag_i, pos_i) if i >= 0]
            if not candidates:
                if len(self.buffer) > 1:
                    del self.buffer[:-1]
                return
            start = min(candidates)
            if start:
                del self.buffer[:start]

            if self.buffer.startswith(DIAG_HEADER):
                if len(self.buffer) < 6:
                    return
                payload_len = struct.unpack_from("<H", self.buffer, 4)[0]
                frame_len = DIAG_FIXED_HEADER + payload_len + 4
                if frame_len < DIAG_FIXED_HEADER + 4 or frame_len > MAX_DIAG_FRAME:
                    del self.buffer[0]
                    self.crc_errors += 1
                    continue
                if len(self.buffer) < frame_len:
                    return
                frame = bytes(self.buffer[:frame_len])
                try:
                    snapshot = decode_diagnostic(frame)
                except ValueError:
                    del self.buffer[0]
                    self.crc_errors += 1
                    continue
                del self.buffer[:frame_len]
                self._accept_locked(snapshot)
                continue

            if len(self.buffer) < POS_FRAME_LEN:
                return
            frame = bytes(self.buffer[:POS_FRAME_LEN])
            try:
                snapshot = decode_position(frame)
            except ValueError:
                del self.buffer[0]
                self.crc_errors += 1
                continue
            del self.buffer[:POS_FRAME_LEN]
            self._accept_locked(snapshot)

    def take_output_points(self):
        with self.lock:
            points = list(self.output_queue)
            self.output_queue.clear()
            return points

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            wait_seconds = (now - self.opened_at
                            if self.port_opened and self.opened_at else 0.0)
            silence_seconds = (now - (self.last_byte_at or self.opened_at)
                               if self.port_opened and self.opened_at else 0.0)
            frame_silence_seconds = (now - (self.last_frame_at or self.opened_at)
                                     if self.port_opened and self.opened_at else 0.0)
            return (self.latest, self.fps, self.byte_rate, self.frames_total,
                    self.crc_errors, self.dropped_sequences,
                    self.bytes_total, self.last_error,
                    self.port_opened, wait_seconds, silence_seconds,
                    frame_silence_seconds)

    def stop(self) -> None:
        self.running = False
        # A new receiver must not race the old thread for the COM handle.
        # Read and write operations above both have finite timeouts.
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout=1.0)

    def send(self, data: bytes) -> None:
        # Kept on the receiver object so reads and writes use one serial owner.
        with self.lock:
            self._pending_write = getattr(self, "_pending_write", bytearray())
            self._pending_write.extend(data)


class WaveformPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        self.show_processed = QCheckBox("显示处理后")
        self.show_processed.setChecked(True)
        self.auto_range = QCheckBox("Y轴自动范围")
        self.auto_range.setChecked(True)
        self.log_scale = QCheckBox("Y轴对数显示")
        self.pause = QCheckBox("暂停画面")
        for widget in (self.show_processed, self.auto_range, self.log_scale, self.pause):
            controls.addWidget(widget)
        controls.addStretch(1)
        controls.addWidget(QLabel(
            "板端结果：18kHz锁相＋频域噪声抑制＋鲁棒单峰滤波；UI只显示、不找峰"))
        layout.addLayout(controls)

        self.canvas = pg.GraphicsLayoutWidget()
        self.canvas.setBackground("#101418")
        layout.addWidget(self.canvas, 1)
        self.plots = {}
        self.processed_curves = {}
        self.centroid_lines = {}
        self.segment_lines = {}
        for index, edge in enumerate(EDGE_ORDER):
            row, col = divmod(index, 2)
            plot = self.canvas.addPlot(row=row, col=col, title=EDGE_LABELS[edge])
            plot.showGrid(x=True, y=True, alpha=0.18)
            plot.setLabel("left", "幅值")
            plot.setLabel("bottom", "传感器索引")
            processed_curve = plot.plot(pen=pg.mkPen("#ffb74d", width=1.8), name="处理后")
            candidate_curve = plot.plot([], [], pen=None, symbol="o", symbolSize=8,
                                        symbolBrush="#ffee58", name="固件候选")
            selected_curve = plot.plot([], [], pen=None, symbol="star", symbolSize=13,
                                       symbolBrush="#ef5350", name="固件选中")
            centroid_line = pg.InfiniteLine(angle=90, pen=pg.mkPen("#ef5350", width=1.5, style=Qt.DashLine))
            centroid_line.hide()
            plot.addItem(centroid_line)
            segment_lines = []
            offset = 0
            for segment in SEG_BOARDS[edge][:-1]:
                offset += segment
                segment_line = pg.InfiniteLine(
                    float(offset),
                    angle=90, pen=pg.mkPen("#546e7a", style=Qt.DotLine))
                plot.addItem(segment_line)
                segment_lines.append((offset, segment_line))
            self.plots[edge] = plot
            self.processed_curves[edge] = processed_curve
            if not hasattr(self, "candidate_curves"):
                self.candidate_curves, self.selected_curves = {}, {}
            self.candidate_curves[edge] = candidate_curve
            self.selected_curves[edge] = selected_curve
            self.centroid_lines[edge] = centroid_line
            self.segment_lines[edge] = segment_lines

    @staticmethod
    def _axis_limits(edge: str, params: Optional[RuntimeValues]) -> tuple[float, float]:
        if params and len(params.edge_min) == len(EDGE_ORDER):
            i = EDGE_ORDER.index(edge)
            return params.edge_min[i], params.edge_max[i]
        fixed = {
            "up": (-OUTER_OFFSET, SCREEN_W + OUTER_OFFSET),
            "down": (-OUTER_OFFSET, SCREEN_W + OUTER_OFFSET),
            "left": (-OUTER_OFFSET, SCREEN_H + OUTER_OFFSET),
            "right": (SCREEN_H + OUTER_OFFSET, -OUTER_OFFSET),
            "in_up": (0.0, SCREEN_W), "in_down": (0.0, SCREEN_W),
            "in_left": (0.0, SCREEN_H), "in_right": (SCREEN_H, 0.0),
        }
        return fixed[edge]

    @staticmethod
    def _default_index_to_axis(edge: str, index: float) -> float:
        """Map detector index using PCB segments, not the mechanical frame span."""
        segments = SEG_BOARDS[edge]
        total_len = sum(PCB_NOMINAL_MM[n] for n in segments)
        prefix_idx = 0.0
        before_len = 0.0
        seg_board = segments[-1]
        for board in segments:
            n = float(board)
            if index < prefix_idx + n:
                seg_board = board
                break
            prefix_idx += n
            before_len += PCB_NOMINAL_MM[board]
        center = SCREEN_W * 0.5 if edge in ("up", "down", "in_up", "in_down") else SCREEN_H * 0.5
        value = (center - total_len * 0.5 + before_len +
                 PCB_FIRST_DETECTOR_MM[seg_board] +
                 (index - prefix_idx) * DETECTOR_PITCH_MM)
        return 2.0 * center - value if edge in ("right", "in_right") else value

    @classmethod
    def _map_axis(cls, edge: str, index: float,
                  params: Optional[RuntimeValues]) -> float:
        if params and len(params.edge_min) == len(EDGE_ORDER):
            i = EDGE_ORDER.index(edge)
            # Firmware applies calibration atomically.  A partial validation
            # mask is diagnostic only; mixing calibrated and default edges
            # would manufacture a false inner/outer centroid displacement.
            if params.calibration_valid_mask == 0xFF:
                raw = params.calibration_raw[i]
                mm = params.calibration_mm[i]
                if len(raw) >= 2 and len(raw) == len(mm):
                    if index <= raw[0]: a, b = 0, 1
                    elif index >= raw[-1]: a, b = len(raw) - 2, len(raw) - 1
                    else:
                        b = next(j for j, v in enumerate(raw) if index <= v)
                        a = b - 1
                    dr = raw[b] - raw[a]
                    if abs(dr) > 1e-6:
                        return mm[a] + (index - raw[a]) * (mm[b] - mm[a]) / dr
            # Firmware diagnostic ranges describe the default map endpoints;
            # use the same piecewise geometry instead of stretching to frame size.
            return cls._default_index_to_axis(edge, index)
        return cls._default_index_to_axis(edge, index)

    @classmethod
    def _index_to_axis(cls, edge: str, index: float, count: int,
                       params: Optional[RuntimeValues]) -> float:
        return cls._map_axis(edge, index, params)

    @classmethod
    def _wave_axis(cls, edge: str, count: int,
                   params: Optional[RuntimeValues]) -> np.ndarray:
        # A waveform is sampled on detector channels, so its only lossless
        # horizontal coordinate is the sensor index.  Physical millimetres are
        # firmware results and are shown in the overview, not reconstructed by
        # UI from obsolete per-edge calibration nodes.
        return np.arange(count, dtype=float)

    def update_snapshot(self, snapshot: Snapshot) -> None:
        if self.pause.isChecked():
            return
        for edge in EDGE_ORDER:
            processed = snapshot.processed.get(edge)
            edge_index = EDGE_ORDER.index(edge)
            seen = bool(snapshot.edge_seen_mask & (1 << edge_index))
            centroid = snapshot.centroids.get(edge)
            centroid_valid = bool(centroid and centroid.valid and
                                  math.isfinite(centroid.position))
            if not seen:
                state, state_color = "通信未收到", "#ef5350"
            elif centroid_valid:
                state, state_color = "通信正常・质心有效", "#81c784"
            else:
                state, state_color = "通信正常・无有效光", "#ffb74d"
            self.plots[edge].setTitle(
                f"{EDGE_LABELS[edge]}　<span style='color:{state_color}'>{state}</span>")
            if processed is not None:
                wave_axis = self._wave_axis(edge, processed.size, snapshot.params)
                self.processed_curves[edge].setData(wave_axis, processed)
                self.processed_curves[edge].setVisible(self.show_processed.isChecked())

                # Infinite centroid/segment lines can dominate pyqtgraph's
                # automatic bounds and collapse the X axis to a single
                # detector position.  The waveform itself is the authoritative
                # display extent, so keep its complete sensor span visible.
                finite_axis = wave_axis[np.isfinite(wave_axis)]
                if finite_axis.size >= 2:
                    x_min = float(np.min(finite_axis))
                    x_max = float(np.max(finite_axis))
                    if x_max > x_min:
                        self.plots[edge].setXRange(x_min, x_max, padding=0.02)

                # This is view scaling only: the uint16 samples received from
                # firmware are passed to the curve unchanged.
                if self.auto_range.isChecked() and processed.size:
                    y_max = max(1.0, float(np.max(processed)))
                    self.plots[edge].setYRange(0.0, y_max * 1.08, padding=0.0)
            else:
                self.processed_curves[edge].setData([], [])
                self.processed_curves[edge].hide()
            if centroid_valid:
                self.centroid_lines[edge].setPos(centroid.position)
                self.centroid_lines[edge].show()
            else:
                self.centroid_lines[edge].hide()
            candidates = snapshot.candidates.get(edge, ())
            self.candidate_curves[edge].setData(
                [c.position for c in candidates],
                [c.peak * 4095.0 for c in candidates]
            )
            selected = snapshot.selection.selected[EDGE_ORDER.index(edge)] if snapshot.selection else 0xFF
            if selected < len(candidates):
                c = candidates[selected]
                self.selected_curves[edge].setData(
                    [c.position],
                    [c.peak * 4095.0])
            else:
                self.selected_curves[edge].setData([], [])
            for offset, line in self.segment_lines[edge]:
                line.setPos(float(offset))
            if not self.auto_range.isChecked():
                self.plots[edge].enableAutoRange(axis="y", enable=False)
            self.plots[edge].setLogMode(y=self.log_scale.isChecked())


class OverviewPanel(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self.canvas = pg.PlotWidget()
        self.canvas.setBackground("#101418")
        self.canvas.setAspectLocked(True)
        self.canvas.setMouseEnabled(x=True, y=True)
        self.canvas.showGrid(x=True, y=True, alpha=0.15)
        self.canvas.setLabel("bottom", "X（内屏有效区左下角 = 0）", units="mm")
        self.canvas.setLabel("left", "Y（内屏有效区左下角 = 0）", units="mm")

        # The inner rectangle is the active screen.  The outer detector frame
        # is expanded by 25 mm on all four sides.  This is display geometry
        # only; all point positions still come directly from firmware.
        assembly = QGraphicsRectItem(QRectF(-OUTER_OFFSET, -OUTER_OFFSET,
                                            ASSEMBLY_W, ASSEMBLY_H))
        assembly.setPen(pg.mkPen("#78909c", width=2))
        assembly.setBrush(pg.mkBrush(0, 0, 0, 0))
        assembly.setZValue(-20)
        self.canvas.addItem(assembly)
        active = QGraphicsRectItem(QRectF(0.0, 0.0, SCREEN_W, SCREEN_H))
        active.setPen(pg.mkPen("#b0bec5", width=2))
        active.setBrush(pg.mkBrush(15, 74, 62, 70))
        active.setZValue(-15)
        self.canvas.addItem(active)

        # Make the test-bench zero unambiguous even though the complete outer
        # frame (and possible off-screen intersections) uses negative axes.
        origin_x = pg.InfiniteLine(pos=0.0, angle=90, movable=False,
                                   pen=pg.mkPen("#ffca28", width=1, style=Qt.DashLine))
        origin_y = pg.InfiniteLine(pos=0.0, angle=0, movable=False,
                                   pen=pg.mkPen("#ffca28", width=1, style=Qt.DashLine))
        origin_x.setZValue(-10)
        origin_y.setZValue(-10)
        self.canvas.addItem(origin_x)
        self.canvas.addItem(origin_y)
        self.origin_dot = self.canvas.plot([0.0], [0.0], pen=None, symbol="+",
                                           symbolSize=15,
                                           symbolPen=pg.mkPen("#ffee58", width=3))
        self.origin_label = pg.TextItem("内屏有效区左下角  (0, 0)",
                                        color="#ffee58", anchor=(0.0, 1.0))
        self.origin_label.setPos(5.0, -3.0)
        self.origin_label.setZValue(20)
        self.canvas.addItem(self.origin_label)
        # Firmware reports four candidate slots in fixed order:
        # top, bottom, left, right.  line_valid_mask marks the two lines that
        # were actually selected for the current crossing.
        self.lines = {}
        self.pair_segments = {}
        for side, color in enumerate(("#29b6f6", "#26c6da", "#ab47bc", "#5c6bc0")):
            line = pg.InfiniteLine(angle=0, movable=False,
                                   pen=pg.mkPen(color, width=2))
            line.hide()
            self.canvas.addItem(line)
            self.lines[side] = line
            segment = self.canvas.plot([], [], pen=pg.mkPen(color, width=3))
            segment.hide()
            self.pair_segments[side] = segment
        self.edge_dots = {
            edge: self.canvas.plot([], [], pen=None, symbol="o", symbolSize=8,
                                   symbolBrush=EDGE_COLORS[edge],
                                   symbolPen=pg.mkPen("#eceff1", width=1))
            for edge in EDGE_ORDER
        }
        self.firmware_dot = self.canvas.plot([], [], pen=None, symbol="o", symbolSize=13,
                                             symbolBrush="#ef5350")
        self.cross_dot = self.canvas.plot([], [], pen=None, symbol="x", symbolSize=14,
                                          symbolPen=pg.mkPen("#ffee58", width=2))
        self.cross_label = pg.TextItem("", color="#ffee58", anchor=(0.0, 1.0))
        self.cross_label.setZValue(30)
        self.cross_label.hide()
        self.canvas.addItem(self.cross_label)
        self.selection_dot = self.canvas.plot([], [], pen=None, symbol="star", symbolSize=16,
                                              symbolBrush="#26c6da")
        self.trail_curve = self.canvas.plot([], [], pen=pg.mkPen("#ff8a65", width=2), connect="finite")
        self.trail_curve.setZValue(5)
        # Only the two aggregate-line intersection is authoritative in the
        # physical view.  Hide legacy final/selection/trail markers so one
        # result cannot look like several different intersections.
        self.firmware_dot.hide()
        self.selection_dot.hide()
        self.trail = deque(maxlen=3000)
        self.trail_connected = False
        self.trail_sequence = None
        self.trail_tick = None
        self.trail_received_at = None
        layout.addWidget(self.canvas, 7)

        side = QWidget()
        side_layout = QHBoxLayout(side)
        side_layout.setContentsMargins(0, 2, 0, 0)
        side_layout.setSpacing(4)
        self.value_labels = {}
        for title, key in (("固件最终坐标", "position"),
                           ("选中两线原始交点", "line"), ("固件稳定输出", "cross")):
            box = QGroupBox(title)
            box_layout = QVBoxLayout(box)
            label = QLabel("未提供")
            label.setAlignment(Qt.AlignCenter)
            label.setStyleSheet("font-size:14px;font-weight:600")
            box_layout.addWidget(label)
            side_layout.addWidget(box, 1)
            self.value_labels[key] = label
        layout.addWidget(side, 1)

        self.cross_meta = QLabel("参与线数 -  X方向选边 -  Y方向选边 -  来源 -  置信度 -")
        self.cross_meta.setWordWrap(True)
        self.cross_meta.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.cross_meta)
        self.centroid_meta = QLabel("八边质心：尚未收到")
        self.centroid_meta.setWordWrap(True)
        self.centroid_meta.setAlignment(Qt.AlignCenter)
        self.centroid_meta.setStyleSheet("color:#b0bec5")
        layout.addWidget(self.centroid_meta)
        self.joint_meta = QLabel("固件选线：尚未收到")
        self.joint_meta.setWordWrap(True)
        self.joint_meta.setAlignment(Qt.AlignCenter)
        self.joint_meta.setStyleSheet("color:#ffca28")
        layout.addWidget(self.joint_meta)
        self.pair_deviation_meta = QLabel("同侧内外质心偏差：尚未收到")
        self.pair_deviation_meta.setWordWrap(True)
        self.pair_deviation_meta.setAlignment(Qt.AlignCenter)
        self.pair_deviation_meta.setStyleSheet("color:#ffcc80")
        layout.addWidget(self.pair_deviation_meta)
        view_row = QHBoxLayout()
        reset_view = QPushButton("复位视图")
        reset_view.clicked.connect(self._fit_mechanical_view)
        view_row.addWidget(reset_view)
        self.line_toggle = QPushButton("隐藏连线")
        self.line_toggle.setCheckable(True)
        self.line_toggle.setChecked(True)
        self.line_toggle.clicked.connect(self._toggle_lines)
        view_row.addWidget(self.line_toggle)
        self.trail_toggle = QCheckBox("显示移动轨迹")
        self.trail_toggle.setChecked(True)
        self.trail_toggle.toggled.connect(self.trail_curve.setVisible)
        view_row.addWidget(self.trail_toggle)
        clear_trail = QPushButton("清空轨迹")
        clear_trail.clicked.connect(self.clear_trail)
        view_row.addWidget(clear_trail)
        view_row.addWidget(QLabel("拖动查看其他象限，滚轮缩放"))
        view_row.addStretch(1)
        layout.addLayout(view_row)
        note = QLabel("彩色圆点＝固件八边质心；彩色直线＝实际求交线；黄色叉号和橙色轨迹＝固件稳定输出。滤波后点可能偏离当前原始求交线；原始交点仅显示数值。UI只显示，不求交点、不滤波。")
        note.setWordWrap(True)
        note.setAlignment(Qt.AlignCenter)
        note.setStyleSheet("color:#90a4ae")
        layout.addWidget(note)
        QTimer.singleShot(0, self._fit_mechanical_view)

    def clear_trail(self) -> None:
        self.trail.clear()
        self.trail_connected = False
        self.trail_sequence = None
        self.trail_tick = None
        self.trail_received_at = None
        self.trail_curve.setData([], [])

    def _append_output(self, received_at, sequence, tick_ms, position) -> None:
        x, y, valid = position
        valid = (valid and math.isfinite(x) and math.isfinite(y) and
                 0.0 <= x <= SCREEN_W and 0.0 <= y <= SCREEN_H)
        # Reject outside-screen product points from old firmware as well.
        # Coordinates are never clamped, filtered or interpolated in UI.
        if (self.trail_sequence is not None and
                (sequence < self.trail_sequence or tick_ms < self.trail_tick)):
            self.trail_connected = False
        if (self.trail_tick is not None and
                tick_ms - self.trail_tick > 500):
            self.trail_connected = False
        if valid:
            if not self.trail_connected and self.trail:
                self.trail.append((math.nan, math.nan))
            if self.trail_received_at != received_at:
                self.trail.append((x, y))
            self.trail_connected = True
        else:
            self.trail_connected = False
        self.trail_sequence = sequence
        self.trail_tick = tick_ms
        self.trail_received_at = received_at

    def _update_output(self, snapshot: Snapshot) -> None:
        # Queued points may already include the displayed latest snapshot.
        if (self.trail_received_at is None or
                snapshot.received_at > self.trail_received_at):
            self._append_output(snapshot.received_at, snapshot.sequence,
                                snapshot.tick_ms, snapshot.position)
        x, y, valid = snapshot.position
        valid = (valid and math.isfinite(x) and math.isfinite(y) and
                 0.0 <= x <= SCREEN_W and 0.0 <= y <= SCREEN_H)
        self.value_labels["position"].setText(self._format_point(x, y, valid))
        self.value_labels["cross"].setText(self._format_point(x, y, valid))
        self.firmware_dot.setData([], [])
        if valid:
            self.cross_dot.setData([x], [y])
            self.cross_label.setText(f"交点 ({x:.2f}, {y:.2f}) mm")
            self.cross_label.setAnchor((1.0 if x > SCREEN_W - 100 else 0.0,
                                        0.0 if y > SCREEN_H - 25 else 1.0))
            self.cross_label.setPos(x, y)
            self.cross_label.show()
        else:
            self.cross_dot.setData([], [])
            self.cross_label.hide()
        self.trail_curve.setData([p[0] for p in self.trail],
                                 [p[1] for p in self.trail], connect="finite")

    def _toggle_lines(self, checked: bool) -> None:
        self.line_toggle.setText("隐藏连线" if checked else "显示连线")
        if not checked:
            for item in self.lines.values():
                item.hide()
            for item in self.pair_segments.values():
                item.hide()

    def _fit_mechanical_view(self) -> None:
        """Reset to the complete detector assembly; user may pan afterwards."""
        view = self.canvas.getViewBox()
        margin = 20.0
        view.setRange(
            xRange=(-OUTER_OFFSET - margin, SCREEN_W + OUTER_OFFSET + margin),
            yRange=(-OUTER_OFFSET - margin, SCREEN_H + OUTER_OFFSET + margin),
            padding=0.0, disableAutoRange=True,
        )

    @staticmethod
    def _format_point(x: float, y: float, valid: bool) -> str:
        return f"({x:.2f}, {y:.2f}) mm" if valid and math.isfinite(x) and math.isfinite(y) else "未提供/无效"

    @staticmethod
    def _along(edge: str, centroid: CentroidValue,
               params: Optional[RuntimeValues]) -> Optional[float]:
        count = EDGE_SENSORS[edge]
        if not centroid.valid or count <= 0 or not math.isfinite(centroid.position):
            return None
        return WaveformPanel._map_axis(edge, centroid.position, params)

    def update_snapshot(self, snapshot: Snapshot) -> None:
        self._update_output(snapshot)

        dbg = snapshot.debug
        if dbg:
            selected_mask = dbg.line_valid_mask & 0x0F
            vertical_point_mask = (dbg.line_valid_mask >> 4) & 0x0F
            vertical_point_count = vertical_point_mask.bit_count()
            if vertical_point_count >= 3:
                x_side = f"上下{vertical_point_count}点竖直融合"
            elif vertical_point_mask == 0x03:
                x_side = "上侧2点竖直融合"
            elif vertical_point_mask == 0x0C:
                x_side = "下侧2点竖直融合"
            else:
                x_side = "无"
            if dbg.cross_source == 3:
                y_side = "左右有效点降级"
            elif dbg.cross_source == 4:
                primary = "左" if selected_mask & 0x04 else "右"
                y_side = f"{primary}侧为主·左右4点约束"
            elif dbg.cross_source == 5:
                primary = "左" if selected_mask & 0x04 else "右"
                rejected_names = ("外左", "内左", "内右", "外右")
                rejected_idx = int(round(dbg.cross_diff))
                rejected = (rejected_names[rejected_idx]
                            if 0 <= rejected_idx < len(rejected_names) else None)
                y_side = (f"{primary}侧为主·3点稳健约束·剔除{rejected}"
                          if rejected else f"{primary}侧为主·3点约束（一路缺失）")
            elif dbg.cross_source == 6:
                primary = "左" if selected_mask & 0x04 else "右"
                y_side = f"{primary}侧为主·短基线方向限幅"
            else:
                y_side = "左" if selected_mask & 0x04 else ("右" if selected_mask & 0x08 else "无")
            self.value_labels["line"].setText(self._format_point(dbg.x_line, dbg.y_line, dbg.have_x and dbg.have_y))
            sources = {0: "无", 1: "当前交点", 2: "交点保持",
                       3: "横向降级交点", 4: "左右4点约束交点",
                       5: "剔除离群点后的3点交点", 6: "短基线限幅交点"}
            self.cross_meta.setText(
                f"参与线数 {dbg.cross_lines}　X方向选边 {x_side}　Y方向选边 {y_side}　"
                f"来源 {sources.get(dbg.cross_source, dbg.cross_source)}　"
                f"置信度 {dbg.cross_conf:.3f}　"
                f"选线掩码 0x{selected_mask:02X}"
            )
            if dbg.cross_out_valid:
                x_desc = f"{x_side}（x=常数）"
                self.joint_meta.setText(
                    f"固件选线：{x_desc} × {y_side}横向线")
            elif dbg.cross_lines == 1:
                only_side = x_side if x_side != "无" else y_side
                missing_axis = "Y（左/右）" if x_side != "无" else "X（上/下）"
                self.joint_meta.setText(
                    f"当前仅有{only_side}光路线；缺少{missing_axis}方向，无法求交")
            else:
                self.joint_meta.setText("当前没有完整的两方向质心线，无法求交")
        else:
            self.value_labels["line"].setText("未提供")
            self.cross_meta.setText("参与线数 -　X方向选边 -　Y方向选边 -　来源 -　置信度 -")
            self.joint_meta.setText("固件选线：尚未收到")

        # Candidate-selection diagnostics are not another positioning result.
        # The overview displays only position_compute's selected two lines and
        # their firmware crossing.
        self.selection_dot.setData([], [])

        for dot in self.edge_dots.values():
            dot.setData([], [])

        point_for_edge = {
            "up": lambda v: (v, SCREEN_H + OUTER_OFFSET),
            "down": lambda v: (v, -OUTER_OFFSET),
            "left": lambda v: (-OUTER_OFFSET, v),
            "right": lambda v: (SCREEN_W + OUTER_OFFSET, v),
            "in_up": lambda v: (v, SCREEN_H),
            "in_down": lambda v: (v, 0.0),
            "in_left": lambda v: (0.0, v),
            "in_right": lambda v: (SCREEN_W, v),
        }
        # Display the exact per-edge millimetre values returned by firmware.
        # Rebuilding the old independent calibration in UI made IN_RIGHT look
        # tens of millimetres away even while firmware used a joint map.
        for edge in EDGE_ORDER:
            centroid = snapshot.centroids.get(edge)
            if centroid is None:
                continue
            value = snapshot.mapped_mm.get(edge)
            if (centroid.valid and value is not None and math.isfinite(value)):
                px, py = point_for_edge[edge](value)
                self.edge_dots[edge].setData([px], [py])

        centroid_text = []
        for edge in EDGE_ORDER:
            centroid = snapshot.centroids.get(edge)
            value = snapshot.mapped_mm.get(edge) if centroid and centroid.valid else None
            centroid_text.append(
                (f"{EDGE_LABELS[edge]} {value:.2f}mm / idx {centroid.position:.3f}"
                 if value is not None and math.isfinite(value) else
                f"{EDGE_LABELS[edge]} 无效"
                )
            )
        self.centroid_meta.setText("固件八边实际质心（毫米 / 质心索引）：" + "　".join(centroid_text))

        # In a perpendicular cross optical path, the inner/outer members of
        # the same side must report the same along-edge physical coordinate.
        # This is deliberately a display-only diagnostic: it does not alter
        # firmware centroids, calibration maps, fitted lines, or the crossing.
        mapped = {}
        for edge in EDGE_ORDER:
            centroid = snapshot.centroids.get(edge)
            value = snapshot.mapped_mm.get(edge) if centroid and centroid.valid else None
            if value is not None and math.isfinite(value):
                mapped[edge] = value
        pair_defs = (
            ("上", "up", "in_up"),
            ("下", "down", "in_down"),
            ("左", "left", "in_left"),
            ("右", "right", "in_right"),
        )
        pair_text = []
        pair_abs = []
        for label, outer, inner in pair_defs:
            if outer in mapped and inner in mapped:
                delta = mapped[outer] - mapped[inner]
                pair_abs.append(abs(delta))
                pair_text.append(f"{label} Δ={delta:+.2f}mm")
            else:
                pair_text.append(f"{label} Δ=无效")
        x_values = [mapped[e] for e in ("up", "down", "in_up", "in_down") if e in mapped]
        y_values = [mapped[e] for e in ("left", "right", "in_left", "in_right") if e in mapped]
        axis_spreads = []
        if len(x_values) >= 2:
            x_spread = max(x_values) - min(x_values)
            axis_spreads.append(x_spread)
            pair_text.append(f"X四边离散={x_spread:.2f}mm")
        if len(y_values) >= 2:
            y_spread = max(y_values) - min(y_values)
            axis_spreads.append(y_spread)
            pair_text.append(f"Y四边离散={y_spread:.2f}mm")
        if pair_abs or axis_spreads:
            if pair_abs:
                pair_text.append(f"最大|Δ|={max(pair_abs):.2f}mm")
            max_delta = max(pair_abs + axis_spreads)
            color = "#a5d6a7" if max_delta <= 1.0 else ("#ffcc80" if max_delta <= 2.0 else "#ef9a9a")
            self.pair_deviation_meta.setStyleSheet(f"color:{color}")
        mapping_mode = "固件回传的各边映射坐标"
        self.pair_deviation_meta.setText(
            f"固件选线前质心偏差（外−内，{mapping_mode}）：" + "　".join(pair_text)
        )

        pair_endpoints = {
            0: ((mapped.get("up"), SCREEN_H + OUTER_OFFSET),
                (mapped.get("in_up"), SCREEN_H)),
            1: ((mapped.get("down"), -OUTER_OFFSET),
                (mapped.get("in_down"), 0.0)),
            2: ((-OUTER_OFFSET, mapped.get("left")),
                (0.0, mapped.get("in_left"))),
            3: ((SCREEN_W + OUTER_OFFSET, mapped.get("right")),
                (SCREEN_W, mapped.get("in_right"))),
        }

        # With a valid crossing, extend both transmitted firmware lines so the
        # intersection remains easy to inspect.  With only one direction,
        # display just the actual 25 mm inner/outer centroid segment; an
        # infinite single line could be mistaken for a valid 2-D solution.
        for side in range(4):
            if (self.line_toggle.isChecked() and dbg and side < len(dbg.lines)
                    and (dbg.line_valid_mask & (1 << side))):
                x1, y1, x2, y2 = dbg.lines[side]
                dx, dy = x2 - x1, y2 - y1
                if (dbg.cross_out_valid and math.isfinite(dx) and
                        math.isfinite(dy) and math.hypot(dx, dy) > 1e-6):
                    self.lines[side].setPos((x1, y1))
                    self.lines[side].setAngle(math.degrees(math.atan2(dy, dx)))
                    self.lines[side].show()
                    self.pair_segments[side].hide()
                else:
                    self.lines[side].hide()
                    if side == 0 and vertical_point_count >= 3:
                        p1, p2 = (x1, y1), (x2, y2)
                    else:
                        p1, p2 = pair_endpoints[side]
                    if all(v is not None and math.isfinite(v)
                           for v in (*p1, *p2)):
                        self.pair_segments[side].setData(
                            [p1[0], p2[0]], [p1[1], p2[1]])
                        self.pair_segments[side].show()
                    else:
                        self.pair_segments[side].hide()
            else:
                self.lines[side].hide()
                self.pair_segments[side].hide()


class DiagnosticsPanel(QWidget):
    HEADERS = ("传感器索引", "板端处理后")

    def __init__(self):
        super().__init__()
        layout = QVBoxLayout(self)
        controls = QHBoxLayout()
        controls.addWidget(QLabel("查看边"))
        self.edge_box = QComboBox()
        for edge in EDGE_ORDER:
            self.edge_box.addItem(EDGE_LABELS[edge], edge)
        controls.addWidget(self.edge_box)
        self.firmware_result = QLabel("固件质心 -　峰值 -　有效 -")
        controls.addWidget(self.firmware_result, 1)
        layout.addLayout(controls)
        note = QLabel("逐点原样显示板端处理后的 uint16；UI不计算最大值、均值、差值、滤波或门限。")
        note.setStyleSheet("color:#90a4ae")
        layout.addWidget(note)
        self.table = QTableWidget(0, len(self.HEADERS))
        self.table.setHorizontalHeaderLabels(self.HEADERS)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        layout.addWidget(self.table, 1)
        self.latest: Optional[Snapshot] = None
        self.edge_box.currentIndexChanged.connect(self._render)

    def update_snapshot(self, snapshot: Snapshot) -> None:
        self.latest = snapshot
        self._render()

    def _render(self) -> None:
        if self.latest is None:
            return
        edge = self.edge_box.currentData()
        processed = self.latest.processed.get(edge)
        count = processed.size if processed is not None else 0
        self.table.setRowCount(count)
        centroid = self.latest.centroids.get(edge)
        if centroid:
            self.firmware_result.setText(
                f"固件质心 {centroid.position:.3f}　峰值 {centroid.peak:.4f}　有效 {'是' if centroid.valid else '否'}"
            )
        else:
            self.firmware_result.setText("固件质心 -　峰值 -　有效 -")
        for row in range(count):
            values = (str(row),
                      str(int(processed[row])) if processed is not None and row < processed.size else "-")
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(row, col, item)


class ParameterPanel(QWidget):
    def __init__(self, apply_callback):
        super().__init__()
        self.apply_callback = apply_callback
        self.loaded = False
        self.awaiting_confirmation = False
        self.pending_values: Optional[RuntimeValues] = None
        self.firmware_values: Optional[RuntimeValues] = None
        layout = QVBoxLayout(self)
        note = QLabel("这里修改的是固件运行参数。UI不使用这些参数计算数据；固件回传生效值后才更新状态。参数断电恢复编译默认值。")
        note.setWordWrap(True)
        note.setStyleSheet("color:#90a4ae")
        layout.addWidget(note)
        self.centroid_filter = QCheckBox("固件质心滤波")
        self.position_filter = QCheckBox("固件坐标滤波")
        layout.addWidget(self.centroid_filter)
        layout.addWidget(self.position_filter)
        self.fields = {}
        for label, key, lo, hi, step, decimals in (
            ("质心有效门限", "threshold", 0.0001, 0.5, 0.0005, 4),
            ("交点最低置信度", "confidence", 0.0, 1.0, 0.01, 2),
            ("坐标跳变门限 mm", "jump", 1.0, 500.0, 1.0, 1),
        ):
            row = QHBoxLayout()
            row.addWidget(QLabel(label))
            box = QDoubleSpinBox()
            box.setRange(lo, hi); box.setSingleStep(step); box.setDecimals(decimals)
            row.addWidget(box)
            layout.addLayout(row)
            self.fields[key] = box
        self.apply_button = QPushButton("下发到固件")
        self.apply_button.clicked.connect(self._apply)
        layout.addWidget(self.apply_button)
        self.state = QLabel("尚未收到固件参数")
        layout.addWidget(self.state)
        layout.addStretch(1)

    def _apply(self):
        values = RuntimeValues(
            self.centroid_filter.isChecked(), self.position_filter.isChecked(),
            self.fields["threshold"].value(),
            self.firmware_values.position_outlier_mm if self.firmware_values else 25.0,
            self.fields["confidence"].value(), self.fields["jump"].value())
        self.pending_values = values
        self.apply_callback(values)
        self.awaiting_confirmation = True
        self.state.setText("已发送，等待固件回传确认")

    def update_snapshot(self, snapshot: Snapshot):
        p = snapshot.params
        if p is None:
            return
        self.firmware_values = p
        q = self.pending_values
        confirmed_now = bool(self.awaiting_confirmation and q and
                             p.centroid_filter == q.centroid_filter and
                             p.position_filter == q.position_filter and
                             abs(p.centroid_threshold - q.centroid_threshold) < 1e-5 and
                             abs(p.position_outlier_mm - q.position_outlier_mm) < 0.05 and
                             abs(p.cross_min_conf - q.cross_min_conf) < 1e-4 and
                             abs(p.position_jump_mm - q.position_jump_mm) < 0.05)
        if confirmed_now:
            self.state.setText(f"固件已确认：帧 {snapshot.sequence}")
            self.awaiting_confirmation = False
            self.pending_values = None
        elif self.awaiting_confirmation:
            self.state.setText(f"等待固件确认：帧 {snapshot.sequence}")
        else:
            self.state.setText(f"固件当前参数：帧 {snapshot.sequence}")
        if not self.loaded or confirmed_now:
            self.centroid_filter.setChecked(p.centroid_filter)
            self.position_filter.setChecked(p.position_filter)
            self.fields["threshold"].setValue(p.centroid_threshold)
            self.fields["confidence"].setValue(p.cross_min_conf)
            self.fields["jump"].setValue(p.position_jump_mm)
            self.loaded = True


class CalibrationPanel(QWidget):
    """Guide calibration and render firmware-owned fit results unchanged."""
    def __init__(self, command_callback):
        super().__init__()
        self.command_callback = command_callback
        self.latest_params: Optional[RuntimeValues] = None
        # Deterministic snake traversal of every planned grid point.  This is
        # a fixed test sequence, not a generated/random calibration pattern.
        self.plan = []
        for row, y in enumerate(CALIB_GRID_Y):
            xs = CALIB_GRID_X if row % 2 == 0 else tuple(reversed(CALIB_GRID_X))
            self.plan.extend((x, y) for x in xs)

        root = QHBoxLayout(self)
        controls = QWidget()
        left = QVBoxLayout(controls)
        note = QLabel(
            "标定时必须使用垂直入射的十字光。将光斑中心放在已知的内屏坐标，"
            "按照固定的7×3、共21点蛇形顺序逐点记录，不使用随机点。"
            "21点全部取自原45点：长边保留端部和PCB拼接区，短边保留底/中/顶。"
            "UI只下发真实坐标并显示固件结果；"
            "固件会先等待质心滤波稳定，再对与产品连线完全相同的滤波质心取中位数；"
            "单调检查、分段拟合和运行时映射均在固件端完成。"
            "当前1mm是上下四边/左右四边一致性的观察目标，不会因为暂时超差而隐藏融合线。"
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#cfd8dc")
        left.addWidget(note)

        target_group = QGroupBox("当前已知坐标")
        target_layout = QVBoxLayout(target_group)
        target_row = QHBoxLayout()
        self.target_x = QDoubleSpinBox(); self.target_x.setRange(0, SCREEN_W); self.target_x.setDecimals(1); self.target_x.setSuffix(" mm")
        self.target_y = QDoubleSpinBox(); self.target_y.setRange(0, SCREEN_H); self.target_y.setDecimals(1); self.target_y.setSuffix(" mm")
        self.capture_count = QSpinBox(); self.capture_count.setRange(5, 32); self.capture_count.setValue(20)
        target_row.addWidget(QLabel("X")); target_row.addWidget(self.target_x)
        target_row.addWidget(QLabel("Y")); target_row.addWidget(self.target_y)
        target_row.addWidget(QLabel("采样帧")); target_row.addWidget(self.capture_count)
        target_layout.addLayout(target_row)

        plan_row = QHBoxLayout()
        self.plan_step = QSpinBox(); self.plan_step.setRange(1, len(self.plan)); self.plan_step.setValue(1)
        self.load_step_button = QPushButton("加载推荐点")
        self.prev_button = QPushButton("上一点")
        self.next_button = QPushButton("下一点")
        plan_row.addWidget(QLabel(f"点阵图固定流程（共 {len(self.plan)} 点）"))
        plan_row.addWidget(self.plan_step)
        plan_row.addWidget(self.load_step_button)
        plan_row.addWidget(self.prev_button)
        plan_row.addWidget(self.next_button)
        target_layout.addLayout(plan_row)
        left.addWidget(target_group)

        action_row = QHBoxLayout()
        self.capture_button = QPushButton("记录当前点")
        self.apply_button = QPushButton("固件检查、拟合并应用")
        self.clear_button = QPushButton("清空全部标定")
        action_row.addWidget(self.capture_button)
        action_row.addWidget(self.apply_button)
        action_row.addWidget(self.clear_button)
        left.addLayout(action_row)
        self.state_label = QLabel("尚未收到固件标定状态")
        self.state_label.setWordWrap(True)
        left.addWidget(self.state_label)
        self.persistence_label = QLabel("板载Flash：等待固件状态；本机备份：等待检查")
        self.persistence_label.setWordWrap(True)
        self.persistence_label.setStyleSheet("color:#90caf9")
        left.addWidget(self.persistence_label)

        self.completion_label = QLabel(
            "唯一节点说明：横向边按X合并，目标7个；纵向边按Y合并，目标3个。"
            "同一X/Y的重复记录用于平均，不会增加节点数量。"
        )
        self.completion_label.setWordWrap(True)
        self.completion_label.setStyleSheet(
            "padding:6px;border:1px solid #607d8b;border-radius:3px;color:#cfd8dc"
        )
        left.addWidget(self.completion_label)

        self.table = QTableWidget(len(EDGE_ORDER), 8)
        self.table.setHorizontalHeaderLabels((
            "边", "唯一节点", "推荐目标", "最近记录有效帧", "应用状态",
            "LOO RMS/mm(诊断)", "索引范围", "毫米范围"
        ))
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setEditTriggers(QTableWidget.NoEditTriggers)
        left.addWidget(self.table, 1)

        fit_group = QGroupBox("八边标定整体物理效果")
        fit_layout = QVBoxLayout(fit_group)
        edge_row = QHBoxLayout()
        self.edge_box = QComboBox()
        for edge in EDGE_ORDER:
            self.edge_box.addItem(EDGE_LABELS[edge], edge)
        edge_row.addWidget(QLabel("下方明细查看边")); edge_row.addWidget(self.edge_box); edge_row.addStretch(1)
        fit_layout.addLayout(edge_row)
        self.plot = pg.PlotWidget()
        self.plot.setBackground("#101820")
        self.plot.showGrid(x=True, y=True, alpha=0.22)
        self.plot.setAspectLocked(True)
        self.plot.setLabel("bottom", "物理 X 坐标", units="mm")
        self.plot.setLabel("left", "物理 Y 坐标", units="mm")
        self.plot.setXRange(-OUTER_OFFSET - 15.0, SCREEN_W + OUTER_OFFSET + 15.0, padding=0)
        self.plot.setYRange(-OUTER_OFFSET - 15.0, SCREEN_H + OUTER_OFFSET + 15.0, padding=0)
        self.plot.addLegend(offset=(12, 12), labelTextSize="10pt")
        outer_frame = QGraphicsRectItem(QRectF(-OUTER_OFFSET, -OUTER_OFFSET,
                                                ASSEMBLY_W, ASSEMBLY_H))
        outer_frame.setPen(pg.mkPen("#78909c", width=1.5))
        inner_frame = QGraphicsRectItem(QRectF(0.0, 0.0, SCREEN_W, SCREEN_H))
        inner_frame.setPen(pg.mkPen("#cfd8dc", width=1.5))
        outer_frame.setZValue(-10); inner_frame.setZValue(-10)
        self.plot.addItem(outer_frame); self.plot.addItem(inner_frame)
        self.calibration_curves = {}
        for edge in EDGE_ORDER:
            self.calibration_curves[edge] = self.plot.plot(
                [], [], pen=pg.mkPen(EDGE_COLORS[edge], width=2.4),
                symbol="o", symbolSize=9, symbolBrush=EDGE_COLORS[edge],
                symbolPen=pg.mkPen("#eceff1", width=1), name=EDGE_LABELS[edge]
            )
        fit_layout.addWidget(self.plot, 1)
        self.fit_explain = QLabel(
            "灰框＝实际外框和内屏；彩色点＝固件保存的八边唯一标定节点；"
            "同色连线＝该边节点在统一物理坐标中的覆盖范围。"
            "实线表示已应用，虚线表示节点尚未完整或尚未应用。"
        )
        self.fit_explain.setWordWrap(True)
        self.fit_explain.setStyleSheet("color:#b0bec5")
        fit_layout.addWidget(self.fit_explain)

        self.node_table = QTableWidget(0, 3)
        self.node_table.setHorizontalHeaderLabels(("节点", "滤波质心索引", "真实坐标/mm"))
        self.node_table.verticalHeader().setVisible(False)
        self.node_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.node_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.node_table.setMaximumHeight(220)
        fit_layout.addWidget(self.node_table)
        self.fit_info = QLabel("暂无节点")
        fit_layout.addWidget(self.fit_info)

        root.addWidget(controls, 5)
        root.addWidget(fit_group, 4)

        self.capture_button.clicked.connect(self._capture)
        self.apply_button.clicked.connect(lambda: self.command_callback(("calib_apply",)))
        self.clear_button.clicked.connect(self._confirm_clear)
        self.load_step_button.clicked.connect(self._load_step)
        self.prev_button.clicked.connect(lambda: self._move_step(-1))
        self.next_button.clicked.connect(lambda: self._move_step(1))
        self.edge_box.currentIndexChanged.connect(self._render_plot)
        self._load_step()

    def _load_step(self) -> None:
        x, y = self.plan[self.plan_step.value() - 1]
        self.target_x.setValue(x)
        self.target_y.setValue(y)

    def _move_step(self, delta: int) -> None:
        self.plan_step.setValue(max(1, min(len(self.plan), self.plan_step.value() + delta)))
        self._load_step()

    def _capture(self) -> None:
        self.command_callback(("calib_capture", self.target_x.value(),
                               self.target_y.value(), self.capture_count.value()))

    def _confirm_clear(self) -> None:
        answer = QMessageBox.warning(
            self, "确认清空标定",
            "此操作会清空主控当前使用的全部标定节点并擦除板载Flash。\n"
            "本机安全备份会保留，但重新开始标定前请确认确实需要清空。",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No,
        )
        if answer == QMessageBox.Yes:
            self.command_callback(("calib_clear",))

    def set_persistence_status(self, text: str) -> None:
        self.persistence_label.setText(text)

    def update_snapshot(self, snapshot: Snapshot) -> None:
        if snapshot.params is None:
            return
        self.latest_params = snapshot.params
        p = snapshot.params
        states = {0: "空闲/未标定", 1: "固件正在采样", 2: "标定点已记录，等待应用",
                  3: "拟合已应用并保存", 4: "拟合检查未通过",
                  5: "拟合已应用，但Flash保存失败", 6: "内存已清空，但Flash擦除失败"}
        progress = ""
        if p.calibration_state == 1 and p.calibration_requested:
            done = p.calibration_requested - p.calibration_pending
            progress = f"；进度 {done}/{p.calibration_requested} 帧"
        state_text = states.get(p.calibration_state, p.calibration_state)
        if p.calibration_valid_mask != 0xFF and p.calibration_state in (3, 4):
            state_text = "部分边验证通过；八边未完整，标定映射尚未启用"
        self.state_label.setText(
            f"固件状态：{state_text}{progress}；有效边掩码 0x{p.calibration_valid_mask:02X}"
        )

        expected = {
            edge: (len(CALIB_GRID_X) if edge in ("up", "down", "in_up", "in_down")
                   else len(CALIB_GRID_Y))
            for edge in EDGE_ORDER
        }
        passed_edges = [EDGE_LABELS[e] for i, e in enumerate(EDGE_ORDER)
                        if p.calibration_valid_mask & (1 << i)]
        missing_edges = [EDGE_LABELS[e] for i, e in enumerate(EDGE_ORDER)
                         if not (p.calibration_valid_mask & (1 << i))]
        ready_full = all(
            i < len(p.calibration_point_count) and
            p.calibration_point_count[i] >= expected[edge]
            for i, edge in enumerate(EDGE_ORDER)
        )
        self.apply_button.setEnabled(ready_full and p.calibration_state != 1)
        self.apply_button.setToolTip(
            "八条边达到推荐节点数后才可应用：横向边7个X节点，纵向边3个Y节点。"
            if not ready_full else "八条边节点完整，可以执行固件检查、拟合并保存。"
        )
        completeness = "；".join(
            f"{EDGE_LABELS[e]} {p.calibration_point_count[i]}/{expected[e]}"
            for i, e in enumerate(EDGE_ORDER)
        )
        horizontal_nodes = max((p.calibration_point_count[i] for i in (0, 1, 4, 5)), default=0)
        vertical_nodes = max((p.calibration_point_count[i] for i in (2, 3, 6, 7)), default=0)
        historical_grid = horizontal_nodes * vertical_nodes
        if p.calibration_valid_mask == 0xFF:
            headline = "完整标定：8/8条边均已应用。"
            style = "padding:6px;border:1px solid #43a047;border-radius:3px;color:#a5d6a7"
        else:
            headline = (
                f"当前验证通过 {len(passed_edges)}/8 条边；为避免内外映射混用，八边标定均未启用。"
                f"验证通过：{'、'.join(passed_edges) if passed_edges else '无'}；"
                f"未通过：{'、'.join(missing_edges) if missing_edges else '无'}。"
            )
            style = "padding:6px;border:1px solid #ef6c00;border-radius:3px;color:#ffcc80"
        self.completion_label.setText(
            headline + "\n唯一节点进度：" + completeness +
            f"\n当前节点覆盖为 {horizontal_nodes}×{vertical_nodes}，对应当前 {historical_grid} 个固定标定位置；"
            "每次照射会同时记录可见的八条边，并按相同目标X/Y合并为唯一节点。"
        )
        self.completion_label.setStyleSheet(style)
        for i, edge in enumerate(EDGE_ORDER):
            raw = p.calibration_raw[i] if i < len(p.calibration_raw) else ()
            mm = p.calibration_mm[i] if i < len(p.calibration_mm) else ()
            values = (
                EDGE_LABELS[edge],
                str(p.calibration_point_count[i]) if i < len(p.calibration_point_count) else "0",
                str(expected[edge]),
                str(p.calibration_last_samples[i]) if i < len(p.calibration_last_samples) else "0",
                ("已应用" if p.calibration_valid_mask == 0xFF else
                 ("验证通过(待整体)" if p.calibration_valid_mask & (1 << i) else "未通过")),
                f"{p.calibration_rms[i]:.3f}" if i < len(p.calibration_rms) and p.calibration_rms[i] >= 0 else "-",
                f"{min(raw):.2f} … {max(raw):.2f}" if raw else "-",
                f"{min(mm):.1f} … {max(mm):.1f}" if mm else "-",
            )
            for col, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.table.setItem(i, col, item)
        self._render_plot()

    def _render_plot(self) -> None:
        if self.latest_params is None:
            for curve in self.calibration_curves.values():
                curve.setData([], [])
            return
        p = self.latest_params
        for edge_index, plot_edge in enumerate(EDGE_ORDER):
            mm_values = p.calibration_mm[edge_index]
            if plot_edge == "up":
                xs, ys = mm_values, [SCREEN_H + OUTER_OFFSET] * len(mm_values)
            elif plot_edge == "down":
                xs, ys = mm_values, [-OUTER_OFFSET] * len(mm_values)
            elif plot_edge == "in_up":
                xs, ys = mm_values, [SCREEN_H] * len(mm_values)
            elif plot_edge == "in_down":
                xs, ys = mm_values, [0.0] * len(mm_values)
            elif plot_edge == "left":
                xs, ys = [-OUTER_OFFSET] * len(mm_values), mm_values
            elif plot_edge == "right":
                xs, ys = [SCREEN_W + OUTER_OFFSET] * len(mm_values), mm_values
            elif plot_edge == "in_left":
                xs, ys = [0.0] * len(mm_values), mm_values
            else:
                xs, ys = [SCREEN_W] * len(mm_values), mm_values
            applied = p.calibration_valid_mask == 0xFF
            self.calibration_curves[plot_edge].setPen(pg.mkPen(
                EDGE_COLORS[plot_edge], width=2.8 if applied else 1.8,
                style=Qt.SolidLine if applied else Qt.DashLine
            ))
            self.calibration_curves[plot_edge].setData(xs, ys)

        edge = self.edge_box.currentData()
        i = EDGE_ORDER.index(edge)
        raw = p.calibration_raw[i]
        mm = p.calibration_mm[i]
        self.node_table.setRowCount(len(raw))
        for row, (raw_value, mm_value) in enumerate(zip(raw, mm), start=1):
            for col, value in enumerate((str(row), f"{raw_value:.3f}", f"{mm_value:.1f}")):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignCenter)
                self.node_table.setItem(row - 1, col, item)
        rms = self.latest_params.calibration_rms[i]
        edge_passed = bool(self.latest_params.calibration_valid_mask & (1 << i))
        fully_applied = self.latest_params.calibration_valid_mask == 0xFF
        expected = len(CALIB_GRID_X) if edge in ("up", "down", "in_up", "in_down") else len(CALIB_GRID_Y)
        self.fit_info.setText(
            f"{EDGE_LABELS[edge]}：{len(raw)} 个固件节点；"
            f"推荐 {expected} 个；留一节点 RMS {rms:.3f} mm；当前"
            f"{'已应用' if fully_applied else ('验证通过，等待八边完整' if edge_passed else '未通过')}"
            if raw else f"{EDGE_LABELS[edge]}：暂无节点；推荐 {expected} 个"
        )


class UnifiedWindow(QMainWindow):
    def __init__(self, initial_port: Optional[str] = None):
        super().__init__()
        self.setWindowTitle("八边光学膜 综合产品调试台")
        self.resize(1800, 1000)
        self.receiver: Optional[SerialReceiver] = None
        self.last_received_at = 0.0
        self.last_sequence: Optional[int] = None
        self.latest_snapshot: Optional[Snapshot] = None
        self.trail = deque(maxlen=300)
        self.recording = False
        self.record_rows = []
        self.settings = QSettings("EightEdgeOpticalFilm", "UnifiedDebugUI")
        self.calibration_backup, self.calibration_backup_time = self._load_calibration_backup()
        self._calibration_backup_signature = (
            json.dumps(self.calibration_backup, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
            if self.calibration_backup else ""
        )
        self._empty_calibration_frames = 0
        self._restore_attempted = False
        self._suppress_auto_restore = False
        self._stale_visuals_cleared = False

        root = QWidget()
        self.setCentralWidget(root)
        layout = QVBoxLayout(root)

        connection = QGroupBox("连接与数据源")
        row = QHBoxLayout(connection)
        self.port_box = QComboBox()
        self.baud_box = QComboBox()
        self.baud_box.addItems(["115200", "230400", "460800", "921600", "1500000", "2000000"])
        self.baud_box.setCurrentText(BAUD_DEFAULT)
        self.refresh_button = QPushButton("刷新串口")
        self.connect_button = QPushButton("连接")
        self.mouse_box = QCheckBox("鼠标输出")
        self.record_button = QPushButton("开始记录")
        self.save_button = QPushButton("保存记录")
        self.snapshot_button = QPushButton("保存当前帧")
        self.stream_label = QLabel("未连接")
        self.rate_label = QLabel("0.0 FPS　0.0 KiB/s")
        self.integrity_label = QLabel("帧 0　错误 0　丢序 0")
        row.addWidget(QLabel("串口"))
        row.addWidget(self.port_box)
        row.addWidget(QLabel("波特率"))
        row.addWidget(self.baud_box)
        row.addWidget(self.refresh_button)
        row.addWidget(self.connect_button)
        row.addWidget(self.mouse_box)
        row.addWidget(self.record_button)
        row.addWidget(self.save_button)
        row.addWidget(self.snapshot_button)
        row.addSpacing(20)
        row.addWidget(self.stream_label, 1)
        row.addWidget(self.rate_label)
        row.addWidget(self.integrity_label)
        layout.addWidget(connection)

        self.tabs = QTabWidget()
        self.overview = OverviewPanel()
        self.waveforms = WaveformPanel()
        self.diagnostics = DiagnosticsPanel()
        self.parameters = ParameterPanel(self.send_parameters)
        self.calibration = CalibrationPanel(self.send_parameters)
        if self.calibration_backup:
            self.calibration.set_persistence_status(
                f"板载Flash：等待固件状态；本机安全备份：{self.calibration_backup_time}"
            )
        else:
            self.calibration.set_persistence_status("板载Flash：等待固件状态；本机安全备份：尚未建立")
        monitor = QWidget()
        monitor_layout = QHBoxLayout(monitor)
        monitor_layout.setContentsMargins(2, 2, 2, 2)
        monitor_layout.setSpacing(6)
        monitor_layout.addWidget(self.overview, 6)
        monitor_layout.addWidget(self.waveforms, 5)
        self.tabs.addTab(monitor, "物理屏幕・连线・八边波形")
        self.tabs.addTab(self.diagnostics, "逐传感器原值")
        self.tabs.addTab(self.calibration, "固件几何标定")
        self.tabs.addTab(self.parameters, "固件调参")
        layout.addWidget(self.tabs, 1)

        self.setStatusBar(QStatusBar())
        self.statusBar().showMessage("UI只校验协议、原样显示和连线；不做统计、滤波、门限、质心、候选、交点或定位")
        self.refresh_button.clicked.connect(lambda: self.refresh_ports())
        self.connect_button.clicked.connect(self.toggle_connection)
        self.record_button.clicked.connect(self.toggle_recording)
        self.save_button.clicked.connect(self.save_recording)
        self.snapshot_button.clicked.connect(self.save_snapshot)
        self.refresh_ports(initial_port)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.refresh_view)
        self.timer.start(33)
        self._ui_font_px = 0
        QTimer.singleShot(0, self._apply_adaptive_fonts)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._apply_adaptive_fonts()

    def _apply_adaptive_fonts(self) -> None:
        """Scale controls and plot text with the window, within usable limits."""
        scale = min(self.width() / 1800.0, self.height() / 1000.0)
        font_px = max(11, min(20, int(round(12.0 * scale))))
        if font_px == self._ui_font_px:
            return
        self._ui_font_px = font_px
        self.setStyleSheet(
            f"QWidget{{font-size:{font_px}px;}} "
            f"QGroupBox{{font-weight:600;margin-top:{max(8, font_px - 3)}px;}} "
            "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;} "
            f"QTableWidget{{font-size:{font_px}px;}} "
            f"QHeaderView::section{{font-size:{font_px}px;padding:{max(3, font_px // 3)}px;}}"
        )
        plot_font = QFont()
        plot_font.setPixelSize(max(10, font_px - 1))
        for plot in self.findChildren(pg.PlotWidget):
            item = plot.getPlotItem()
            for axis_name in ("left", "bottom", "top", "right"):
                axis = item.getAxis(axis_name)
                axis.setStyle(tickFont=plot_font)
                axis.label.setFont(plot_font)
            if item.titleLabel is not None:
                item.titleLabel.item.setFont(plot_font)

    @staticmethod
    def _calibration_backup_from_params(p: RuntimeValues) -> Optional[dict]:
        """Serialize a complete eight-edge node set without fitting it in UI.

        A complete set that fails strict firmware validation is still worth
        preserving before a diagnostic firmware update.  The receiving
        firmware always revalidates imported nodes before it writes Flash.
        """
        if p.calibration_state not in (2, 3, 4):
            return None
        if len(p.calibration_point_count) != len(EDGE_ORDER):
            return None
        edges = []
        for i, edge in enumerate(EDGE_ORDER):
            count = int(p.calibration_point_count[i])
            expected = len(CALIB_GRID_X) if edge in ("up", "down", "in_up", "in_down") else len(CALIB_GRID_Y)
            raw = list(p.calibration_raw[i]) if i < len(p.calibration_raw) else []
            mm = list(p.calibration_mm[i]) if i < len(p.calibration_mm) else []
            if count < expected or count > CALIB_MAX_POINTS or len(raw) != count or len(mm) != count:
                return None
            if not all(math.isfinite(v) for v in raw + mm):
                return None
            edges.append({"edge": edge, "count": count, "raw": raw, "mm": mm})
        return {"schema": 5, "layout": "vertical-joint-7x3-30_35-35_30",
                "estimator": "filtered_centroid", "valid_mask": 0xFF,
                "source_valid_mask": int(p.calibration_valid_mask),
                "edges": edges}

    def _load_calibration_backup(self) -> tuple[Optional[dict], str]:
        text = self.settings.value("calibration/record_v5_vertical_joint", "", type=str)
        saved_at = self.settings.value("calibration/saved_at_v5", "未知时间", type=str)
        if not text:
            return None, saved_at
        try:
            record = json.loads(text)
            if (record.get("schema") != 5 or
                    record.get("layout") != "vertical-joint-7x3-30_35-35_30" or
                    record.get("estimator") != "filtered_centroid" or
                    record.get("valid_mask") != 0xFF):
                return None, saved_at
            edges = record.get("edges", [])
            if len(edges) != len(EDGE_ORDER):
                return None, saved_at
            for i, item in enumerate(edges):
                count = int(item.get("count", 0))
                raw = item.get("raw", [])
                mm = item.get("mm", [])
                if item.get("edge") != EDGE_ORDER[i] or not (2 <= count <= CALIB_MAX_POINTS):
                    return None, saved_at
                if len(raw) != count or len(mm) != count:
                    return None, saved_at
                if not all(math.isfinite(float(v)) for v in raw + mm):
                    return None, saved_at
            return record, saved_at
        except (TypeError, ValueError, json.JSONDecodeError):
            return None, saved_at

    def _save_calibration_backup(self, p: RuntimeValues) -> bool:
        record = self._calibration_backup_from_params(p)
        if record is None:
            return False
        signature = json.dumps(record, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        if signature != self._calibration_backup_signature:
            saved_at = QDateTime.currentDateTime().toString("yyyy-MM-dd HH:mm:ss")
            self.settings.setValue("calibration/record_v5_vertical_joint", signature)
            self.settings.setValue("calibration/saved_at_v5", saved_at)
            self.settings.sync()
            self.calibration_backup = record
            self.calibration_backup_time = saved_at
            self._calibration_backup_signature = signature
        return True

    def _handle_calibration_persistence(self, p: Optional[RuntimeValues]) -> None:
        if p is None:
            return
        if self._save_calibration_backup(p):
            self._empty_calibration_frames = 0
            if p.calibration_valid_mask == 0xFF:
                status = "板载Flash：完整标定已加载并正在应用"
            else:
                status = (f"板载Flash：当前验证通过 {p.calibration_valid_mask.bit_count()}/8；"
                          "完整八边节点已作本机调试备份")
            self.calibration.set_persistence_status(
                f"{status}；本机安全备份：{self.calibration_backup_time}")
            return
        counts = tuple(p.calibration_point_count)
        flash_empty = p.calibration_valid_mask == 0 and counts and not any(counts)
        if flash_empty and self.calibration_backup:
            self._empty_calibration_frames += 1
            if (self._empty_calibration_frames >= 3 and not self._restore_attempted
                    and not self._suppress_auto_restore):
                self._restore_attempted = True
                self.calibration.set_persistence_status(
                    f"板载Flash：空；正在从本机安全备份 {self.calibration_backup_time} 恢复并重新应用…"
                )
                self.send_parameters(("calib_restore", self.calibration_backup))
            elif not self._restore_attempted:
                self.calibration.set_persistence_status(
                    f"板载Flash：空；本机安全备份：{self.calibration_backup_time}，等待自动恢复"
                )
            return
        self._empty_calibration_frames = 0
        if self.calibration_backup:
            self.calibration.set_persistence_status(
                f"板载Flash：当前为未完成/未应用状态；本机安全备份仍保留：{self.calibration_backup_time}"
            )
        else:
            self.calibration.set_persistence_status("板载Flash：当前无完整标定；本机安全备份：尚未建立")

    def refresh_ports(self, select: Optional[str] = None) -> None:
        current = select or self.port_box.currentText()
        self.port_box.clear()
        if not SERIAL_OK:
            self.stream_label.setText("缺少 pyserial")
            return
        infos = list(list_ports.comports())
        ports = []
        # Enumerate without opening devices: probing toggles DTR/RTS and
        # hides occupied ports, making a normal refresh disruptive.
        for info in infos:
            device = info.device
            if self.receiver is not None and device == self.receiver.port:
                ports.append(device)
                continue
            ports.append(device)
        self.port_box.addItems(ports)
        if current in ports:
            self.port_box.setCurrentText(current)
        else:
            usb_ports = [p.device for p in infos
                         if p.device in ports and
                         (p.vid is not None or "USB" in (p.description or "").upper())]
            if usb_ports:
                self.port_box.setCurrentText(usb_ports[0])

    def toggle_connection(self) -> None:
        if self.receiver is not None:
            self.receiver.stop()
            if self.receiver.is_alive():
                self.stream_label.setText("正在释放串口，请稍后再连接")
                return
            self.receiver = None
            self.connect_button.setText("连接")
            self.stream_label.setText("已断开")
            return
        if not SERIAL_OK:
            self.stream_label.setText("缺少 pyserial")
            return
        port = self.port_box.currentText().strip()
        if not port:
            self.stream_label.setText("没有可用串口")
            return
        self.receiver = SerialReceiver(port, int(self.baud_box.currentText()))
        self.overview.clear_trail()
        self.receiver.start()
        self._empty_calibration_frames = 0
        self._restore_attempted = False
        self._suppress_auto_restore = False
        self._stale_visuals_cleared = False
        self.connect_button.setText("断开")
        self.stream_label.setText(f"正在打开 {port}")
        self.statusBar().showMessage("正在打开串口")

    def send_parameters(self, values) -> None:
        if self.receiver is None:
            self.statusBar().showMessage("尚未连接，参数未发送")
            return
        if isinstance(values, RuntimeValues):
            payload = struct.pack("<BBBBffff", int(values.centroid_filter),
                                  int(values.position_filter), 0, 0,
                                  values.centroid_threshold, values.position_outlier_mm,
                                  values.cross_min_conf, values.position_jump_mm)
            body = bytes((CMD_VERSION, 1)) + payload
        elif values[0] == "calib_capture":
            body = bytes((CMD_VERSION, 2)) + struct.pack("<ffB", values[1], values[2], values[3])
        elif values[0] == "calib_clear":
            self._suppress_auto_restore = True
            body = bytes((CMD_VERSION, 3))
        elif values[0] == "calib_apply":
            body = bytes((CMD_VERSION, 4))
        elif values[0] == "calib_restore":
            record = values[1]
            bodies = [bytes((CMD_VERSION, 5))]
            for edge_index, item in enumerate(record["edges"]):
                count = int(item["count"])
                raw = tuple(float(v) for v in item["raw"])
                mm = tuple(float(v) for v in item["mm"])
                payload = struct.pack(f"<BB{count}f{count}f", edge_index, count, *raw, *mm)
                bodies.append(bytes((CMD_VERSION, 6)) + payload)
            bodies.append(bytes((CMD_VERSION, 7)))
            for restore_body in bodies:
                frame = (b"\x55\xaa" + restore_body +
                         struct.pack("<H", crc16_ccitt(restore_body)) + FOOTER)
                self.receiver.send(frame)
            self.statusBar().showMessage("已下发本机安全备份；等待固件检查、应用并写回Flash")
            return
        else:
            return
        frame = b"\x55\xaa" + body + struct.pack("<H", crc16_ccitt(body)) + FOOTER
        self.receiver.send(frame)

    def toggle_recording(self) -> None:
        self.recording = not self.recording
        self.record_button.setText("停止记录" if self.recording else "开始记录")
        if self.recording:
            self.record_rows.clear()

    def save_recording(self) -> None:
        if not self.record_rows:
            self.statusBar().showMessage("没有可保存的固件输出记录")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存固件输出", "firmware_output.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write("host_time,sequence,tick_ms,x,y,valid,cross_x,cross_y,cross_valid\n")
            for row in self.record_rows:
                f.write(",".join(map(str, row)) + "\n")
        self.statusBar().showMessage(f"已保存 {len(self.record_rows)} 帧；内容均为固件输出")

    def save_snapshot(self) -> None:
        s = self.latest_snapshot
        if s is None:
            self.statusBar().showMessage("尚未收到诊断帧")
            return
        path, _ = QFileDialog.getSaveFileName(self, "保存当前固件帧", "firmware_snapshot.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", encoding="utf-8", newline="") as f:
            f.write("edge,index,processed\n")
            for edge in EDGE_ORDER:
                processed = s.processed.get(edge)
                count = len(processed) if processed is not None else 0
                for i in range(count):
                    pv = int(processed[i]) if processed is not None and i < len(processed) else ""
                    f.write(f"{edge},{i},{pv}\n")
        self.statusBar().showMessage("已保存固件原样数据；未做任何重算")

    def refresh_view(self) -> None:
        if self.receiver is None:
            return
        (snapshot, fps, byte_rate, frames, errors, dropped, bytes_total,
         error, port_opened, wait_seconds, silence_seconds,
         frame_silence_seconds) = self.receiver.snapshot()
        for point in self.receiver.take_output_points():
            self.overview._append_output(*point)
        self.rate_label.setText(f"{fps:.1f} FPS　{byte_rate / 1024.0:.1f} KiB/s")
        self.integrity_label.setText(f"帧 {frames}　校验/格式错误 {errors}　丢序 {dropped}")
        if error:
            self.stream_label.setText("串口错误: " + error)
            self.statusBar().showMessage("连接已失效；重新插拔后点击连接重试")
            if not self.receiver.is_alive():
                self.receiver = None
                self.connect_button.setText("连接")
            return
        elif snapshot is None:
            if not port_opened:
                self.stream_label.setText(f"正在打开 {self.receiver.port}")
                self.statusBar().showMessage("正在打开串口")
            elif bytes_total == 0:
                if wait_seconds < 2.0:
                    self.stream_label.setText(
                        f"串口已打开 {self.receiver.port}，等待固件首帧")
                else:
                    self.stream_label.setText(
                        f"串口已打开 {self.receiver.port}，但 {wait_seconds:.0f} 秒未收到任何字节")
                    self.statusBar().showMessage(
                        "端口已连接；主控没有UART输出，请检查主控运行、复位和TX接线")
            else:
                self.stream_label.setText(
                    f"已收到 {bytes_total} 字节，但尚未解析出有效固件帧")
                self.statusBar().showMessage(
                    "串口有数据但帧不匹配，请检查波特率和主控固件版本")
        elif silence_seconds >= 1.5:
            self.stream_label.setText(
                f"数据已中断 {silence_seconds:.1f} 秒；画面已清空（最后有效帧 {frames}）")
            self.statusBar().showMessage(
                "端口仍打开但主控UART已断流，请复位主控；这不是暂停画面")
            if not self._stale_visuals_cleared:
                empty = Snapshot(
                    time.time(), 0, 0, FLAG_PROCESSED | FLAG_CENTROIDS |
                    FLAG_POSITION | FLAG_POSITION_DEBUG,
                    {}, {}, {}, {}, (0.0, 0.0, False), None, {}, None,
                    snapshot.params, 0, 0)
                self.overview.update_snapshot(empty)
                self.waveforms.update_snapshot(empty)
                self.diagnostics.update_snapshot(empty)
                self._stale_visuals_cleared = True
            return
        if snapshot is None or snapshot.received_at == self.last_received_at:
            return
        self._stale_visuals_cleared = False
        self.last_received_at = snapshot.received_at
        self.latest_snapshot = snapshot
        self._handle_calibration_persistence(snapshot.params)
        if snapshot.flags & (FLAG_BEFORE | FLAG_PROCESSED):
            received_edges = [EDGE_LABELS[e] for i, e in enumerate(EDGE_ORDER)
                              if snapshot.edge_seen_mask & (1 << i)]
            missing_edges = [EDGE_LABELS[e] for i, e in enumerate(EDGE_ORDER)
                             if not (snapshot.edge_seen_mask & (1 << i))]
            self.stream_label.setText(
                f"诊断流 v{DIAG_VERSION}　序号 {snapshot.sequence}　固件时钟 {snapshot.tick_ms} ms　"
                f"帧长 {snapshot.frame_bytes} B　CAN本轮 {len(received_edges)}/8　"
                f"缺失：{'、'.join(missing_edges) if missing_edges else '无'}"
            )
        else:
            self.stream_label.setText("产品坐标流（未提供波形/质心/连线数据）")
        self.statusBar().showMessage("正在接收固件数据")
        self.overview.update_snapshot(snapshot)
        self.waveforms.update_snapshot(snapshot)
        self.diagnostics.update_snapshot(snapshot)
        self.calibration.update_snapshot(snapshot)
        self.parameters.update_snapshot(snapshot)
        x, y, valid = snapshot.position
        if self.recording:
            dbg = snapshot.debug
            self.record_rows.append((snapshot.received_at, snapshot.sequence, snapshot.tick_ms,
                                     x, y, int(valid),
                                     dbg.x_cross_out if dbg else "",
                                     dbg.y_cross_out if dbg else "",
                                     int(dbg.cross_out_valid) if dbg else 0))
        if self.mouse_box.isChecked() and valid and MOUSE_OK:
            sw, sh = pyautogui.size()
            pyautogui.moveTo(max(0, min(sw - 1, x / SCREEN_W * sw)),
                             max(0, min(sh - 1, (1.0 - y / SCREEN_H) * sh)),
                             duration=0)

    def closeEvent(self, event) -> None:
        if self.receiver is not None:
            self.receiver.stop()
            self.receiver = None
        event.accept()


def main() -> None:
    initial_port = sys.argv[1] if len(sys.argv) > 1 else None
    pg.setConfigOptions(antialias=True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(
        "QWidget{font-size:12px;} QGroupBox{font-weight:600;margin-top:8px;} "
        "QGroupBox::title{subcontrol-origin:margin;left:8px;padding:0 4px;}"
    )
    window = UnifiedWindow(initial_port)
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
