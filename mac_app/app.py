from __future__ import annotations

import asyncio
import queue
import threading
from concurrent.futures import Future
from dataclasses import dataclass
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from typing import Any

from bleak import BleakClient, BleakScanner
import serial
from serial.tools import list_ports


DEVICE_NAME = "Greenhouse-ESP32"
SENSOR_SERVICE_UUID = "7d9e0001-dc6d-4f04-9fb8-11f8d5c7a001"
SENSOR_WRITE_UUID = "7d9e0002-dc6d-4f04-9fb8-11f8d5c7a001"
SERIAL_BAUD_RATE = 115200
SERIAL_READ_TIMEOUT_SECONDS = 0.25
SCAN_TIMEOUT_SECONDS = 12.0
RECONNECT_DELAY_SECONDS = 2.0

BACKGROUND = "#08111f"
PANEL = "#101b2d"
CARD = "#15243a"
CARD_BORDER = "#263a55"
TEXT = "#f3f7fb"
MUTED = "#8fa2b8"
TEAL = "#35d0ba"
BLUE = "#54a8ff"
GREEN = "#6fd08c"
YELLOW = "#f6c85f"
ORANGE = "#f19a55"
PURPLE = "#b594f6"
RED = "#ff6b75"

@dataclass(frozen=True)
class SensorReading:
    temperature_1_f: float
    temperature_2_f: float
    humidity_1_pct: float
    humidity_2_pct: float
    occupancy: int
    wind_speed_m_s: float
    fan_rpm: int
    water_level_pct: float
    battery_voltage_v: float
    panel_voltage_v: float
    solar_power_w: float
    uv_index: float


class BluetoothWorker:
    """Bridge Arduino Mega serial readings to the ESP32 over BLE."""

    def __init__(self, events: queue.Queue[tuple[str, object]]) -> None:
        self.events = events
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        self.stop_event: asyncio.Event | None = None
        self.disconnect_event: asyncio.Event | None = None
        self.future: Future[None] | None = None
        self.client: BleakClient | None = None
        self.serial_connection: serial.Serial | None = None
        self.stop_requested = threading.Event()
        self.packet_sequence = 0

    def _run_loop(self) -> None:
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def start(self) -> None:
        if self.future is not None and not self.future.done():
            return

        self.stop_requested.clear()
        self.future = asyncio.run_coroutine_threadsafe(
            self._run_bridge(),
            self.loop,
        )

    def stop(self) -> None:
        self.stop_requested.set()
        self.loop.call_soon_threadsafe(self._signal_stop)

    def _signal_stop(self) -> None:
        if self.stop_event is not None:
            self.stop_event.set()
        if self.disconnect_event is not None:
            self.disconnect_event.set()

    def _signal_disconnect(self) -> None:
        if self.disconnect_event is not None:
            self.disconnect_event.set()

    def _on_disconnected(self, client: BleakClient) -> None:
        if client is not self.client or self.stop_requested.is_set():
            return

        # Wake the bridge without ending the worker so it can reconnect.
        self.loop.call_soon_threadsafe(self._signal_disconnect)

    @staticmethod
    def _parse_mega_line(line: str) -> SensorReading | None:
        parts = [part.strip() for part in line.split(",")]
        if len(parts) != 13 or parts[0].upper() != "DATA":
            return None
        try:
            return SensorReading(
                temperature_1_f=float(parts[1]),
                temperature_2_f=float(parts[2]),
                humidity_1_pct=float(parts[3]),
                humidity_2_pct=float(parts[4]),
                occupancy=int(float(parts[5])),
                wind_speed_m_s=float(parts[6]),
                fan_rpm=int(float(parts[7])),
                water_level_pct=float(parts[8]),
                battery_voltage_v=float(parts[9]),
                panel_voltage_v=float(parts[10]),
                solar_power_w=float(parts[11]),
                uv_index=float(parts[12]),
            )
        except ValueError:
            return None

    @staticmethod
    def _serial_port_score(port: Any) -> int:
        details = " ".join(
            str(value or "")
            for value in (
                port.device,
                port.description,
                port.manufacturer,
                port.hwid,
            )
        ).lower()
        score = 0
        if "arduino" in details:
            score += 100
        if "mega" in details:
            score += 80
        if getattr(port, "vid", None) in {0x2341, 0x2A03}:
            score += 80
        if "usbmodem" in details:
            score += 30
        if "usbserial" in details:
            score += 15
        if "bluetooth" in details:
            score -= 200
        if "cp210" in details or "silicon labs" in details:
            score -= 20
        return score

    def _open_mega_serial(self) -> None:
        if self.serial_connection is not None:
            return

        candidates = [
            port
            for port in list_ports.comports()
            if port.device.startswith(("/dev/cu.", "/dev/tty."))
            and "bluetooth" not in port.device.lower()
        ]
        if not candidates:
            raise RuntimeError("Arduino Mega USB serial port was not found")

        port = max(candidates, key=self._serial_port_score)
        self.serial_connection = serial.Serial(
            port.device,
            SERIAL_BAUD_RATE,
            timeout=SERIAL_READ_TIMEOUT_SECONDS,
        )
        self.serial_connection.reset_input_buffer()
        self.events.put(("stage", ("mega", "connected")))
        self.events.put(("stage", ("interpreter", "connecting")))
        self.events.put(("status", f"Arduino Mega connected on {port.device}"))

    def _close_mega_serial(self) -> None:
        connection = self.serial_connection
        self.serial_connection = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def _packet_chunks(self, reading: SensorReading) -> list[str]:
        self.packet_sequence = (self.packet_sequence % 999999) + 1
        sequence = self.packet_sequence
        values = (
            ("T1", f"{reading.temperature_1_f:.1f}"),
            ("T2", f"{reading.temperature_2_f:.1f}"),
            ("H1", f"{reading.humidity_1_pct:.1f}"),
            ("H2", f"{reading.humidity_2_pct:.1f}"),
            ("OCC", str(reading.occupancy)),
            ("WIND", f"{reading.wind_speed_m_s:.1f}"),
            ("RPM", str(reading.fan_rpm)),
            ("WATER", f"{reading.water_level_pct:.1f}"),
            ("BAT", f"{reading.battery_voltage_v:.1f}"),
            ("PANEL", f"{reading.panel_voltage_v:.1f}"),
            ("SOLAR", f"{reading.solar_power_w:.0f}"),
            ("UV", f"{reading.uv_index:.1f}"),
        )
        return [
            f"SEQ,{sequence}",
            *(f"{key},{value}" for key, value in values),
            f"END,{sequence}",
        ]

    async def _send_reading(self, reading: SensorReading) -> None:
        if self.client is None or not self.client.is_connected:
            raise ConnectionError("ESP32 Bluetooth link was lost")

        for chunk in self._packet_chunks(reading):
            payload = chunk.encode("utf-8")
            if len(payload) > 20:
                raise ValueError(f"BLE packet is too long: {chunk}")
            if self.disconnect_event is not None and self.disconnect_event.is_set():
                raise ConnectionError("ESP32 Bluetooth link was lost")
            await self.client.write_gatt_char(
                SENSOR_WRITE_UUID,
                payload,
                response=True,
            )

    async def _bridge_readings(self) -> None:
        while (
            self.stop_event is not None
            and not self.stop_event.is_set()
            and self.disconnect_event is not None
            and not self.disconnect_event.is_set()
        ):
            if self.serial_connection is None:
                raise serial.SerialException("Arduino Mega serial port closed")

            raw_line = await asyncio.to_thread(self.serial_connection.readline)
            if not raw_line:
                continue

            line = raw_line.decode("utf-8", errors="replace").strip()
            if line.startswith("ERROR,"):
                self.events.put(("sensor_error", line.split(",", 1)[1]))
                continue

            reading = self._parse_mega_line(line)
            if reading is None:
                continue

            await self._send_reading(reading)
            self.events.put(("reading", reading))

    async def _wait_before_retry(self) -> None:
        if self.stop_event is None:
            return

        try:
            await asyncio.wait_for(
                self.stop_event.wait(),
                timeout=RECONNECT_DELAY_SECONDS,
            )
        except asyncio.TimeoutError:
            pass

    async def _disconnect_client(self) -> None:
        # Clear first so an expected callback is not treated as a new failure.
        client = self.client
        self.client = None
        self.disconnect_event = None

        if client is None or not client.is_connected:
            return

        try:
            await client.disconnect()
        except Exception:
            pass

    async def _scan_for_device(
        self,
        timeout: float,
        service_uuids: list[str] | None = None,
    ) -> Any | None:
        if self.stop_event is None:
            return None

        scan_task = asyncio.create_task(
            BleakScanner.find_device_by_name(
                DEVICE_NAME,
                timeout=timeout,
                service_uuids=service_uuids,
            )
        )
        stop_task = asyncio.create_task(self.stop_event.wait())
        try:
            done, _pending = await asyncio.wait(
                {scan_task, stop_task},
                return_when=asyncio.FIRST_COMPLETED,
            )
            if stop_task in done and self.stop_event.is_set():
                scan_task.cancel()
                await asyncio.gather(scan_task, return_exceptions=True)
                return None
            return await scan_task
        finally:
            if not stop_task.done():
                stop_task.cancel()
            await asyncio.gather(stop_task, return_exceptions=True)

    async def _find_device(self) -> Any | None:
        device = await self._scan_for_device(
            SCAN_TIMEOUT_SECONDS,
            [SENSOR_SERVICE_UUID],
        )
        if device is not None:
            return device
        if self.stop_event is None or self.stop_event.is_set():
            return None

        # Some ESP32 firmware advertises the name but omits the service UUID
        # after restarting its advertising. A short name-only fallback handles
        # that case on macOS.
        return await self._scan_for_device(5.0)

    async def _run_bridge(self) -> None:
        self.stop_event = asyncio.Event()
        if self.stop_requested.is_set():
            self.stop_event.set()
        reconnect_attempt = 0

        try:
            while not self.stop_event.is_set():
                try:
                    self.disconnect_event = asyncio.Event()

                    self.events.put(("stage", ("mega", "connecting")))
                    self.events.put(("stage", ("bluetooth", "connecting")))
                    self.events.put(("stage", ("interpreter", "off")))
                    self.events.put(("stage", ("esp32", "off")))
                    self._open_mega_serial()

                    if reconnect_attempt == 0:
                        status = f"Searching for {DEVICE_NAME}..."
                    else:
                        status = (
                            f"Reconnecting to {DEVICE_NAME} "
                            f"(attempt {reconnect_attempt})..."
                        )
                    self.events.put(("reconnecting", status))

                    device = await self._find_device()
                    if device is None:
                        raise RuntimeError(f"Could not find {DEVICE_NAME}")

                    self.events.put(("stage", ("esp32", "connecting")))
                    self.events.put(
                        ("status", f"Connecting to {DEVICE_NAME}...")
                    )

                    self.client = BleakClient(
                        device,
                        disconnected_callback=self._on_disconnected,
                        services=[SENSOR_SERVICE_UUID],
                    )
                    await self.client.connect()
                    characteristic = self.client.services.get_characteristic(
                        SENSOR_WRITE_UUID
                    )
                    if characteristic is None:
                        raise RuntimeError(
                            "ESP32 write characteristic ...0002 was not found; "
                            "upload the current receiver firmware"
                        )

                    reconnect_attempt = 0
                    self.events.put(("stage", ("bluetooth", "connected")))
                    self.events.put(("stage", ("esp32", "connected")))
                    self.events.put(
                        ("running", "Connected — forwarding Mega readings")
                    )

                    await self._bridge_readings()
                    if self.stop_event.is_set():
                        break

                    reconnect_attempt = 1
                    self.events.put(
                        (
                            "reconnecting",
                            "Bluetooth link dropped — reconnecting automatically...",
                        )
                    )

                except asyncio.CancelledError:
                    raise
                except Exception as error:
                    if self.stop_event.is_set():
                        break
                    if isinstance(error, serial.SerialException):
                        self._close_mega_serial()
                    reconnect_attempt += 1
                    self.events.put(
                        (
                            "reconnecting",
                            f"Connection unavailable ({error}). Retrying...",
                        )
                    )
                finally:
                    await self._disconnect_client()

                if not self.stop_event.is_set():
                    await self._wait_before_retry()

        finally:
            await self._disconnect_client()
            self._close_mega_serial()
            self.stop_event = None
            self.disconnect_event = None

            for stage in ("mega", "interpreter", "bluetooth", "esp32"):
                self.events.put(("stage", (stage, "off")))

            if self.stop_requested.is_set():
                self.events.put(("status", "Monitoring stopped"))

            self.events.put(("finished", None))

    def shutdown(self) -> None:
        self.stop()

        if self.future is not None:
            try:
                self.future.result(timeout=4)
            except Exception:
                pass

        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(timeout=2)


class MonitorWindow:
    METRICS = (
        ("temperature_1_f", "Temperature · Zone 1", "°F", ORANGE),
        ("temperature_2_f", "Temperature · Zone 2", "°F", ORANGE),
        ("humidity_1_pct", "Humidity · Zone 1", "%", BLUE),
        ("humidity_2_pct", "Humidity · Zone 2", "%", BLUE),
        ("occupancy", "Occupancy", "people", PURPLE),
        ("wind_speed_m_s", "Wind speed", "m/s", TEAL),
        ("fan_rpm", "Fan rotation", "RPM", GREEN),
        ("water_level_pct", "Water level", "%", BLUE),
        ("battery_voltage_v", "Battery voltage", "V", YELLOW),
        ("panel_voltage_v", "Panel voltage", "V", YELLOW),
        ("solar_power_w", "Solar generation", "W", ORANGE),
        ("uv_index", "UV index", "index", PURPLE),
    )

    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        self.root.title("Environmental Monitoring Dashboard")
        self.root.geometry("1220x820")
        self.root.minsize(1080, 740)
        self.root.configure(bg=BACKGROUND)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.worker = BluetoothWorker(self.events)

        self.status = tk.StringVar(value="Ready to connect")
        self.status_badge = tk.StringVar(value="OFFLINE")
        self.last_update = tk.StringVar(value="Waiting for the first reading")
        self.transfer_count = tk.StringVar(value="0 readings received")
        self.metric_values = {
            name: tk.StringVar(value="--")
            for name, _label, _unit, _color in self.METRICS
        }
        self.stage_labels: dict[str, tk.Label] = {}
        self.history_rows = 0

        self._configure_styles()
        self._build_ui()

        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.after(100, self._poll_events)

    def _configure_styles(self) -> None:
        style = ttk.Style()
        style.theme_use("clam")
        style.configure(
            "Primary.TButton",
            background=TEAL,
            foreground=BACKGROUND,
            borderwidth=0,
            padding=(16, 11),
            font=("Helvetica Neue", 12, "bold"),
        )
        style.map(
            "Primary.TButton",
            background=[("active", "#59e0cc"), ("disabled", "#315d59")],
            foreground=[("disabled", MUTED)],
        )
        style.configure(
            "Secondary.TButton",
            background=CARD,
            foreground=TEXT,
            bordercolor=CARD_BORDER,
            padding=(14, 10),
            font=("Helvetica Neue", 11, "bold"),
        )
        style.map(
            "Secondary.TButton",
            background=[("active", "#20324d"), ("disabled", PANEL)],
            foreground=[("disabled", MUTED)],
        )
        style.configure(
            "History.Treeview",
            background=PANEL,
            fieldbackground=PANEL,
            foreground=TEXT,
            rowheight=28,
            borderwidth=0,
            font=("Helvetica Neue", 10),
        )
        style.configure(
            "History.Treeview.Heading",
            background=CARD,
            foreground=MUTED,
            relief="flat",
            font=("Helvetica Neue", 10, "bold"),
        )
        style.map(
            "History.Treeview",
            background=[("selected", "#23465c")],
        )

    def _build_ui(self) -> None:
        header = tk.Frame(self.root, bg=BACKGROUND, padx=30, pady=22)
        header.pack(fill="x")

        title_group = tk.Frame(header, bg=BACKGROUND)
        title_group.pack(side="left")
        tk.Label(
            title_group,
            text="ENVIRONMENT MONITOR",
            bg=BACKGROUND,
            fg=TEXT,
            font=("Helvetica Neue", 24, "bold"),
        ).pack(anchor="w")
        tk.Label(
            title_group,
            text="DHT11 → Arduino Mega → Mac dashboard → Bluetooth → ESP32",
            bg=BACKGROUND,
            fg=MUTED,
            font=("Helvetica Neue", 11),
        ).pack(anchor="w", pady=(4, 0))

        self.badge = tk.Label(
            header,
            textvariable=self.status_badge,
            bg="#263140",
            fg=MUTED,
            padx=14,
            pady=7,
            font=("Helvetica Neue", 10, "bold"),
        )
        self.badge.pack(side="right")

        body = tk.Frame(self.root, bg=BACKGROUND, padx=30)
        body.pack(fill="both", expand=True, pady=(0, 26))
        body.columnconfigure(1, weight=1)
        body.rowconfigure(0, weight=1)

        sidebar = tk.Frame(
            body,
            bg=PANEL,
            width=270,
            padx=18,
            pady=20,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        sidebar.grid(row=0, column=0, sticky="nsew", padx=(0, 18))
        sidebar.grid_propagate(False)

        tk.Label(
            sidebar,
            text="BLUETOOTH DEVICE",
            bg=PANEL,
            fg=MUTED,
            font=("Helvetica Neue", 10, "bold"),
        ).pack(anchor="w")
        tk.Label(
            sidebar,
            text=DEVICE_NAME,
            bg=PANEL,
            fg=TEXT,
            font=("Helvetica Neue", 16, "bold"),
        ).pack(anchor="w", pady=(10, 2))
        tk.Label(
            sidebar,
            text="Mega USB + ESP32 Bluetooth",
            bg=PANEL,
            fg=TEAL,
            font=("Helvetica Neue", 10),
        ).pack(anchor="w")

        tk.Frame(sidebar, bg=CARD_BORDER, height=1).pack(
            fill="x", pady=(22, 18)
        )
        tk.Label(
            sidebar,
            text="LIVE DATA PATH",
            bg=PANEL,
            fg=MUTED,
            font=("Helvetica Neue", 10, "bold"),
        ).pack(anchor="w")

        self._add_stage(sidebar, "mega", "Arduino Mega + DHT11")
        self._add_stage(sidebar, "interpreter", "Mac data interpreter")
        self._add_stage(sidebar, "bluetooth", "Bluetooth Low Energy")
        self._add_stage(sidebar, "esp32", "ESP32 display")

        tk.Frame(sidebar, bg=CARD_BORDER, height=1).pack(
            fill="x", pady=(22, 18)
        )
        tk.Label(
            sidebar,
            text="STATUS",
            bg=PANEL,
            fg=MUTED,
            font=("Helvetica Neue", 10, "bold"),
        ).pack(anchor="w")
        tk.Label(
            sidebar,
            textvariable=self.status,
            bg=PANEL,
            fg=TEXT,
            justify="left",
            wraplength=225,
            font=("Helvetica Neue", 10),
        ).pack(anchor="w", pady=(10, 0))

        tk.Label(
            sidebar,
            text=(
                "The DHT11 temperature and humidity readings come from the "
                "Arduino Mega. The remaining representative values follow "
                "the same path and can be replaced by physical sensors later."
            ),
            bg=PANEL,
            fg=MUTED,
            justify="left",
            wraplength=225,
            font=("Helvetica Neue", 9),
        ).pack(anchor="w", pady=(22, 0))

        button_area = tk.Frame(sidebar, bg=PANEL)
        button_area.pack(side="bottom", fill="x")
        self.start_button = ttk.Button(
            button_area,
            text="Connect to ESP32",
            command=self._start,
            style="Primary.TButton",
        )
        self.start_button.pack(fill="x")
        self.stop_button = ttk.Button(
            button_area,
            text="Disconnect",
            command=self._stop,
            state="disabled",
            style="Secondary.TButton",
        )
        self.stop_button.pack(fill="x", pady=(9, 0))

        content = tk.Frame(body, bg=BACKGROUND)
        content.grid(row=0, column=1, sticky="nsew")
        content.columnconfigure((0, 1, 2, 3), weight=1)
        content.rowconfigure(1, weight=1)

        overview_header = tk.Frame(content, bg=BACKGROUND)
        overview_header.grid(row=0, column=0, columnspan=4, sticky="ew")
        tk.Label(
            overview_header,
            text="Live sensor overview",
            bg=BACKGROUND,
            fg=TEXT,
            font=("Helvetica Neue", 17, "bold"),
        ).pack(side="left")
        tk.Label(
            overview_header,
            textvariable=self.last_update,
            bg=BACKGROUND,
            fg=MUTED,
            font=("Helvetica Neue", 10),
        ).pack(side="right")

        cards = tk.Frame(content, bg=BACKGROUND)
        cards.grid(row=1, column=0, columnspan=4, sticky="nsew", pady=(12, 16))
        for column in range(4):
            cards.columnconfigure(column, weight=1)
        for row in range(3):
            cards.rowconfigure(row, weight=1)

        for index, metric in enumerate(self.METRICS):
            name, label, unit, color = metric
            self._add_metric_card(
                cards,
                row=index // 4,
                column=index % 4,
                name=name,
                label=label,
                unit=unit,
                color=color,
            )

        history_panel = tk.Frame(
            content,
            bg=PANEL,
            padx=14,
            pady=12,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        history_panel.grid(row=2, column=0, columnspan=4, sticky="ew")

        history_heading = tk.Frame(history_panel, bg=PANEL)
        history_heading.pack(fill="x", pady=(0, 8))
        tk.Label(
            history_heading,
            text="Recent readings",
            bg=PANEL,
            fg=TEXT,
            font=("Helvetica Neue", 12, "bold"),
        ).pack(side="left")
        tk.Label(
            history_heading,
            textvariable=self.transfer_count,
            bg=PANEL,
            fg=MUTED,
            font=("Helvetica Neue", 10),
        ).pack(side="right")

        columns = (
            "time",
            "temp",
            "humidity",
            "occupancy",
            "water",
            "solar",
        )
        self.history = ttk.Treeview(
            history_panel,
            columns=columns,
            show="headings",
            height=4,
            style="History.Treeview",
        )
        headings = {
            "time": "Time",
            "temp": "Zone 1 temp",
            "humidity": "Zone 1 humidity",
            "occupancy": "Occupancy",
            "water": "Water level",
            "solar": "Solar power",
        }
        widths = {
            "time": 105,
            "temp": 115,
            "humidity": 130,
            "occupancy": 100,
            "water": 110,
            "solar": 110,
        }
        for column in columns:
            self.history.heading(column, text=headings[column])
            self.history.column(
                column,
                width=widths[column],
                anchor="center",
                stretch=True,
            )
        self.history.pack(fill="x")

    def _add_stage(self, parent: tk.Widget, key: str, text: str) -> None:
        row = tk.Frame(parent, bg=PANEL)
        row.pack(fill="x", pady=(14, 0))
        dot = tk.Label(
            row,
            text="●",
            bg=PANEL,
            fg="#46556a",
            font=("Helvetica Neue", 13),
        )
        dot.pack(side="left")
        tk.Label(
            row,
            text=text,
            bg=PANEL,
            fg=TEXT,
            font=("Helvetica Neue", 10),
        ).pack(side="left", padx=(8, 0))
        self.stage_labels[key] = dot

    def _add_metric_card(
        self,
        parent: tk.Widget,
        row: int,
        column: int,
        name: str,
        label: str,
        unit: str,
        color: str,
    ) -> None:
        card = tk.Frame(
            parent,
            bg=CARD,
            padx=15,
            pady=13,
            highlightbackground=CARD_BORDER,
            highlightthickness=1,
        )
        card.grid(
            row=row,
            column=column,
            sticky="nsew",
            padx=(0, 8) if column < 3 else (0, 0),
            pady=(0, 8),
        )
        tk.Frame(card, bg=color, height=3).pack(fill="x", pady=(0, 10))
        tk.Label(
            card,
            text=label.upper(),
            bg=CARD,
            fg=MUTED,
            font=("Helvetica Neue", 9, "bold"),
        ).pack(anchor="w")

        value_row = tk.Frame(card, bg=CARD)
        value_row.pack(anchor="w", pady=(7, 0))
        tk.Label(
            value_row,
            textvariable=self.metric_values[name],
            bg=CARD,
            fg=TEXT,
            font=("Helvetica Neue", 24, "bold"),
        ).pack(side="left")
        tk.Label(
            value_row,
            text=unit,
            bg=CARD,
            fg=color,
            font=("Helvetica Neue", 10, "bold"),
        ).pack(side="left", padx=(7, 0), pady=(10, 0))

    def _start(self) -> None:
        self.history_rows = 0
        for row in self.history.get_children():
            self.history.delete(row)

        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status_badge.set("CONNECTING")
        self.badge.configure(bg="#3b321e", fg=YELLOW)
        self.worker.start()

    def _stop(self) -> None:
        self.status.set("Disconnecting...")
        self.stop_button.configure(state="disabled")
        self.worker.stop()

    def _poll_events(self) -> None:
        try:
            while True:
                event, value = self.events.get_nowait()
                self._handle_event(event, value)
        except queue.Empty:
            pass

        self.root.after(100, self._poll_events)

    def _handle_event(self, event: str, value: object) -> None:
        if event == "status":
            self.status.set(str(value))

        elif event == "running":
            self.status.set(str(value))
            self.status_badge.set("CONNECTED")
            self.badge.configure(bg="#143d35", fg=TEAL)

        elif event == "reconnecting":
            self.status.set(str(value))
            self.status_badge.set("RECONNECTING")
            self.badge.configure(bg="#3b321e", fg=YELLOW)

        elif event == "stage":
            stage, stage_status = value  # type: ignore[misc]
            colors = {
                "off": "#46556a",
                "connecting": YELLOW,
                "connected": GREEN,
                "error": RED,
            }
            self.stage_labels[str(stage)].configure(
                fg=colors.get(str(stage_status), MUTED)
            )

        elif event == "reading":
            self.stage_labels["interpreter"].configure(fg=GREEN)
            self.status.set("Mega reading forwarded to the ESP32 display")
            self.status_badge.set("LIVE")
            self.badge.configure(bg="#143d35", fg=TEAL)
            self._show_reading(value)  # type: ignore[arg-type]

        elif event == "packet_error":
            self.status.set(f"Ignored malformed packet: {value}")

        elif event == "sensor_error":
            self.status.set(f"Mega sensor error: {value}")
            self.status_badge.set("SENSOR ERROR")
            self.badge.configure(bg="#4a2028", fg=RED)

        elif event == "error":
            self.status.set(f"Error: {value}")
            self.status_badge.set("ERROR")
            self.badge.configure(bg="#4a2028", fg=RED)

        elif event == "finished":
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
            if self.status_badge.get() != "ERROR":
                self.status_badge.set("OFFLINE")
                self.badge.configure(bg="#263140", fg=MUTED)

    def _show_reading(self, reading: SensorReading) -> None:
        values: dict[str, str] = {
            "temperature_1_f": f"{reading.temperature_1_f:.1f}",
            "temperature_2_f": f"{reading.temperature_2_f:.1f}",
            "humidity_1_pct": f"{reading.humidity_1_pct:.1f}",
            "humidity_2_pct": f"{reading.humidity_2_pct:.1f}",
            "occupancy": str(reading.occupancy),
            "wind_speed_m_s": f"{reading.wind_speed_m_s:.1f}",
            "fan_rpm": f"{reading.fan_rpm:,}",
            "water_level_pct": f"{reading.water_level_pct:.1f}",
            "battery_voltage_v": f"{reading.battery_voltage_v:.1f}",
            "panel_voltage_v": f"{reading.panel_voltage_v:.1f}",
            "solar_power_w": f"{reading.solar_power_w:.0f}",
            "uv_index": f"{reading.uv_index:.1f}",
        }
        for name, display_value in values.items():
            self.metric_values[name].set(display_value)

        now = datetime.now()
        self.history_rows += 1
        self.last_update.set(f"Updated {now.strftime('%I:%M:%S %p')}")
        self.transfer_count.set(
            f"{self.history_rows} reading"
            f"{'s' if self.history_rows != 1 else ''} received"
        )

        self.history.insert(
            "",
            0,
            values=(
                now.strftime("%I:%M:%S %p"),
                f"{reading.temperature_1_f:.1f} °F",
                f"{reading.humidity_1_pct:.1f} %",
                reading.occupancy,
                f"{reading.water_level_pct:.1f} %",
                f"{reading.solar_power_w:.0f} W",
            ),
        )

        rows = self.history.get_children()
        for old_row in rows[8:]:
            self.history.delete(old_row)

    def _close(self) -> None:
        self.worker.shutdown()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    MonitorWindow(root)
    root.mainloop()


if __name__ == "__main__":
    main()
