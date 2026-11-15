#!/usr/bin/env python3
"""Приёмник для скетча usb_mic.ino: читает сэмплы из USB-Serial и выводит звук.

Примеры:
  python usb_mic.py --list-devices            # список звуковых устройств
  python usb_mic.py                           # играть в колонки по умолчанию
  python usb_mic.py --device "CABLE Input"    # в VB-Cable -> виртуальный микрофон
  python usb_mic.py --no-play --record a.wav  # только записать в файл
"""

import argparse
import collections
import shutil
import sys
import threading
import time
import wave

import numpy as np
import serial
import serial.tools.list_ports

BAUD = 1_000_000  # для Nano/Uno; для Leonardo значения не имеет
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1B4F, 0x239A, 0x1A86, 0x0403, 0x10C4}


class Decoder:
    """Разбирает поток пар байт [1000 0hhh][0lll llll] в 10-битные сэмплы."""

    def __init__(self):
        self.rest = b""

    def feed(self, data: bytes) -> np.ndarray:
        buf = self.rest + data
        if not buf:
            return np.empty(0, np.int16)
        a = np.frombuffer(buf, np.uint8)
        hi = a >= 0x80
        idx = np.nonzero(hi[:-1] & ~hi[1:])[0]
        vals = ((a[idx].astype(np.int16) & 0x07) << 7) | a[idx + 1]
        self.rest = buf[-1:] if hi[-1] else b""
        return vals


class Processor:
    """Убирает постоянную составляющую, усиливает, режет шум."""

    def __init__(self, gain: float, gate: float):
        self.gain = gain
        self.gate = gate
        self.dc = None
        self.gate_open = 0.0

    def process(self, raw: np.ndarray) -> np.ndarray:
        x = raw.astype(np.float32)
        m = float(x.mean())
        self.dc = m if self.dc is None else self.dc + 0.05 * (m - self.dc)
        y = (x - self.dc) / 512.0 * self.gain
        if self.gate > 0:
            rms = float(np.sqrt(np.mean(y * y)))
            target = 1.0 if rms > self.gate else 0.0
            self.gate_open += 0.3 * (target - self.gate_open)
            y *= self.gate_open
        return np.clip(y, -1.0, 1.0)


class Player:
    """Буфер между Arduino и звуковой картой. Их часы немного расходятся,
    поэтому при переполнении выкидываем старое, при нехватке — тишина."""

    def __init__(self, rate: int, device, max_latency: float):
        import sounddevice as sd

        self.max_frames = int(rate * max_latency)
        self.chunks = collections.deque()
        self.frames = 0
        self.lock = threading.Lock()
        self.stream = sd.OutputStream(
            samplerate=rate, channels=1, dtype="float32",
            device=device, callback=self._callback, blocksize=256,
        )
        self.stream.start()

    def push(self, y: np.ndarray):
        with self.lock:
            self.chunks.append(y)
            self.frames += len(y)
            while self.frames > self.max_frames and len(self.chunks) > 1:
                self.frames -= len(self.chunks.popleft())

    def _callback(self, outdata, frames, _time, _status):
        out = outdata[:, 0]
        pos = 0
        with self.lock:
            while pos < frames and self.chunks:
                c = self.chunks[0]
                take = min(len(c), frames - pos)
                out[pos:pos + take] = c[:take]
                pos += take
                self.frames -= take
                if take == len(c):
                    self.chunks.popleft()
                else:
                    self.chunks[0] = c[take:]
        out[pos:] = 0.0

    def close(self):
        self.stream.stop()
        self.stream.close()


def find_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.vid in ARDUINO_VIDS or "arduino" in (p.description or "").lower():
            return p.device
    if len(ports) == 1:
        return ports[0].device
    names = ", ".join(p.device for p in ports) or "нет портов"
    sys.exit(f"Не нашёл Arduino автоматически ({names}). Укажи --port.")


def meter(raw: np.ndarray, y: np.ndarray, dc: float, in_rate: float) -> str:
    rms = float(np.sqrt(np.mean(y * y))) if len(y) else 0.0
    db = 20 * np.log10(max(rms, 1e-5))
    bar = "#" * int(max(0.0, min(40.0, (db + 60) * 40 / 60)))
    clip = "CLIP!" if len(y) and np.max(np.abs(y)) >= 0.999 else ""
    if dc < 50 or dc > 973:
        warn = "у края шкалы: проводка / крути потенциометр"
    elif in_rate < 14000:
        warn = "теряются сэмплы"
    else:
        warn = ""
    line = (f"DC {dc:6.1f}/1023  размах {int(raw.max() - raw.min()) if len(raw) else 0:4d}"
            f"  {in_rate / 1000:4.1f} кГц  {db:6.1f} dB |{bar:<40}| {clip:5} {warn}")
    # Дополняем пробелами до ширины окна (но не больше), чтобы не оставались
    # хвосты от прошлой строки и строка не переносилась.
    width = shutil.get_terminal_size((100, 20)).columns - 1
    return "\r" + line[:width].ljust(width)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="COM-порт Arduino (по умолчанию ищется сам)")
    ap.add_argument("--rate", type=int, default=16000,
                    help="частота дискретизации, как SAMPLE_RATE в скетче")
    ap.add_argument("--device", help="устройство вывода: номер или часть имени")
    ap.add_argument("--gain", type=float, default=20.0, help="усиление (по умолч. 20)")
    ap.add_argument("--gate", type=float, default=0.0,
                    help="шумоподавитель: порог RMS 0..1, например 0.02 (0 = выкл)")
    ap.add_argument("--latency", type=float, default=0.15,
                    help="максимальная задержка буфера, с")
    ap.add_argument("--record", metavar="FILE.wav", help="также писать в WAV")
    ap.add_argument("--no-play", action="store_true", help="не выводить звук")
    ap.add_argument("--list-devices", action="store_true",
                    help="показать звуковые устройства и выйти")
    args = ap.parse_args()

    if args.list_devices:
        import sounddevice as sd
        print(sd.query_devices())
        return

    port = args.port or find_port()
    ser = serial.Serial(port, BAUD, timeout=0.05)
    ser.dtr = True  # Leonardo начинает передачу, только когда порт открыт
    print(f"Порт {port}. Ctrl+C — выход.")

    dec = Decoder()
    proc = Processor(args.gain, args.gate)
    player = None if args.no_play else Player(args.rate, args.device, args.latency)
    wav = None
    if args.record:
        wav = wave.open(args.record, "wb")
        wav.setnchannels(1)
        wav.setsampwidth(2)
        wav.setframerate(args.rate)

    last_print = time.monotonic()
    count = 0
    raw_acc, y_acc = [], []
    try:
        while True:
            data = ser.read(max(ser.in_waiting, 64))
            raw = dec.feed(data)
            if not len(raw):
                continue
            y = proc.process(raw)
            if player:
                player.push(y)
            if wav:
                wav.writeframes((y * 32767).astype("<i2").tobytes())
            count += len(raw)
            raw_acc.append(raw)
            y_acc.append(y)
            now = time.monotonic()
            if now - last_print >= 0.1:
                r, yy = np.concatenate(raw_acc), np.concatenate(y_acc)
                sys.stdout.write(meter(r, yy, proc.dc, count / (now - last_print)))
                sys.stdout.flush()
                last_print, count = now, 0
                raw_acc, y_acc = [], []
    except KeyboardInterrupt:
        print()
    finally:
        ser.close()
        if player:
            player.close()
        if wav:
            wav.close()
            print(f"Записано в {args.record}")


if __name__ == "__main__":
    main()
