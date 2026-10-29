"""
Live viewer for the STM32F411 THD Analyzer.

Install once:
    pip install pyserial numpy matplotlib sounddevice

Find the COM port: Device Manager -> Ports (COM & LPT) -> "USB Serial Device (COMx)"

Run:
    python spectrum_plot.py COM5                      # board's own PWM/ADC (main demo)
    (on the board: short-press KEY = next load profile, long-press KEY = FFT <-> DFT)
    python spectrum_plot.py COM5 --tone sine 200      # clean test tone  -> THD ~0 %
    python spectrum_plot.py COM5 --tone square 200    # distorted tone   -> THD ~45 %
    python spectrum_plot.py COM5 --mic                # laptop microphone
    python spectrum_plot.py COM5 --wav Moonlight_accompaniment.wav       # 16-bit PCM WAV (played + streamed)
"""
import argparse
import threading
import time
import wave

import numpy as np
import serial
import matplotlib.pyplot as plt

N_FFT = 1024
BINS = N_FFT // 2
AUDIO_FS = 16000          # must match AUDIO_FS_HZ in firmware
CPU_MHZ = 96
THD_LIMIT = 60.0          # must match THD_LIMIT_PCT in firmware
HDR = 9                   # header fields before the magnitudes
FRAME_LEN = HDR + BINS + N_FFT   # header + 512 magnitudes + 1024 raw samples
ADC_VREF = 3.3
ADC_MAX = 4095

ap = argparse.ArgumentParser()
ap.add_argument("port")
ap.add_argument("--mic", action="store_true", help="stream laptop microphone")
ap.add_argument("--wav", help="16-bit PCM WAV file to play and stream")
ap.add_argument("--tone", nargs=2, metavar=("SHAPE", "FREQ"),
                help="stream a synthetic test tone: sine|square FREQ_HZ")
args = ap.parse_args()

ser = serial.Serial(args.port, 115200, timeout=1)  # baud ignored by USB CDC
stop = threading.Event()


def to_int16_bytes(x):
    return np.clip(x, -32768, 32767).astype("<i2").tobytes()


def pump(get_block):
    """Send N_FFT-sample blocks at real-time pace."""
    t0 = time.perf_counter()
    i = 0
    while not stop.is_set():
        blk = get_block(i)
        if blk is None:
            return
        ser.write(to_int16_bytes(blk))
        i += 1
        target = t0 + i * N_FFT / AUDIO_FS
        time.sleep(max(0.0, target - time.perf_counter()))


# ---------------- signal sources (A mode) ----------------
def start_tone(shape, freq):
    freq = float(freq)
    n = np.arange(N_FFT)

    def get_block(i):
        t = (i * N_FFT + n) / AUDIO_FS
        s = np.sin(2 * np.pi * freq * t)
        if shape == "square":
            s = np.sign(s)
        return 8000 * s
    threading.Thread(target=pump, args=(get_block,), daemon=True).start()


def start_mic():
    import sounddevice as sd

    def cb(indata, frames, t, status):
        x = indata[:, 0].astype(np.float32)
        x = x[: len(x) // 3 * 3].reshape(-1, 3).mean(axis=1)   # 48k -> 16k
        ser.write(to_int16_bytes(x * 4))
    stream = sd.InputStream(samplerate=48000, channels=1, dtype="int16",
                            blocksize=3072, callback=cb)
    stream.start()
    return stream


def start_wav(path):
    import sounddevice as sd
    with wave.open(path, "rb") as w:
        if w.getsampwidth() != 2:
            raise SystemExit("WAV must be 16-bit PCM")
        fs, ch = w.getframerate(), w.getnchannels()
        raw = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
    data = raw.reshape(-1, ch).astype(np.float32)
    sd.play(data / 32768.0, fs)
    mono = data.mean(axis=1)
    t_old = np.arange(len(mono)) / fs
    res = np.interp(np.arange(0, t_old[-1], 1 / AUDIO_FS), t_old, mono)

    def get_block(i):
        s = i * N_FFT
        return res[s:s + N_FFT] if s + N_FFT <= len(res) else None
    threading.Thread(target=pump, args=(get_block,), daemon=True).start()


mic_stream = None
if args.tone:
    start_tone(args.tone[0].lower(), args.tone[1])
elif args.mic:
    mic_stream = start_mic()
elif args.wav:
    start_wav(args.wav)

# ---------------- plot ----------------
plt.ion()
fig, (ax_t, ax) = plt.subplots(2, 1, figsize=(11, 8),
                               gridspec_kw={"height_ratios": [1, 1.3]})

# top: original signal (time domain)
wave_line, = ax_t.plot([], [], lw=1.2, color="tab:green")
ax_t.set_title("1. Original signal - samples taken by the ADC (time domain)", fontsize=10)
ax_t.set_xlabel("Time (ms)")
ax_t.set_ylabel("Voltage at PA1 (V)")
ax_t.grid(True, alpha=0.3)

# bottom: spectrum (frequency domain)
line, = ax.plot(np.zeros(BINS), np.full(BINS, -80.0), lw=1.2)
harm_lines = [ax.axvline(0, color="tab:orange", alpha=0.35, lw=1, ls="--")
              for _ in range(9)]
ax.set_ylim(-80, 5)
ax.set_title("2. After FFT on the STM32 - spectrum (frequency domain)", fontsize=10)
ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("Magnitude (dB, relative to peak)")
ax.grid(True, alpha=0.3)

sup = fig.suptitle("Waiting for data from the board...", fontsize=12)
fig.tight_layout(rect=(0, 0, 1, 0.95))
plt.show(block=False)
plt.pause(0.1)
cur_fs = None
ser.timeout = 0.05
rx = bytearray()


def read_latest_frame():
    """Read everything that has arrived (in big chunks, not byte by byte)
    and return only the newest complete 'F,...' line; older ones are dropped
    so the plot never lags behind the board."""
    global rx
    n = ser.in_waiting
    chunk = ser.read(n if n else 1)
    if chunk:
        rx += chunk
    if b"\n" not in rx:
        if len(rx) > 200000:          # garbage with no line end: start over
            rx.clear()
        return None
    *lines, rest = rx.split(b"\n")
    rx = bytearray(rest)
    for ln in reversed(lines):
        if ln.startswith(b"F,"):
            return ln.decode(errors="ignore").strip()
    return None


last_ok = time.time()
last_warn = 0.0
last_draw = 0.0
shown_raw = False

while plt.fignum_exists(fig.number):
    raw = read_latest_frame() or ""
    parts = raw.split(",") if raw else []
    if len(parts) != FRAME_LEN:
        # keep the window alive and report what (if anything) is arriving
        if raw and not shown_raw:
            print(f"Unexpected line ({len(parts)} fields, expected {FRAME_LEN}): {raw[:120]}")
            shown_raw = True
        now = time.time()
        if now - last_ok > 3 and now - last_warn > 3:
            print(f"No valid frames for {now - last_ok:.0f} s "
                  f"(bytes waiting: {ser.in_waiting})")
            last_warn = now
        plt.pause(0.01)
        continue
    last_ok = time.time()
    if last_ok - last_draw < 0.12:          # redraw at most ~8x per second,
        plt.pause(0.005)                    # but keep the window responsive
        continue
    last_draw = last_ok
    try:
        src = parts[1]
        fs = int(parts[2])
        set_hz = int(parts[3])
        duty = int(parts[4])
        f0 = int(parts[5]) / 10.0
        thd = int(parts[6]) / 10.0
        cycles = int(parts[7])
        engine = "FFT" if parts[8] == "1" else "DFT"
        mag = np.array(parts[HDR:HDR + BINS], dtype=float)
        samples = np.array(parts[HDR + BINS:], dtype=float)
    except ValueError:
        continue

    if fs != cur_fs:
        cur_fs = fs
        line.set_xdata(np.arange(BINS) * fs / N_FFT)
        ax.set_xlim(0, fs / 2)

    # ---- top: original signal, about 4 cycles of the measured fundamental
    t_ms = np.arange(N_FFT) * 1000.0 / fs
    span = t_ms[-1] if f0 <= 0 else min(t_ms[-1], 4 * 1000.0 / f0)
    if src == "P":
        y = samples * ADC_VREF / ADC_MAX
        ax_t.set_title("1. Original signal - TIM3 PWM sampled by the ADC on PA1 (time domain)", fontsize=10)
        ax_t.set_ylabel("Voltage at PA1 (V)")
        ax_t.set_ylim(-0.2, ADC_VREF + 0.3)
    else:
        y = samples
        ax_t.set_title("1. Original signal - samples streamed from the laptop over USB (time domain)", fontsize=10)
        ax_t.set_ylabel("Sample value (int16)")
        lim = max(np.abs(y).max() * 1.15, 100)
        ax_t.set_ylim(-lim, lim)
    wave_line.set_data(t_ms, y)
    wave_line.set_marker("." if span * fs / 1000 < 150 else "")
    wave_line.set_markersize(4)
    ax_t.set_xlim(0, span)

    # ---- bottom: spectrum
    db = 20 * np.log10(np.maximum(mag, 1.0) / max(mag.max(), 1.0))
    line.set_ydata(db)
    for h, hl in enumerate(harm_lines, start=2):          # mark harmonics 2..10
        hl.set_xdata([h * f0, h * f0])

    # THD needs at least the 2nd harmonic below Nyquist to mean anything
    df = fs / N_FFT
    thd_valid = 2 * f0 < fs / 2 - 3 * df
    alarm = thd_valid and thd > THD_LIMIT
    thd_txt = f"THD {thd:.1f} %" if thd_valid else "THD n/a (no harmonics below Nyquist)"
    ms = cycles / CPU_MHZ / 1000.0
    if src == "P":
        head = f"[BOARD PWM/ADC] Set {set_hz} Hz, {duty}% duty"
    else:
        head = "[USB STREAM]"
    sup.set_text(f"{head}  |  Fundamental {f0:.1f} Hz  |  {thd_txt}"
                 f"{'  ** ALARM **' if alarm else ''}  |  {engine}: {ms:.2f} ms")
    sup.set_color("tab:red" if alarm else "black")
    fig.canvas.draw_idle()
    plt.pause(0.001)


stop.set()
if mic_stream is not None:
    mic_stream.stop()
ser.close()