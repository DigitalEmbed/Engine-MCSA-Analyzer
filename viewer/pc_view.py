#!/usr/bin/env python3
"""
pc_view.py — Visualizador MCSA via UART

Novidades:
- Diagnóstico com severidades [E]/[W]/[I] no CSV (sem emojis)
- No gráfico: diagnósticos coloridos (E=red, W=orange, I=green), sem ícones
- No terminal: mantém emojis (🔴/🟠/⚪)
- Eixo Y fixo em 1.2× do maior valor observado na sessão
- Persistência de contadores por sessão e por minuto; gauges THD/SNR/SLIP

Uso:
  pip install pyserial matplotlib
  python pc_view.py --port COM5 --baud 921600 --ch 0
  python pc_view.py --port /dev/cu.usbmodem12303 --baud 115200 --ch 0
"""
import argparse
import struct
import time
import csv
import os
from datetime import datetime
import serial
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.text import Text

MAGIC = 0xA55A  # little-endian on wire => 0x5A, 0xA5

# ------------------------------ Protocol ------------------------------

def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if (crc & 0x8000) != 0:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def read_exact(ser: serial.Serial, n: int) -> bytes:
    data = ser.read(n)
    if len(data) != n:
        return b""
    return data

def sync_magic(ser: serial.Serial) -> bool:
    while True:
        b = ser.read(1)
        if not b:
            return False
        if b[0] == 0x5A:
            b2 = ser.read(1)
            if not b2:
                return False
            if b2[0] == 0xA5:
                return True

def read_frame(ser: serial.Serial):
    """
    Protocolo (LE), após MAGIC (0xA55A):
      uint8  version
      uint8  ch_id
      uint32 nfft
      float  fs_hz
      int16  dc_removed
      float  f1_hz, a1, thd, snr_db, slip_est, fr_hz, a_fr, a_2fr
      uint16 n_harm
        repeat n_harm: uint16 order; float freq_hz; float amp
      uint16 n_bins
        repeat n_bins: float mag
      uint16 crc16_ccitt (sobre [version..último dado])
    """
    if not sync_magic(ser):
        return None

    header = read_exact(ser, 1 + 1 + 4 + 4 + 2)
    if not header:
        return None
    version, ch_id = struct.unpack_from('<BB', header, 0)
    nfft, fs_hz = struct.unpack_from('<If', header, 2)
    dc_removed, = struct.unpack_from('<h', header, 2 + 4 + 4)

    metrics_block = read_exact(ser, 4 * 8 + 2)
    if not metrics_block:
        return None
    f1_hz, a1, thd, snr_db, slip_est, fr_hz, a_fr, a_2fr = struct.unpack_from('<8f', metrics_block, 0)
    n_harm, = struct.unpack_from('<H', metrics_block, 32)

    harmonics = []
    for _ in range(n_harm):
        hb = read_exact(ser, 2 + 4 + 4)
        if not hb:
            return None
        order, = struct.unpack_from('<H', hb, 0)
        freq_hz, amp = struct.unpack_from('<2f', hb, 2)
        harmonics.append((order, freq_hz, amp))

    nb_raw = read_exact(ser, 2)
    if not nb_raw:
        return None
    n_bins, = struct.unpack('<H', nb_raw)

    spec_raw = read_exact(ser, 4 * n_bins)
    if not spec_raw:
        return None
    mag = list(struct.unpack('<' + 'f' * n_bins, spec_raw))

    crc_raw = read_exact(ser, 2)
    if not crc_raw:
        return None
    sent_crc, = struct.unpack('<H', crc_raw)

    # Recalcula CRC sobre [version..fim]
    data_crc = bytes([version, ch_id]) \
        + struct.pack('<I', nfft) \
        + struct.pack('<f', fs_hz) \
        + struct.pack('<h', dc_removed) \
        + struct.pack('<8f', f1_hz, a1, thd, snr_db, slip_est, fr_hz, a_fr, a_2fr) \
        + struct.pack('<H', n_harm)
    for order, freq_hz, amp in harmonics:
        data_crc += struct.pack('<Hff', order, freq_hz, amp)
    data_crc += struct.pack('<H', n_bins) + spec_raw
    calc_crc = crc16_ccitt(data_crc)
    if calc_crc != sent_crc:
        return None

    return {
        'ts': time.time(),
        'version': version,
        'ch_id': ch_id,
        'nfft': nfft,
        'fs_hz': fs_hz,
        'dc_removed': dc_removed,
        'f1_hz': f1_hz,
        'a1': a1,
        'thd': thd,
        'snr_db': snr_db,
        'slip_est': slip_est,
        'fr_hz': fr_hz,
        'a_fr': a_fr,
        'a_2fr': a_2fr,
        'harmonics': harmonics,
        'mag': mag,
    }

# ------------------------------ Diagnosis ------------------------------

def quick_diagnosis(frame: dict):
    """
    Retorna (entries, tags_dict), entries = [(sev, msg), ...] com sev ∈ {'E','W','I'}.
    """
    entries = []
    tags = {
        'bars_high': False,
        'bars_moderate': False,
        'unbalance': False,
        'misalignment': False,
        'thd_high': False,
        'thd_moderate': False,
        'thd_notice': False,
        'snr_low': False,
    }

    a1 = frame.get('a1', 0.0) or 0.0
    thd = frame.get('thd', 0.0) or 0.0
    snr_db = frame.get('snr_db', 0.0) or 0.0
    slip_est = frame.get('slip_est', -1.0)
    fr_hz = frame.get('fr_hz', 0.0) or 0.0
    a_fr = frame.get('a_fr', 0.0) or 0.0
    a_2fr = frame.get('a_2fr', 0.0) or 0.0

    fr_rel = (a1 > 0.0) and (a_fr / a1) or 0.0
    two_fr_rel = (a1 > 0.0) and (a_2fr / a1) or 0.0

    if slip_est is not None and slip_est >= 0.0:
        if slip_est > 0.06 and snr_db > 12.0:
            entries.append(('W', 'Possível defeito em barras do rotor (slip elevado > 6%).'))
            tags['bars_high'] = True
        elif slip_est > 0.03 and snr_db > 12.0:
            entries.append(('W', 'Slip moderado (> 3%). Observe sidebands de (1±2s)f1.'))
            tags['bars_moderate'] = True

    if fr_hz > 1.0 and fr_rel > 0.20 and two_fr_rel < 0.10:
        entries.append(('W', 'Desbalanceamento provável (1×fr forte, 2×fr fraco).'))
        tags['unbalance'] = True

    if fr_hz > 1.0 and two_fr_rel > 0.15:
        entries.append(('W', 'Desalinhamento/folga provável (2×fr elevado).'))
        tags['misalignment'] = True

    if thd > 0.15:
        entries.append(('E', 'THD > 15% — distorção elevada.'))
        tags['thd_high'] = True
    elif thd > 0.10:
        entries.append(('W', 'THD > 10% — distorção considerável.'))
        tags['thd_moderate'] = True
    elif thd > 0.05:
        entries.append(('W', 'THD > 5% — distorção moderada.'))
        tags['thd_notice'] = True

    if snr_db < 12.0:
        entries.append(('I', 'SNR baixo (<12 dB). Medição pouco confiável.'))
        tags['snr_low'] = True

    if not entries:
        entries.append(('I', 'Sem assinaturas fortes neste bloco.'))

    return entries, tags

# ------------------------------ Gauges ------------------------------

def clamp01(x: float) -> float:
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)

def gauge_color(value: float, kind: str) -> str:
    if kind == 'thd':
        v = clamp01(value / 0.20)
        return 'green' if v < 0.33 else ('gold' if v < 0.66 else 'red')
    if kind == 'snr':
        v = clamp01(value / 30.0)
        return 'red' if v < 0.33 else ('gold' if v < 0.66 else 'green')
    if kind == 'slip':
        v = clamp01(value / 0.10)
        return 'green' if v < 0.33 else ('gold' if v < 0.66 else 'red')
    return 'blue'

# ------------------------------ Main ------------------------------

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', '-p', required=True, help='porta serial (ex: COM5 ou /dev/ttyUSB0)')
    parser.add_argument('--baud', '-b', type=int, default=921600, help='baud rate (default: 921600)')
    parser.add_argument('--ch', type=int, default=0, help='canal a exibir (default: 0)')
    parser.add_argument('--xmax', type=int, default=0, help='limite X (bins, 0=auto)')
    args = parser.parse_args()

    os.makedirs('log', exist_ok=True)
    start_dt = datetime.now().strftime('%Y%m%d_%H%M%S')
    csv_path = os.path.join('log', f'ch{args.ch}_{start_dt}.csv')
    csv_file = open(csv_path, 'w', newline='')
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        'timestamp', 'ch_id', 'nfft', 'fs_hz', 'dc_removed',
        'f1_hz', 'a1', 'thd', 'snr_db', 'slip_est', 'fr_hz', 'a_fr', 'a_2fr',
        'harmonics', 'diagnosis_text',
        'bars_high', 'bars_moderate', 'unbalance', 'misalignment',
        'thd_high', 'thd_moderate', 'thd_notice', 'snr_low'
    ])
    csv_file.flush()

    # contadores
    session_counts = {
        'bars_high': 0, 'bars_moderate': 0, 'unbalance': 0, 'misalignment': 0,
        'thd_high': 0, 'thd_moderate': 0, 'thd_notice': 0, 'snr_low': 0
    }
    minute_counts = session_counts.copy()
    last_minute = int(time.time() // 60)

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f'Escutando {args.port} @ {args.baud} baud ...  CSV: {csv_path}')

    plt.ion()
    fig, ax = plt.subplots()
    line, = ax.plot([], [], lw=1)
    ax.set_xlabel('bin')
    ax.set_ylabel('|X[k]|')
    ax.set_title('Espectro (0..~F_SPEC_MAX_HZ)')

    gauge_rects = {
        'thd': Rectangle((0, 0), 0, 0, fill=True, alpha=0.3),
        'snr': Rectangle((0, 0), 0, 0, fill=True, alpha=0.3),
        'slip': Rectangle((0, 0), 0, 0, fill=True, alpha=0.3),
    }
    for r in gauge_rects.values():
        ax.add_patch(r)
    gauge_text = ax.text(0.01, 0.97, "", transform=ax.transAxes, va='top', ha='left', fontsize=9,
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.6))

    # diagnóstico no gráfico (várias linhas coloridas)
    diag_texts: list[Text] = []
    def draw_diag(entries):
        nonlocal diag_texts
        # remove antigos
        for t in diag_texts:
            t.remove()
        diag_texts = []
        colors = {'E': 'red', 'W': 'orange', 'I': 'green'}
        # até 4 linhas
        base_y = 0.83
        dy = 0.05
        for idx, (sev, msg) in enumerate(entries[:4]):
            t = ax.text(0.01, base_y - idx*dy, msg,
                        transform=ax.transAxes, va='top', ha='left',
                        fontsize=9, color=colors.get(sev, 'black'),
                        bbox=dict(boxstyle='round', facecolor='white', alpha=0.6))
            diag_texts.append(t)

    last_log_ts = 0.0
    session_max_mag = 1.0  # maior |X| visto; eixo Y = 1.2 * session_max_mag

    try:
        while True:
            frame = read_frame(ser)
            if frame is None:
                continue
            if frame['ch_id'] != args.ch:
                continue

            y = frame['mag']
            x = list(range(len(y)))
            line.set_data(x, y)

            # atualiza sessão max
            if y:
                m = max(y)
                if m > session_max_mag:
                    session_max_mag = m

            xmax = (args.xmax if args.xmax > 0 else (len(y) - 1 if len(y) > 0 else 1))
            ax.set_xlim(0, xmax)
            ax.set_ylim(0, 1.2 * session_max_mag)

            # Gauges ----------------------------------------------------------
            y_top = 1.2 * session_max_mag
            bar_height = y_top * 0.05
            spacing = y_top * 0.02

            thd_val = frame['thd']
            snr_val = frame['snr_db']
            slip_val = max(0.0, frame['slip_est']) if frame['slip_est'] >= 0.0 else 0.0

            thd_norm = clamp01(thd_val / 0.20)
            snr_norm = clamp01(snr_val / 30.0)
            slip_norm = clamp01(slip_val / 0.10)

            total_width = xmax if xmax > 0 else (len(y) - 1 if len(y) > 0 else 100)
            width = max(10, total_width * 0.25)

            gauge_rects['thd'].set_xy((0, y_top - bar_height))
            gauge_rects['thd'].set_width(thd_norm * width)
            gauge_rects['thd'].set_height(bar_height - spacing)
            gauge_rects['thd'].set_facecolor(gauge_color(thd_val, 'thd'))

            gauge_rects['snr'].set_xy((0, y_top - 2*(bar_height + spacing)))
            gauge_rects['snr'].set_width(snr_norm * width)
            gauge_rects['snr'].set_height(bar_height - spacing)
            gauge_rects['snr'].set_facecolor(gauge_color(snr_val, 'snr'))

            gauge_rects['slip'].set_xy((0, y_top - 3*(bar_height + spacing)))
            gauge_rects['slip'].set_width(slip_norm * width)
            gauge_rects['slip'].set_height(bar_height - spacing)
            gauge_rects['slip'].set_facecolor(gauge_color(slip_val, 'slip'))

            gauge_text.set_text(f"THD={100*thd_val:.1f}%   SNR={snr_val:.1f} dB   SLIP={slip_val:.3f}")

            # Diagnóstico -----------------------------------------------------
            diag_entries, tags = quick_diagnosis(frame)
            draw_diag(diag_entries)

            # Persistência contadores
            now_minute = int(time.time() // 60)
            if now_minute != last_minute:
                minute_counts = {k: 0 for k in minute_counts.keys()}
                last_minute = now_minute
            for key, val in tags.items():
                if val:
                    session_counts[key] += 1
                    minute_counts[key] += 1

            # CSV -------------------------------------------------------------
            timestamp_iso = datetime.now().isoformat(timespec='seconds')
            harms_str = str(frame['harmonics'])
            diag_str_csv = " | ".join([f"[{sev}] {msg}" for (sev, msg) in diag_entries])
            csv_writer.writerow([
                timestamp_iso, frame['ch_id'], frame['nfft'], f"{frame['fs_hz']:.3f}", frame['dc_removed'],
                f"{frame['f1_hz']:.6f}", f"{frame['a1']:.6f}", f"{frame['thd']:.6f}", f"{frame['snr_db']:.6f}",
                f"{frame['slip_est']:.6f}", f"{frame['fr_hz']:.6f}", f"{frame['a_fr']:.6f}", f"{frame['a_2fr']:.6f}",
                harms_str, diag_str_csv,
                int(tags['bars_high']), int(tags['bars_moderate']), int(tags['unbalance']), int(tags['misalignment']),
                int(tags['thd_high']), int(tags['thd_moderate']), int(tags['thd_notice']), int(tags['snr_low']),
            ])
            csv_file.flush()

            # Terminal --------------------------------------------------------
            now_ts = time.time()
            if (now_ts - last_log_ts) > 1.0:
                last_log_ts = now_ts
                harms_txt = ', '.join([f"{o}ª:{f:.1f}Hz({a:.3f})" for (o, f, a) in frame['harmonics'][:6]])
                print(
                    f"CH{frame['ch_id']} f1={frame['f1_hz']:.2f}Hz A1={frame['a1']:.3f} "
                    f"THD={100*frame['thd']:.1f}% SNR={frame['snr_db']:.1f}dB "
                    f"slip={frame['slip_est']:.3f} fr={frame['fr_hz']:.1f}Hz | {harms_txt}"
                )
                emoji = {'E':'\U0001F534','W':'\U0001F7E0','I':'\u26AA'}
                diag_terms = [f"{emoji.get(sev,'')} {msg}" for (sev,msg) in diag_entries[:3]]
                print("   diag:", " | ".join(diag_terms))
                print("   session_counts:", session_counts)
                print("   minute_counts:", minute_counts)

            # Render ----------------------------------------------------------
            fig.canvas.draw()
            fig.canvas.flush_events()

    finally:
        try:
            csv_file.close()
        except Exception:
            pass

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
