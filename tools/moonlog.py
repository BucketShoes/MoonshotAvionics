#!/usr/bin/env python3
"""moonlog.py - decode raw 8MB flash dumps from the rocket (moonshot) or base
station (moonbase) into one line per log record.

The partition table in the dump is read to find log_index / log_data, so the
same script works on both devices. Records are walked linearly through the
data ring (not via the index), so half-written or orphaned records are still
seen and reported rather than silently dropped.

Usage:
  python3 tools/moonlog.py DUMP.bin                    # every record, text
  python3 tools/moonlog.py DUMP.bin --from 473400 --to 476000
  python3 tools/moonlog.py DUMP.bin --sessions         # boot/session summary
  python3 tools/moonlog.py DUMP.bin --csv out.csv      # flat CSV
  python3 tools/moonlog.py DUMP.bin --types AF,0B,0F   # filter by type byte
  python3 tools/moonlog.py DUMP.bin --changes          # header records only when
                                                       # phase/flags change

Formats: "design documents/Packet, log, and data formats.txt" and
"design documents/Ring buffer layout and log storage format.txt". Where the
code and the doc disagree (page 0x0B flag bits, 0x0F page), the code wins.
"""
import argparse, csv, datetime, struct, sys

SECTOR = 4096
CHUNK = 256
HDR = 6
MAXP = 254
SNR_LOCAL = 0x7F

PHASES = {0: "IDLE", 1: "PAD_READY", 2: "BOOST", 3: "COAST", 4: "APOGEE",
          5: "DROGUE", 6: "MAIN_DEPLOY", 7: "DESCENT", 8: "LANDED",
          9: "GROUND_TEST", 10: "SLEEP", 11: "STARTUP", 15: "ERROR"}
CMDS = {0x01: "ARM", 0x02: "DISARM", 0x03: "FIRE_PYRO", 0x10: "SET_TX_RATE",
        0x12: "SET_RADIO", 0x40: "PING", 0x50: "OTA_BEGIN", 0x51: "OTA_FINALIZE",
        0x52: "OTA_CONFIRM", 0x60: "LR_LISTEN", 0xF0: "REBOOT", 0xF1: "LOG_ERASE"}
GPS_FRAC = {65533: "nofix", 65534: "gpsinit", 65535: "gpsoff"}


# ---------------------------------------------------------------- flash layout

def partitions(img):
    out = {}
    for off in range(0x8000, 0x8C00, 32):
        e = img[off:off + 32]
        if e[:2] != b"\xaa\x50":
            break
        poff, psz = struct.unpack_from("<II", e, 4)
        out[e[12:28].rstrip(b"\0").decode()] = (poff, psz)
    return out


def walk(data):
    """Linear scan of the data ring. Yields (phys, len, snr, ts, payload).
    Order is physical; the caller re-orders by virtual position."""
    pos, n = 0, len(data)
    while pos + HDR <= n:
        s = pos % SECTOR
        ln = data[pos]
        if ln == 0xFF or ln == 0 or ln > MAXP or s + HDR + ln > SECTOR:
            pos = pos - s + SECTOR  # rest of this sector is gap/erased
            continue
        snr = struct.unpack_from("<b", data, pos + 1)[0]
        ts = struct.unpack_from("<I", data, pos + 2)[0]
        yield pos, ln, snr, ts, bytes(data[pos + HDR:pos + HDR + ln])
        pos += HDR + ln


def load(path):
    img = open(path, "rb").read()
    parts = partitions(img)
    io, isz = parts["log_index"]
    do, dsz = parts["log_data"]
    index = list(struct.unpack_from("<%dI" % (isz // 4), img, io))
    data = img[do:do + dsz]
    return img, parts, index, data, dsz


def numbered(index, data, ring):
    """Assign record numbers using the index: chunk k starts at virtual pos
    index[k]. Walk each surviving chunk forward. Yields dicts."""
    used = [(k, v) for k, v in enumerate(index) if v != 0xFFFFFFFF]
    if not used:
        return
    lastv = used[-1][1]
    # Estimate end of written data: walk the last chunk to its end
    # Anything older than (end - ring) has been overwritten.
    for i, (k, v) in enumerate(used):
        nextv = used[i + 1][1] if i + 1 < len(used) else None
        if nextv is not None and nextv + 0 < lastv - ring + SECTOR:
            continue  # whole chunk overwritten by the ring
        pos = v
        for j in range(CHUNK):
            s = pos % SECTOR
            if s and s + HDR > SECTOR:
                pos += SECTOR - s
                s = 0
            p = pos % ring
            ln = data[p]
            if (ln == 0xFF or ln == 0) and s:
                pos += SECTOR - s
                p = pos % ring
                ln = data[p]
            if ln == 0xFF or ln == 0 or ln > MAXP:
                break
            if nextv is not None and pos >= nextv:
                break
            if (pos % SECTOR) + HDR + ln > SECTOR:
                break
            # ring overwrite check: older than one ring before the head
            if pos < lastv - ring:
                pos += HDR + ln
                continue
            snr = struct.unpack_from("<b", data, p + 1)[0]
            ts = struct.unpack_from("<I", data, p + 2)[0]
            yield dict(rec=k * CHUNK + j, vpos=pos, snr=snr, ts=ts,
                       pl=bytes(data[p + HDR:p + HDR + ln]))
            pos += HDR + ln


# ---------------------------------------------------------------- decoding

def u16(b, o): return struct.unpack_from("<H", b, o)[0]
def s16(b, o): return struct.unpack_from("<h", b, o)[0]
def u32(b, o): return struct.unpack_from("<I", b, o)[0]
def s32(b, o): return struct.unpack_from("<i", b, o)[0]


def flags_str(f):
    ph = PHASES.get(f & 15, str(f & 15))
    bits = []
    if f & 0x10: bits.append("ARMED")
    if f & 0x20: bits.append("P1FIRED")
    if f & 0x40: bits.append("P2FIRED")
    if f & 0x80: bits.append("P3FIRED")
    if f & 0x100: bits.append("LOWBATT")
    bits.append("baro" + ("ok" if f & 0x200 else "BAD"))
    bits.append("accel" + ("ok" if f & 0x400 else "BAD"))
    if f & 0x800: bits.append("armReady")
    return ph, " ".join(bits)


def frac(v):
    return GPS_FRAC.get(v, "%.5f" % (v * 0.00002)) if v > 49999 else ".%05d" % (v * 2)


def page(t, b):
    """Decode a data page body (after the type byte). Returns (name, dict)."""
    try:
        if t == 0x01:
            return "GPS", dict(lat=s32(b, 0) / 1e7, lon=s32(b, 4) / 1e7,
                               hdop=b[8] / 10, sats=b[9])
        if t == 0x02:
            return "BARO", dict(altM=s32(b, 0) / 100, vvel=s16(b, 4) / 10,
                                gndM=s16(b, 6))
        if t == 0x03:
            return "MAG", dict(x=s16(b, 0), y=s16(b, 2), z=s16(b, 4))
        if t == 0x04:
            x, y, z = s16(b, 0), s16(b, 2), s16(b, 4)
            return "ACCEL", dict(x=x, y=y, z=z,
                                 tot=round((x * x + y * y + z * z) ** .5))
        if t == 0x05:
            return "GYRO", dict(x=s16(b, 0) / 10, y=s16(b, 2) / 10, z=s16(b, 4) / 10)
        if t == 0x06:
            return "GPSX", dict(spd=u16(b, 0) / 100, crs=u16(b, 2) / 100,
                                age=b[4], vdop=b[5] / 10, altM=s32(b, 6) / 100)
        if t == 0x08:
            return "SYS", dict(tempC=struct.unpack_from("<b", b, 0)[0],
                               heapKB=u16(b, 1), upS=u16(b, 3), battMv=u16(b, 5),
                               logRec=u32(b, 7))
        if t == 0x09:
            return "PEAKS", dict(maxAltM=s32(b, 0) / 100, maxG=u16(b, 4) / 100,
                                 maxVv=s16(b, 6) / 10)
        if t == 0x0A:
            return "CMDACK", dict(nonce=u32(b, 0), result=b[4],
                                  rssi=struct.unpack_from("<b", b, 5)[0],
                                  snr=struct.unpack_from("<b", b, 6)[0] / 4,
                                  badHmac=u16(b, 7), rxPos=b[9])
        if t == 0x0B:
            # code (telemetry.cpp buildPage0B), not the doc:
            # [0..2] ch1..3 fired, [4..6] ch1..3 continuity, [8] HV present
            f = u16(b, 4)
            return "FLIGHT", dict(tLaunchMs=s32(b, 0),
                                  fired="".join("123"[i] if f >> i & 1 else "-" for i in range(3)),
                                  cont="".join("123"[i] if f >> (4 + i) & 1 else "-" for i in range(3)),
                                  hv=f >> 8 & 1)
        if t == 0x0C:
            return "RADIO", dict(delayedTx=u16(b, 0), invalidRx=u16(b, 2),
                                 noise=struct.unpack_from("<b", b, 4)[0], sync=b[5])
        if t == 0x0D:
            ms = struct.unpack_from("<Q", b, 0)[0]
            iso = (datetime.datetime.fromtimestamp(ms / 1000, datetime.timezone.utc)
                   .strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + "Z") if ms else "none"
            return "UTC", dict(utc=iso, ms=ms)
        if t == 0x0E:
            dur, mn, mx = u16(b, 0), s16(b, 2) * 8, s16(b, 4) * 8
            n = len(b) - 6
            return "THRUST", dict(durMs=dur, minMg=mn, maxMg=mx, n=n)
        if t == 0x0F:
            f = b[0]
            return "PYRO", dict(cont="".join("123"[i] if f >> i & 1 else "-" for i in range(3)),
                                hv=f >> 3 & 1,
                                fired="".join("123"[i] if f >> (4 + i) & 1 else "-" for i in range(3)),
                                active=b[1], durMs=u16(b, 2), hvMv=u16(b, 4))
    except (struct.error, IndexError):
        return "TRUNC%02X" % t, dict(raw=b.hex())
    return "PAGE%02X" % t, dict(raw=b.hex())


def header(pl):
    devid, la, lo, alt, f = pl[1], u16(pl, 2), u16(pl, 4), s16(pl, 6), u16(pl, 8)
    ph, fl = flags_str(f)
    return dict(dev=devid, lat=frac(la), lon=frac(lo),
                altM=("inv" if alt == -32768 else alt), phase=ph, flags=fl, raw=f)


def decode(pl):
    """Return (kind, fields) for a record payload."""
    t = pl[0]
    if t < 0x80:
        return page(t, pl[1:])
    if t == 0xAF:
        d = header(pl)
        if len(pl) > 10:
            name, pd = page(pl[10], pl[11:])
            d["page"] = name
            d.update({"p." + k: v for k, v in pd.items()})
        return ("TELEM" if len(pl) > 10 else "HDR"), d
    if t == 0x9A:
        d = dict(target=pl[1], cmd=CMDS.get(pl[2], "0x%02X" % pl[2]),
                 nonce=u32(pl, 3), params=pl[7:-10].hex())
        if pl[2] == 0x01 and len(pl) - 17 >= 9:
            p = pl[7:-10]
            d.update(boostMg=u16(p, 0), boostAltM=u16(p, 2), coastAltM=u16(p, 4),
                     mainAltM=u16(p, 6), armFlags=p[8])
        if pl[2] == 0x12 and len(pl) - 17 >= 3:
            p = pl[7:-10]
            d.update(ch=p[0], sf=p[1], pwr=struct.unpack_from("<b", p, 2)[0])
        if pl[2] == 0x10 and len(pl) - 17 >= 1:
            d.update(rate=struct.unpack_from("<b", pl, 7)[0])
        return "CMD", d
    if t == 0xBB:
        w = pl[2] | pl[3] << 8 | pl[4] << 16 if len(pl) >= 5 else 0
        return "LR", dict(latFrac=w & 0x7FF, lonFrac=w >> 11 & 0x7FF, lowBatt=w >> 22 & 1)
    if t == 0xCA:
        return "LOGCHUNK", dict(src=pl[1], start=u32(pl, 2), count=pl[6])
    return "UNK%02X" % t, dict(raw=pl.hex())


def fmt(d):
    return " ".join("%s=%s" % (k, v) for k, v in d.items() if k != "raw" or len(d) == 1)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dump")
    ap.add_argument("--from", dest="frm", type=int, default=0)
    ap.add_argument("--to", type=int, default=1 << 32)
    ap.add_argument("--types", help="comma list of hex type bytes, e.g. AF,0B,9A")
    ap.add_argument("--sessions", action="store_true",
                    help="summarise boot sessions (timestamp resets)")
    ap.add_argument("--changes", action="store_true",
                    help="only print header records when phase/flags change; other types as normal")
    ap.add_argument("--csv")
    ap.add_argument("--orphans", action="store_true",
                    help="list physically present records the index walk did not reach")
    a = ap.parse_args()

    img, parts, index, data, ring = load(a.dump)
    used = [v for v in index if v != 0xFFFFFFFF]
    print("# partitions:", ", ".join("%s@0x%x+0x%x" % (k, o, sz) for k, (o, sz) in parts.items()))
    print("# ring=%d chunks=%d head~vpos=%d (wrapped=%s)" %
          (ring, len(used), used[-1] if used else 0, (used[-1] if used else 0) > ring))
    types = {int(x, 16) for x in a.types.split(",")} if a.types else None

    recs = list(numbered(index, data, ring))
    if a.orphans:
        seen = {r["vpos"] % ring for r in recs}
        for p, ln, snr, ts, pl in walk(data):
            if p not in seen:
                k, d = decode(pl)
                print("ORPHAN phys=0x%06x t=%10.3f %s %s" % (p, ts / 1000, k, fmt(d)))
        return

    if a.sessions:
        # A boot resets timestamps; corrupt records can look like a reset too,
        # so only split when records a little further on stay low as well.
        start, prev = None, None
        for i, r in enumerate(recs):
            ahead = recs[min(i + 12, len(recs) - 1)]
            if prev is not None and (
                    (r["ts"] + 2000 < prev["ts"] and not ahead["ts"] + 2000 < prev["ts"]) or
                    (r["ts"] > prev["ts"] + 60000 and ahead["ts"] < prev["ts"] + 60000)):
                print("  (out-of-sequence record %d t=%.3f — corrupt?)" % (r["rec"], r["ts"] / 1000))
                continue
            if prev is None or r["ts"] + 2000 < prev["ts"]:
                if start:
                    print("session rec %d..%d  t=%.1fs..%.1fs  (%d recs)" %
                          (start["rec"], prev["rec"], start["ts"] / 1000, prev["ts"] / 1000,
                           prev["rec"] - start["rec"] + 1))
                start = r
            prev = r
        if start:
            print("session rec %d..%d  t=%.1fs..%.1fs  (%d recs)" %
                  (start["rec"], prev["rec"], start["ts"] / 1000, prev["ts"] / 1000,
                   prev["rec"] - start["rec"] + 1))
        return

    w = None
    if a.csv:
        w = csv.writer(open(a.csv, "w", newline=""))
        w.writerow(["rec", "t_s", "snr", "kind", "fields"])
    lastflags = None
    for r in recs:
        if not (a.frm <= r["rec"] < a.to):
            continue
        pl = r["pl"]
        if types and pl[0] not in types:
            continue
        k, d = decode(pl)
        if a.changes and k == "HDR":
            key = (d["phase"], d["flags"])
            if key == lastflags:
                continue
            lastflags = key
        snr = "" if r["snr"] == SNR_LOCAL else " snr=%.2f" % (r["snr"] / 4)
        line = "%8d t=%10.3f%s %-7s %s" % (r["rec"], r["ts"] / 1000, snr, k, fmt(d))
        if w:
            w.writerow([r["rec"], r["ts"] / 1000, "" if r["snr"] == SNR_LOCAL else r["snr"] / 4, k, fmt(d)])
        else:
            print(line)


if __name__ == "__main__":
    main()
