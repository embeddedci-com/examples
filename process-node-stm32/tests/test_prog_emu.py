"""The pod's pin programmers (``prog_flash``) against the node's chip emulators (progemu.c).

The NUCLEO answers as a chip on the programming pins: `prog_emu <proto>` on the console starts an
emulator (UPDI, PIC with either command set, C2, TPI, PDI), the pod flashes it over the same wires a
real chip would be on, and the emulated flash is compared with the image by CRC32 (`prog_emu stat`).

Wiring (the existing wires, README): clock LA13 -> PB14, data LA14 <-> PB12, PIC MCLR / TPI RESET
LA5 -> PB0, UPDI LA7 <-> PC10. Needs pod firmware with the ``prog_flash`` command (the tests skip
without it) and the pod on the deep-replay gateware image, which stages the image in PSRAM.

Each test prints the edge-to-edge times the emulator measured (``prog_emu stat``): the pod's
bit-bang timing seen from the chip. Run with ``-s`` to see them; they are also recorded as test
properties and appended to the file named by ``PROG_EMU_TIMING_LOG`` when it is set.
"""

from __future__ import annotations

import json
import os
import re
import time
import zlib

import pytest

from embeddedci.benchpod import FpgaImage

pytestmark = [pytest.mark.hardware, pytest.mark.prog]

CLK, DATA, RESET, UPDI_DATA = 13, 14, 5, 7

# PIC flags (prog_pic.h): command set and the code-protect bit
PIC_F_SET8, PIC_F_BYTEPC, PIC_F_HAS_EE, PIC_F_Q10 = 1, 2, 4, 0x10


def pic_cp(bit):
    return bit << 8


# name -> what the console starts, what the pod is told, the image
CASES = {
    "pic": dict(pins={"proto": "pic", "clk": CLK, "data": DATA, "reset": RESET}, page=16,
                flags=pic_cp(5) | PIC_F_HAS_EE, cp_addr=0x1000E, size=8192, id="a4000200", w14=True),
    "pic8": dict(pins={"proto": "pic", "clk": CLK, "data": DATA, "reset": RESET}, page=32,
                 flags=PIC_F_SET8 | pic_cp(1), cp_addr=0x10016, size=8192, id="b2300120", w14=True),
    "pic18": dict(pins={"proto": "pic", "clk": CLK, "data": DATA, "reset": RESET}, page=32,
                  flags=PIC_F_SET8 | PIC_F_BYTEPC | pic_cp(3), cp_addr=0x300004, size=8192, id="317a0320"),
    "pic18q10": dict(pins={"proto": "pic", "clk": CLK, "data": DATA, "reset": RESET}, page=32,
                     flags=PIC_F_SET8 | PIC_F_BYTEPC | PIC_F_Q10 | pic_cp(3), cp_addr=0x300004, size=8192,
                     id="107b0420"),
    "c2": dict(pins={"proto": "c2", "clk": CLK, "data": DATA}, page=512, size=8192, id="320321",
               keep_last_ff=True),
    "c2early": dict(pins={"proto": "c2", "clk": CLK, "data": DATA}, page=512, size=8192, id="320321",
                    keep_last_ff=True),
    "tpi": dict(pins={"proto": "tpi", "clk": CLK, "data": DATA, "reset": RESET}, page=16, size=1024,
                id="1e9003"),
    # flags: words per write (2 = ATtiny20, 4 = ATtiny40), 8 = page erase (ATtiny102/104)
    "tpi20": dict(pins={"proto": "tpi", "clk": CLK, "data": DATA, "reset": RESET}, page=16, flags=2,
                  size=2048, id="1e910f"),
    "tpi40": dict(pins={"proto": "tpi", "clk": CLK, "data": DATA, "reset": RESET}, page=16, flags=4,
                  size=4096, id="1e920e"),
    "tpi104": dict(pins={"proto": "tpi", "clk": CLK, "data": DATA, "reset": RESET}, page=16, flags=8,
                   size=1024, id="1e900b"),
    "pdi": dict(pins={"proto": "pdi", "clk": CLK, "data": DATA}, page=128, size=20480, id="1e9441"),
    "updi": dict(pins={"proto": "updi", "data": UPDI_DATA}, page=64, size=16384, id="1e9421",
                 uart=True),
    "updimega": dict(pins={"proto": "updi", "data": UPDI_DATA}, page=64, size=16384, id="1e9426",
                     uart=True),
    "updi2": dict(pins={"proto": "updi", "data": UPDI_DATA}, page=512, size=16384, id="1e970a",
                  uart=True),
}


# ---------------------------------------------------------------- helpers

def _kv(line):
    return {k: v for k, v in re.findall(r"(\w+)=(\S+)", line)}


class Emu:
    """One emulator on the node, driven by the console, flashed by the pod."""

    def __init__(self, benchpod, node, stage):
        self.bp, self.node, self.stage = benchpod, node, stage

    def start(self, name):
        m = self.node.cmd(f"prog_emu {name}", r"PROG_EMU (ok|err)[^\r\n]*\r\n")
        assert m.group(1) == "ok", m.group(0)

    def stop(self):
        self.node.cmd("prog_emu stop", r"PROG_EMU ok stopped\r\n")

    def lock(self):
        self.node.cmd("prog_emu lock", r"PROG_EMU ok locked[^\r\n]*\r\n")

    def clear(self):
        self.node.cmd("prog_emu clear", r"PROG_EMU ok cleared\r\n")

    def stat(self):
        """``prog_emu stat`` as a dict of lines: stat, high_ns, low_ns, crc, size, <proto>..."""
        m = self.node.cmd("prog_emu stat", r"(?s)PROG_EMU stat .*?PROG_EMU size[^\r\n]*\r\n")
        out = {}
        for line in m.group(0).splitlines():
            parts = line.split(None, 2)
            if len(parts) >= 2 and parts[0] == "PROG_EMU":
                key = parts[1]
                out[key] = _kv(line)
                out[key]["_line"] = line
        return out

    def dump(self, mem, addr, n):
        m = self.node.cmd(f"prog_emu dump {mem} {addr} {n}", r"(?s)PROG_EMU dump [^\r\n]*\r\n.*?\r\n>")
        hexes = re.findall(r"^[0-9a-f]{4}:([0-9a-f]+)\r?$", m.group(0), re.M)
        return bytes.fromhex("".join(hexes))

    def job(self, cmd, timeout=90.0, **extra):
        """Run one prog_flash op to its end (the console is closed meanwhile for UPDI: the pod's
        UART engine and a UART session exclude each other). Returns the final status."""
        req = {"cmd": "prog_flash", **cmd, **extra}
        uart = cmd.get("proto") == "updi"
        ctx = self.node.paused() if uart else _nullctx()
        with ctx:
            r = self.bp.command(req)
            assert r.get("started"), r
            end = time.monotonic() + timeout
            while True:
                st = self.bp.command({"cmd": "prog_flash", "op": "status"})
                if not st.get("running"):
                    return st
                if time.monotonic() > end:
                    self.bp.command({"cmd": "prog_flash", "op": "abort"})
                    raise TimeoutError(f"prog_flash did not finish in {timeout} s: {st}")
                time.sleep(0.1)


class _nullctx:
    def __enter__(self):
        return self

    def __exit__(self, *a):
        return False


def image_for(case, seed_bytes=None):
    img = bytearray(seed_bytes if seed_bytes is not None else os.urandom(case["size"]))
    if case.get("w14"):
        for i in range(1, len(img), 2):
            img[i] &= 0x3F           # 14-bit words: the high byte keeps 6 bits
    if case.get("keep_last_ff"):
        img[-1] = 0xFF               # C2: the flash's last byte is the lock byte
    return bytes(img)


def _cp(case):
    return {"cp_addr": case["cp_addr"]} if "cp_addr" in case else {}


def start_args(case, img, **extra):
    a = {"op": "start", **case["pins"], "mem": "flash", "addr": 0, "len": len(img), "page": case["page"],
         "erase": "chip", "expect_id": "0x" + case["id"][:4], "flags": case.get("flags", 0), "run": True}
    if "cp_addr" in case:
        a["cp_addr"] = case["cp_addr"]
    a.update(extra)
    return a


def report(name, what, stat):
    hi, lo = stat.get("high_ns"), stat.get("low_ns")
    line = f"PROG_EMU {name} {what}: stat[{stat['stat']['_line'][9:]}]"
    if hi and "min" in hi:
        line += f" | high_ns min={hi['min']} max={hi['max']} avg={hi['avg_under_100us']}"
    if lo and "min" in lo:
        line += f" | low_ns min={lo['min']} max={lo['max']} avg={lo['avg_under_100us']}"
    if "updi" in stat:
        u = stat["updi"]
        line += f" | updi baud={u.get('baud')} sync_bit_ns={u.get('sync_bit_ns')} syncs={u.get('syncs')}"
    print(line)
    log = os.environ.get("PROG_EMU_TIMING_LOG")
    if log:
        with open(log, "a") as f:
            f.write(json.dumps({"proto": name, "what": what, "stat": {k: v for k, v in stat.items()}}) + "\n")


# ---------------------------------------------------------------- fixtures

@pytest.fixture(scope="module")
def emu(benchpod, wiring, node):
    try:
        st = benchpod.command({"cmd": "prog_flash", "op": "status"})
    except Exception as e:                      # an older pod does not know the command
        pytest.skip(f"the pod's firmware has no prog_flash: {e}")
    if not isinstance(st, dict) or "protos" not in st:
        pytest.skip(f"the pod's firmware has no prog_flash: {st}")
    info = node.info()
    if tuple(int(x) for x in info["version"].split(".")) < (0, 3, 4):
        pytest.skip("the node's firmware has no prog_emu (0.3.4+): flash it (--benchpod-firmware)")
    benchpod.fpga_image(FpgaImage.DEEP_REPLAY)
    # the wiring profile marks LA14 (PB12, the node's SPI CS) and others as DUT outputs, and the pod
    # refuses to drive a DUT output (BP0034): during these tests the chip wires are bidirectional
    benchpod.la_dir([CLK, DATA, RESET, UPDI_DATA], "bidir")
    stage = getattr(benchpod._transport, "stage_psram", None)
    if stage is None:
        pytest.skip("staging an image needs the pod over the LAN (TCP) or the cloud")
    e = Emu(benchpod, node, stage)
    try:
        yield e
    finally:
        node.cmd("prog_emu stop", r"PROG_EMU ok stopped\r\n")
        benchpod.apply_pin_directions()            # the profile's directions again


@pytest.fixture(params=list(CASES))
def case_name(request, emu):
    yield request.param
    emu.stop()                                  # the node's own pins back after every test


# ---------------------------------------------------------------- tests

def test_info_reads_the_chip_id(emu, case_name):
    case = CASES[case_name]
    emu.start(case_name)
    st = emu.job({"op": "info", **case["pins"]}, flags=case.get("flags", 0), **_cp(case))
    assert st["result"] == "ok", st
    assert st["id"] == case["id"], st


def test_erase_chip_and_flash(emu, case_name):
    """Random image into PSRAM, info, start with a chip erase, poll to done: the emulated flash
    holds the image (CRC32 over the whole memory) and the emulator saw no protocol errors."""
    case = CASES[case_name]
    img = image_for(case)
    emu.start(case_name)
    emu.stage(img)
    st = emu.job(start_args(case, img))
    assert st["result"] == "ok", st
    assert st["id"] == case["id"], st
    assert st["done"] == st["total"] and st["total"] > 0, st
    stat = emu.stat()
    report(case_name, "flash", stat)
    assert int(stat["crc"]["flash"], 16) == zlib.crc32(img), \
        f"flash CRC {stat['crc']['flash']} != image {zlib.crc32(img):08x}"
    assert int(stat["stat"]["frames"]) > 0
    assert int(stat["stat"]["edges"]) > 0 or case.get("uart")
    # the hard protocol errors (bad frames, early commands); timing waits are reported, not failed
    hard = int(stat["stat"]["errors"]) - int(stat["stat"]["timing"])
    assert hard == 0, stat["stat"]["_line"]
    # a few bytes through the console: dump agrees with the image
    assert emu.dump("flash", 0, 32) == img[:32]


def test_wrong_expect_id_stops_before_erasing(emu, case_name):
    case = CASES[case_name]
    img = image_for(case)
    emu.start(case_name)
    before = emu.stat()["crc"]["flash"]
    emu.stage(img)
    st = emu.job(start_args(case, img, expect_id="0x00"))
    assert st["result"].startswith("wrong chip"), st
    assert emu.stat()["crc"]["flash"] == before, "the flash changed although the chip was the wrong one"


def test_locked_chip(emu, case_name):
    case = CASES[case_name]
    img = image_for(case)
    emu.start(case_name)
    emu.lock()
    emu.stage(img)
    # no erase: the chip refuses
    st = emu.job(start_args(case, img, erase="none"))
    assert st["result"].startswith("locked"), st
    # a chip erase unlocks it, then the image goes in
    st = emu.job(start_args(case, img, erase="chip"))
    assert st["result"] == "ok", st
    assert int(emu.stat()["crc"]["flash"], 16) == zlib.crc32(img)


def test_stopped_emulator_does_not_answer(emu, case_name):
    case = CASES[case_name]
    emu.start(case_name)
    emu.stop()
    st = emu.job({"op": "info", **case["pins"]}, flags=case.get("flags", 0), **_cp(case))
    assert st["result"].startswith("no answer") or st["result"].startswith("protocol error"), st
