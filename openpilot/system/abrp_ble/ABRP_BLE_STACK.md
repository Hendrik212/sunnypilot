# ABRP BLE Stack — Architecture, Fixes & Recovery Guide

## Overview

`abrp_ble` is a custom service that emulates an ELM327 OBD adapter over Bluetooth
Low Energy. ABRP (A Better Route Planner) on the phone connects to it as if it were
a real OBDLink CX dongle and reads battery/SoC data over the FFF0 GATT service.

The BLE stack is entirely custom because:
- AGNOS 17+ removed the system BlueZ tools (`bluetoothd`, `btmgmt`, etc.)
- The WCN3990 Bluetooth chip requires a kernel that exposes it over UART (`/dev/ttyHS0`)
  with `CONFIG_BT_HCIUART_QCA` enabled — the stock AGNOS kernel does not have this
- A custom kernel must be built and flashed before `abrp_ble` can function

---

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  abrp_ble.py (comma user)               │
│                                                         │
│  ┌──────────┐   ┌────────────┐   ┌──────────────────┐  │
│  │ELM327    │   │bless 0.3.0 │   │HCI adv patch     │  │
│  │Handler   │◄──│GATT server │   │(hci_adv_patch.py)│  │
│  └──────────┘   └─────┬──────┘   └────────┬─────────┘  │
│                       │ D-Bus              │ sudo       │
└───────────────────────┼────────────────────┼────────────┘
                        │                    │ raw HCI socket
               ┌────────▼────────┐   ┌───────▼──────────┐
               │  bluetoothd 5.66│   │  hci0 (kernel)   │
               │  (bundled, root)│──►│  /dev/ttyHS0     │
               └─────────────────┘   └──────────────────┘
                        ▲                    ▲
               D-Bus    │           btattach │
               system   │           ┌────────┴─────────┐
                        │           │btattach -P qca   │
                        │           │(bundled, root)   │
                        │           └──────────────────┘
               ┌────────┴────────┐
               │  D-Bus daemon   │
               │  (system.d/     │
               │  bluetooth.conf)│
               └─────────────────┘
```

### Key components

| Component | Path | Notes |
|-----------|------|-------|
| `abrp_ble.py` | `system/abrp_ble/abrp_ble.py` | Main service, runs as `comma` |
| `hci_adv_patch.py` | `system/abrp_ble/hci_adv_patch.py` | Root HCI socket helper |
| `bluetoothd` | `system/abrp_ble/bin/bluetoothd` | BlueZ 5.66, Debian 12 arm64 |
| `btattach` | `system/abrp_ble/bin/btattach` | BlueZ 5.66, Debian 12 arm64 |
| `btmgmt` | `system/abrp_ble/bin/btmgmt` | BlueZ 5.66, Debian 12 arm64 |
| QCA firmware | `system/abrp_ble/firmware/` | WCN3990 ROME rampatch + NVM |
| D-Bus policy | `/etc/dbus-1/system.d/bluetooth.conf` | Written at startup |

---

## Custom Kernel — Why and How

### Why a custom kernel is needed

The stock AGNOS 17 kernel does not include Bluetooth HCI UART support for the WCN3990
chip. The required kernel config options are:

```
CONFIG_BT=y
CONFIG_BT_BREDR=y
CONFIG_BT_LE=y
CONFIG_BT_HCIUART=y
CONFIG_BT_HCIUART_QCA=y
CONFIG_BT_QCA=y
# Disabled (conflicts with userspace btattach):
CONFIG_MSM_BT_POWER=n
CONFIG_BTFM_SLIM=n
CONFIG_BTFM_SLIM_WCN3990=n
```

The DT (`arch/arm64/boot/dts/qcom/comma_common.dtsi`) must have the in-kernel
`wcn3990-bt` node **removed** and the UART (`qupv3_se6_4uart`) **enabled**. The
kernel's `btqca.c` must have the firmware size limit raised to 512 KB to accept
WCN3990 ROME 0x02140201 rampatch/NVM blobs.

### Build environment

Build machine: `192.168.1.131` (Ubuntu, x86\_64)
Repo: `~/kernel-build/openpilot/agnos-builder/`
Kernel submodule: `agnos-kernel-sdm845/`

### Build steps

```bash
# On 192.168.1.131
cd ~/kernel-build/openpilot/agnos-builder

# First time only — initialise kernel submodule and extract cross-compiler
git submodule update --init agnos-kernel-sdm845
./tools/extract_tools.sh

# Build (runs inside Docker; requires Docker with buildkit)
./build_kernel.sh
# Output: output/boot.img
```

`build_kernel.sh` automatically:
1. Spins up a Docker container with the ARM64 cross-compiler
2. Removes `wcn3990-bt` DT node and enables `qupv3_se6_4uart` via a Python patch
3. Patches `btqca.c` firmware size limits
4. Enables all required BT config options
5. Builds `Image.gz-dtb` and wraps it in a signed `boot.img`

### Flash the kernel

Put the device in **QDL (EDL) mode** first (see `EDL_RECOVERY.md` if the device is
unresponsive).

```bash
# On 192.168.1.131
cd ~/kernel-build/openpilot/agnos-builder
./flash_kernel.sh
# Detects active slot (a/b), flashes output/boot.img to boot_<slot>
```

The device reboots into the new kernel automatically.

### Verify the kernel is active

```bash
ssh comma@192.168.1.197 "uname -r"
# Expected: 4.9.103   (built by hendrik@docker)
# Check BT UART driver is present:
ssh comma@192.168.1.197 "ls /dev/ttyHS0"
```

---

## Startup Sequence (what abrp_ble does at boot)

1. **Auto-chmod bundled binaries** — git LFS blobs lose exec bits; fixed on startup.

2. **Write D-Bus policy** (`_ensure_dbus_policy`) — writes
   `/etc/dbus-1/system.d/bluetooth.conf` if missing or stale. This policy:
   - Allows `root` to own `org.bluez`
   - Allows `root` to send method calls to any D-Bus connection (required for BlueZ
     to call back into bless's GATT objects during `RegisterApplication`)

3. **Start `bluetoothd`** — `sudo bluetoothd -n --noplugin=*`

4. **Write QCA firmware** — copies WCN3990 rampatch + NVM to `/data/firmware/qca/`
   via a tmpfs bind so the kernel `btqca` driver can load them.

5. **Set firmware class path** — writes `qca` to
   `/sys/bus/platform/devices/*/firmware_class` so the kernel looks in the right place.

6. **Start `btattach`** — `sudo btattach -B /dev/ttyHS0 -P qca`
   This attaches the WCN3990 chip to the kernel HCI layer, loads QCA firmware, and
   registers `hci0`.

7. **Poll for hci0** — polls `btmgmt info` (not sysfs — see pitfalls below) until
   `hci0` appears. Timeout: 35 seconds.

8. **Configure adapter** (`_configure_bt`):
   ```
   btmgmt power off
   btmgmt name "OBDLink CX"   ← sets HCI-level name (appears in advertisement)
   btmgmt le on
   btmgmt bredr off
   btmgmt connectable on
   btmgmt power on
   ```

9. **Start bless GATT server** — creates `BlessServer`, registers three GATT services:
   - `FFF0` — OBDLink-style UART (FFF1 notify, FFF2 write)
   - `6E400001` — Nordic UART Service (NUS)
   - `180A` — Device Information Service

10. **Force advertising on** — `btmgmt advertising on`

11. **Patch HCI advertisement data** (`hci_adv_patch.py` via sudo) — see next section.

12. **Health loop** — every 5 seconds checks `btmgmt info`; restarts BT stack if
    `hci0` disappears or advertising stops.

---

## The HCI Advertisement Patch (critical fix)

### Problem

`bluetoothd 5.66` does not include the `ServiceUUIDs` field from bless's D-Bus
`LEAdvertisement1` object in the actual `HCI_LE_Set_Advertising_Data` command sent
to the chip. As a result, the over-the-air advertisement contains no service UUID.

ABRP identifies OBDLink CX devices by the `FFF0` service UUID in the advertisement.
Without it, ABRP shows the device as "Unsupported device" and refuses to connect.

This was confirmed by:
- `busctl` showing `ServiceUUIDs: FFF0` in bless's D-Bus advertisement object ✓
- nRF Connect showing **no service UUID** in the air packet ✗
- A raw Python HCI script sending `HCI_LE_Set_Advertising_Data` directly showing
  **FFF0 present** in nRF Connect ✓

The issue does **not** affect btmgmt `add-adv` either — btmgmt goes through
bluetoothd's management socket and has the same bug.

### Fix

`hci_adv_patch.py` opens a raw `AF_BLUETOOTH / BTPROTO_HCI` socket (requires root)
and sends:

1. `HCI_LE_Set_Advertising_Data` with payload:
   ```
   02 01 06        Flags: LE General Discoverable, BR/EDR not supported
   03 03 F0 FF     Complete List of 16-bit UUIDs: 0xFFF0
   0B 09 4F 42 44 4C 69 6E 6B 20 43 58   Complete Local Name: "OBDLink CX"
   ```
2. `HCI_LE_Set_Advertising_Enable` → `0x01` (enable)

Called in `abrp_ble.py` via `sudo python3 hci_adv_patch.py` after `server.start()`.

The patch is stable — bluetoothd does not overwrite the advertisement data after it
has been set unless the adapter is power-cycled or advertising is restarted.

### If ABRP shows "unsupported device" again after an upstream merge

1. Check nRF Connect: does the device advertise FFF0?
   ```bash
   # Manually run the patch to test:
   ssh comma@192.168.1.197 "sudo python3 /data/openpilot/system/abrp_ble/hci_adv_patch.py"
   ```
2. If FFF0 appears after the manual patch → bluetoothd regression, patch is still
   needed. Verify `hci_adv_patch.py` is being called from `abrp_ble.py`.
3. If FFF0 still doesn't appear → chip/kernel issue, check btattach is running and
   hci0 is registered (`btmgmt info`).

---

## Known Pitfalls and Fixes

### sysfs false-positive for hci0 registration

`/sys/class/bluetooth/hci0` persists after btattach exits due to kernel reference
counting on the btattach file descriptor. Do **not** use sysfs to detect hci0
registration. Use `btmgmt info` — it reflects actual `hci_register_dev` state.

### sudo with `Defaults use_pty`

The sudoers configuration has `Defaults use_pty`, which causes sudo to allocate a
PTY for the child process. When abrp_ble is started without a controlling TTY
(e.g., `nohup`), `sudo btmgmt` returns rc=0 with empty stdout. The service **must**
be started by the openpilot manager (which gives it `pts/1`) for btmgmt to work.

Do not test by running `nohup python3 -m system.abrp_ble.abrp_ble` — it will fail
silently in all btmgmt calls.

### Blocking asyncio event loop

All subprocess calls in async context must use `asyncio.create_subprocess_exec`
(see `_arun_cmd`). Using `subprocess.run()` blocks the event loop for up to 3
seconds, preventing bless from responding to bluetoothd's `GetManagedObjects`
callback during `RegisterApplication`, causing a timeout and GATT registration
failure.

### D-Bus policy missing

On AGNOS 17+, the default system D-Bus policy denies method calls between
connections. BlueZ calls back into bless's D-Bus objects (root → comma user) to
enumerate GATT services during `RegisterApplication`. Without the custom policy in
`/etc/dbus-1/system.d/bluetooth.conf`, GATT registration silently fails.

The policy is written by `_ensure_dbus_policy()` at startup. If the file content
changes (e.g., previous version was different), it will be overwritten.

### btmgmt adapter name vs BlessServer name

`BlessServer(name="OBDLink CX")` sets the BlueZ D-Bus **Alias** only. It does NOT
set the HCI-level adapter name used in BLE advertisement packets. Use
`btmgmt name "OBDLink CX"` (called in `_configure_bt`) to set the name that
actually appears in the advertisement.

---

## Upstream Merge Checklist

When merging a new AGNOS version, verify the following before pushing to the device:

### 1. Kernel still compatible

```bash
ssh comma@192.168.1.197 "uname -r; ls /dev/ttyHS0"
```

If `ttyHS0` is missing or the kernel changed, rebuild and reflash:
```bash
cd ~/kernel-build/openpilot/agnos-builder
./build_kernel.sh && ./flash_kernel.sh
```

### 2. BlueZ binaries still work

```bash
ssh comma@192.168.1.197 "/data/openpilot/system/abrp_ble/bin/bluetoothd --version"
# Expected: 5.66
```

If AGNOS ships a new glibc version that breaks the Debian 12 binaries, they need to
be recompiled from source on a matching ARM64 target.

### 3. bless still installed

```bash
ssh comma@192.168.1.197 "ls /data/bless_packages/bless/"
```

bless is installed from openpilot's Python package management. Check
`system/abrp_ble/abrp_ble.py` imports work after any `uv` / package updates.

### 4. abrp_ble enabled in manager

```bash
grep -r "abrp_ble" selfdrive/manager/
```

The manager process entry must not be commented out. This has been toggled in past
merges (commits `e13e1e5f6` disabled it, `b5550f37b` re-enabled it).

### 5. Verify end-to-end after reboot

```bash
ssh comma@192.168.1.197 "sudo /data/openpilot/system/abrp_ble/bin/btmgmt -i hci0 info"
# Must show: current settings: powered connectable le advertising secure-conn
#            name OBDLink CX
```

Then open nRF Connect on phone → verify device visible with `FFF0` service UUID.
Then open ABRP → device should appear **bold** (supported).

---

## ABRP Response Format & Byte Indexing

### How ABRP parses responses

ABRP's `genericProtocol` parser:
1. Strips all spaces, `\r`, `\n` from the BLE notification
2. Searches for `"62"` (Mode 22 positive response byte) via `indexOf('62')`
3. Reads the service byte (`62`) and PID bytes (`01 05`)
4. Parses the **remaining** hex into a byte array: `A=byte[0]`, `B=byte[1]`, ..., `Z=byte[25]`, `AA=byte[26]`, `AB=byte[27]`, ..., `AF=byte[31]`
5. Evaluates a server-defined equation (e.g., `AF/2`) against that array

**ABRP does NOT handle ISO-TP multi-frame reassembly.** It expects the ELM327 to
do that internally. Our response must be a single line with the full payload.

### Response format (with headers on — ATH1)

```
7EC 10 3E 62 01 01 <61 frame bytes>    ← for 220101
7EC 10 2A 62 01 05 <41 frame bytes>    ← for 220105
```

The `7EC 10 XX` prefix is the ISO-TP first-frame header. ABRP skips past it because
`indexOf('62')` finds the `62` after it. This must be a **single line** — do NOT
split into multi-frame `7EC 21 ...` consecutive frames.

### Byte positions for Hyundai Ioniq 6

ABRP equations (fetched from server for the car model):

| PID | Equation | ABRP Index | Frame Byte | Description |
|-----|----------|------------|------------|-------------|
| 220105 | `AF/2` | 31 | frame[31] | Display SoC (%) |
| 220101 | varies | — | frame[4] | BMS SoC |
| 220101 | varies | — | frame[10,11] | Current (signed, /10 = A) |
| 220101 | varies | — | frame[12,13] | Voltage (/10 = V) |

**Critical:** ABRP strips `62 01 XX` (3 bytes) before indexing. So ABRP letter `AF`
(index 31) maps to frame byte 31 in our bytearray, NOT byte 28 or 30.

### If SOC shows 0% in ABRP after changes

1. Check the equation in ABRP settings → OBD config → look for the SOC equation
2. Convert the ABRP letter index: A=0, ..., Z=25, AA=26, AB=27, ..., AF=31
3. That index maps directly to the frame bytearray index (after `62 01 XX` prefix)
4. Use the marker test: put distinct values (0x38, 0x3A, 0x3C, 0x3E) at candidate
   positions and read which percentage ABRP displays

---

## Debugging

### Check abrp_ble is running

```bash
ps aux | grep abrp_ble | grep -v grep
```

### Check BT stack status

```bash
sudo /data/openpilot/system/abrp_ble/bin/btmgmt -i hci0 info
sudo /data/openpilot/system/abrp_ble/bin/btmgmt -i hci0 advinfo
ps aux | grep -E 'btattach|bluetoothd'
```

### Check abrp_ble logs

```bash
sudo journalctl -u comma -f | grep -v btmgmt
```

### Check GATT objects registered

```bash
sudo busctl tree :1.44   # bless's D-Bus name (number may vary)
# Should show:
# /org/bluez/OBDLinkCX/advertisement1
# /org/bluez/OBDLinkCX/service0001  (FFF0)
# /org/bluez/OBDLinkCX/service0002  (NUS)
# /org/bluez/OBDLinkCX/service0003  (DIS)
```

### Manually test advertisement patch

```bash
sudo python3 /data/openpilot/system/abrp_ble/hci_adv_patch.py
# Then check nRF Connect for FFF0
```

### Force restart abrp_ble (via manager)

The manager will restart it automatically after a crash. To force a restart, reboot
the device. Do not attempt to start it manually with `nohup` — see "sudo pty" pitfall.

### AT command log

When ABRP connects, `abrp_ble.py` logs all received commands:
```
[ABRP-BLE] RX: ATZ
[ABRP-BLE] TX: ELM327 v1.5
[ABRP-BLE] RX: ATSP0
[ABRP-BLE] TX: OK
```

If no `RX:` lines appear, ABRP connected at the BLE level but no data arrived —
check the write characteristic UUID and the `_on_write` handler.

---

## IP Addresses

| Device | IP | Branch | Notes |
|--------|----|--------|-------|
| Hyundai Ioniq 6 | `192.168.1.197` | `isla-master` | abrp_ble active |
| VW device | `192.168.1.100` | `vw-master` | no abrp_ble |
| Build machine | `192.168.1.131` | — | Ubuntu x86\_64, kernel builds |

SSH user on device: `comma`
SSH user on build machine: `hendrik`
