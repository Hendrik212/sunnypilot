#!/usr/bin/env python3
"""
Patch HCI LE advertising data to include FFF0 service UUID.

Must run as root (raw HCI socket requires CAP_NET_RAW / root).
Called via sudo by abrp_ble.py after bless server starts.

ADV_IND payload built here:
  02 01 06        Flags: LE General Discoverable, BR/EDR not supported
  03 03 F0 FF     Complete List of 16-bit UUIDs: 0xFFF0 (OBDLink CX service)
  0B 09 ...       Complete Local Name: "OBDLink CX"
"""
import socket
import struct
import time

HCI_COMMAND_PKT       = 0x01
OGF_LE                = 0x08
OCF_LE_SET_ADV_DATA   = 0x0008
OCF_LE_SET_ADV_ENABLE = 0x000A


def opcode(ogf, ocf):
    return (ogf << 10) | ocf


def send_cmd(sock, ogf, ocf, params=b''):
    pkt = struct.pack('<BHB', HCI_COMMAND_PKT, opcode(ogf, ocf), len(params)) + params
    sock.send(pkt)
    time.sleep(0.05)


adv_data = bytes([
    0x02, 0x01, 0x06,                                              # Flags
    0x03, 0x03, 0xF0, 0xFF,                                        # UUID FFF0
    0x0B, 0x09, 0x4F, 0x42, 0x44, 0x4C, 0x69, 0x6E, 0x6B,
                0x20, 0x43, 0x58,                                  # "OBDLink CX"
])
payload = bytes([len(adv_data)]) + adv_data + bytes(31 - len(adv_data))

sock = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_RAW, socket.BTPROTO_HCI)
sock.bind((0,))  # hci0
send_cmd(sock, OGF_LE, OCF_LE_SET_ADV_DATA, payload)
send_cmd(sock, OGF_LE, OCF_LE_SET_ADV_ENABLE, b'\x01')
sock.close()
