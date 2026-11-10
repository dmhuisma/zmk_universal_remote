#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.11,<3.15"
# dependencies = ["bleak", "bthome-ble"]
# ///
"""Prints the BTHome packets around you, decoded by bthome-ble: the parser
Home Assistant's BTHome integration uses, so what it prints is what Home
Assistant would see.

    uv run tools/bthome_monitor.py [--address XX:XX:XX:XX:XX:XX]

Each packet shows its raw service data, the values and events decoded from it,
and which remote button a dimmer event stands for (see
home_assistant/bthome_buttons.yaml).
"""

import argparse
import asyncio
import time
from datetime import datetime

from bleak import BleakScanner
from bleak.backends.device import BLEDevice
from bleak.backends.scanner import AdvertisementData
from bthome_ble import BTHomeBluetoothDeviceData
from habluetooth import BluetoothServiceInfoBleak

BTHOME_UUID = "0000fcd2-0000-1000-8000-00805f9b34fb"

PRESS_TYPES = {"rotate_right": "tap", "rotate_left": "long press"}


class Monitor:
    def __init__(self, address: str | None) -> None:
        self.address = address
        # One parser per device, as Home Assistant keeps, so its duplicate
        # filter sees the same packet history.
        self.parsers: dict[str, BTHomeBluetoothDeviceData] = {}
        self.last_payloads: dict[str, bytes] = {}

    def on_advertisement(self, device: BLEDevice, adv: AdvertisementData) -> None:
        payload = adv.service_data.get(BTHOME_UUID)
        if payload is None:
            return
        if self.address and device.address.upper() != self.address:
            return

        # BlueZ repeats a device's last service data with every advert it
        # hears from it, including the remote's connectable ones. Every packet
        # has a new packet ID, so only changed data is a new packet.
        if self.last_payloads.get(device.address) == payload:
            return
        self.last_payloads[device.address] = payload

        parser = self.parsers.setdefault(device.address, BTHomeBluetoothDeviceData())
        service_info = BluetoothServiceInfoBleak(
            name=adv.local_name or device.address,
            address=device.address,
            rssi=adv.rssi,
            manufacturer_data=adv.manufacturer_data,
            service_data={BTHOME_UUID: payload},
            service_uuids=adv.service_uuids,
            source="local",
            device=device,
            advertisement=adv,
            connectable=False,
            time=time.monotonic(),
            tx_power=adv.tx_power,
        )
        update = parser.update(service_info)

        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        print(f"{stamp}  {device.address}  {update.title}  {adv.rssi} dBm  {payload.hex(' ')}")

        # The parser only records a packet it accepts. It drops one whose
        # packet ID has not moved on from the last, as Home Assistant would.
        if parser.last_service_info is not service_info:
            print("    dropped as a repeat or an older packet")
            return

        # These are the device's latest values, which a packet may only partly
        # update.
        values = []
        for key, value in update.entity_values.items():
            if key.key == "signal_strength":
                continue
            description = update.entity_descriptions.get(key)
            unit = description.native_unit_of_measurement if description else None
            values.append(f"{value.name}: {value.native_value}{f' {unit}' if unit else ''}")

        if values:
            print("    " + " | ".join(values))

        for key, event in update.events.items():
            line = f"    event {key.key}: {event.event_type}"
            if event.event_properties:
                line += ", " + ", ".join(f"{k}={v}" for k, v in event.event_properties.items())
            if key.key == "dimmer" and event.event_type in PRESS_TYPES:
                line += f"  ->  button {event.event_properties['steps']} {PRESS_TYPES[event.event_type]}"
            print(line)


async def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--address", help="only show this device's packets")
    args = parser.parse_args()

    monitor = Monitor(args.address.upper() if args.address else None)
    async with BleakScanner(monitor.on_advertisement):
        print("Listening for BTHome packets; Ctrl+C to stop.")
        await asyncio.Event().wait()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
