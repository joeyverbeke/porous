#!/usr/bin/env python3

import argparse
import asyncio
from contextlib import suppress
from dataclasses import dataclass

from bleak import BleakClient, BleakScanner
from bleak.exc import BleakError
from prompt_toolkit import PromptSession
from prompt_toolkit.patch_stdout import patch_stdout


DEFAULT_DEVICE_NAME = "Porous"
DEFAULT_SERVICE_UUID = "64dbf000-3c92-4ca7-b9f0-d5f4d7f25b10"
DEFAULT_TX_UUID = "64dbf001-3c92-4ca7-b9f0-d5f4d7f25b10"
DEFAULT_RX_UUID = "64dbf002-3c92-4ca7-b9f0-d5f4d7f25b10"
DEFAULT_CONNECT_RETRIES = 3
DEFAULT_RECONNECT_DELAY = 1.0


class LineBuffer:
    def __init__(self, emit_line) -> None:
        self._buffer = bytearray()
        self._emit_line = emit_line

    def feed(self, data: bytearray) -> None:
        self._buffer.extend(data)
        while True:
            newline_index = self._buffer.find(b"\n")
            if newline_index < 0:
                return

            raw_line = self._buffer[:newline_index]
            del self._buffer[: newline_index + 1]
            line = raw_line.rstrip(b"\r").decode("utf-8", errors="replace")
            self._emit_line(line)

    def flush(self) -> None:
        if not self._buffer:
            return
        line = self._buffer.rstrip(b"\r").decode("utf-8", errors="replace")
        self._buffer.clear()
        if line:
            self._emit_line(line)


@dataclass
class ControllerState:
    client: BleakClient | None = None

    def __post_init__(self) -> None:
        self.client_lock = asyncio.Lock()
        self.connected_event = asyncio.Event()
        self.shutdown_event = asyncio.Event()
        self.reconnect_event = asyncio.Event()

    async def set_client(self, client: BleakClient | None) -> None:
        async with self.client_lock:
            self.client = client
            if client is None:
                self.connected_event.clear()
            else:
                self.connected_event.set()

    async def send_command(self, rx_uuid: str, command: str) -> bool:
        async with self.client_lock:
            client = self.client

        if client is None or not client.is_connected:
            return False

        payload = command if command.endswith("\n") else f"{command}\n"
        await client.write_gatt_char(rx_uuid, payload.encode("utf-8"), response=False)
        return True


async def log_message(log_queue: asyncio.Queue[str], message: str) -> None:
    await log_queue.put(message)


async def find_device(name: str, timeout: float):
    devices = await BleakScanner.discover(timeout=timeout)
    for device in devices:
        if device.name == name:
            return device
    return None


async def display_loop(log_queue: asyncio.Queue[str], shutdown_event: asyncio.Event) -> None:
    while True:
        if shutdown_event.is_set() and log_queue.empty():
            return

        get_task = asyncio.create_task(log_queue.get())
        shutdown_task = asyncio.create_task(shutdown_event.wait())
        done, pending = await asyncio.wait(
            {get_task, shutdown_task},
            return_when=asyncio.FIRST_COMPLETED,
        )

        for task in pending:
            task.cancel()
        await asyncio.gather(*pending, return_exceptions=True)

        if get_task in done:
            print(get_task.result(), flush=True)
            continue

        if shutdown_event.is_set() and log_queue.empty():
            return


async def input_loop(
    session: PromptSession,
    state: ControllerState,
    args: argparse.Namespace,
    log_queue: asyncio.Queue[str],
) -> None:
    while not state.shutdown_event.is_set():
        try:
            line = await session.prompt_async("porous> ")
        except EOFError:
            state.shutdown_event.set()
            return
        except KeyboardInterrupt:
            await log_message(log_queue, "Use /quit to exit.")
            continue

        command = line.strip()
        if not command:
            continue

        if command == "/quit":
            state.shutdown_event.set()
            return

        if command == "/reconnect":
            state.reconnect_event.set()
            continue

        try:
            sent = await state.send_command(args.rx_uuid, command)
        except BleakError as exc:
            await log_message(log_queue, f"Send failed: {exc}")
            state.reconnect_event.set()
            continue

        if not sent:
            await log_message(log_queue, "Not connected. Command not sent.")


async def connection_loop(
    state: ControllerState,
    args: argparse.Namespace,
    log_queue: asyncio.Queue[str],
) -> None:
    loop = asyncio.get_running_loop()

    while not state.shutdown_event.is_set():
        device = await find_device(args.name, args.scan_timeout)
        if device is None:
            await log_message(log_queue, f"Did not find BLE device named '{args.name}'. Retrying...")
            await asyncio.sleep(args.reconnect_delay)
            continue

        for attempt in range(1, args.connect_retries + 1):
            if state.shutdown_event.is_set():
                return

            await log_message(
                log_queue,
                f"Connecting to {device.name} ({device.address})... attempt {attempt}/{args.connect_retries}",
            )

            disconnect_event = asyncio.Event()
            line_buffer = LineBuffer(lambda line: loop.call_soon_threadsafe(log_queue.put_nowait, line))

            def handle_disconnect(_: BleakClient) -> None:
                loop.call_soon_threadsafe(disconnect_event.set)

            def handle_notification(_: str, data: bytearray) -> None:
                line_buffer.feed(data)

            try:
                async with BleakClient(device, disconnected_callback=handle_disconnect) as client:
                    await client.start_notify(args.tx_uuid, handle_notification)
                    await state.set_client(client)
                    state.reconnect_event.clear()
                    await log_message(
                        log_queue,
                        "Connected. Type Porous commands and press Enter. Use /reconnect or /quit if needed.",
                    )

                    disconnect_task = asyncio.create_task(disconnect_event.wait())
                    reconnect_task = asyncio.create_task(state.reconnect_event.wait())
                    shutdown_task = asyncio.create_task(state.shutdown_event.wait())

                    done, pending = await asyncio.wait(
                        {disconnect_task, reconnect_task, shutdown_task},
                        return_when=asyncio.FIRST_COMPLETED,
                    )

                    for task in pending:
                        task.cancel()
                    await asyncio.gather(*pending, return_exceptions=True)

                    if reconnect_task in done and state.reconnect_event.is_set():
                        await log_message(log_queue, "Reconnecting...")
                    elif disconnect_task in done and disconnect_event.is_set():
                        await log_message(log_queue, "Disconnected.")

                    if client.is_connected:
                        with suppress(BleakError, AttributeError):
                            await client.stop_notify(args.tx_uuid)

                line_buffer.flush()
                await state.set_client(None)
                break
            except BleakError as exc:
                await state.set_client(None)
                if attempt == args.connect_retries:
                    await log_message(log_queue, f"Connect failed: {exc}. Rescanning...")
                else:
                    await log_message(log_queue, f"Connect failed: {exc}. Retrying...")
                await asyncio.sleep(args.reconnect_delay)

        state.reconnect_event.clear()
        if not state.shutdown_event.is_set():
            await asyncio.sleep(args.reconnect_delay)


async def run(args: argparse.Namespace) -> int:
    state = ControllerState()
    log_queue: asyncio.Queue[str] = asyncio.Queue()
    session = PromptSession()

    display_task = asyncio.create_task(display_loop(log_queue, state.shutdown_event))
    connection_task = asyncio.create_task(connection_loop(state, args, log_queue))

    try:
        with patch_stdout():
            await input_loop(session, state, args, log_queue)
    finally:
        state.shutdown_event.set()
        connection_task.cancel()
        with suppress(asyncio.CancelledError):
            await connection_task
        await display_task

    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Connect to the Porous ESP over BLE, show log output, and forward typed commands."
    )
    parser.add_argument("--name", default=DEFAULT_DEVICE_NAME, help="BLE device name to scan for.")
    parser.add_argument(
        "--scan-timeout",
        type=float,
        default=8.0,
        help="Seconds to scan before giving up.",
    )
    parser.add_argument("--service-uuid", default=DEFAULT_SERVICE_UUID, help="BLE service UUID.")
    parser.add_argument("--tx-uuid", default=DEFAULT_TX_UUID, help="Notify characteristic UUID.")
    parser.add_argument("--rx-uuid", default=DEFAULT_RX_UUID, help="Write characteristic UUID.")
    parser.add_argument(
        "--connect-retries",
        type=int,
        default=DEFAULT_CONNECT_RETRIES,
        help="Connection attempts before rescanning.",
    )
    parser.add_argument(
        "--reconnect-delay",
        type=float,
        default=DEFAULT_RECONNECT_DELAY,
        help="Seconds to wait before retrying BLE operations.",
    )
    return parser.parse_args()


def main() -> int:
    try:
        return asyncio.run(run(parse_args()))
    except KeyboardInterrupt:
        print("\nStopped.", flush=True)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
