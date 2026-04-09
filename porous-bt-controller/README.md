# Porous Bluetooth Controller

Minimal terminal bridge for the Porous BLE control interface.

## What It Does

- connects to the `Porous` BLE device
- prints BLE log output while keeping a stable command prompt
- sends each typed line to the ESP unchanged when you press Enter
- reconnects cleanly if BLE drops

The command set is the same as USB serial:

- `help`
- `show`
- `save`
- `load`
- `set gain 0.3`
- `set min_on_ms 3000`

Controller-only commands:

- `/reconnect`
- `/quit`

## Setup

```bash
cd /Users/joeyverbeke/Documents/_projects/3_subliminal-earworm/0_porous/Porous/porous-bt-controller
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

## Run

```bash
./.venv/bin/python porous_bt_controller.py
```

Optional flags:

```bash
./.venv/bin/python porous_bt_controller.py --name Porous --scan-timeout 10
```

If you change the BLE UUIDs in the firmware, pass the matching UUIDs here with `--service-uuid`, `--tx-uuid`, and `--rx-uuid`.
