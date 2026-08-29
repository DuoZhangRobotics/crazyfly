# Legacy direct-radio tools

These scripts predate the ROS 2/OptiTrack safety gateway. They talk directly to
`cflib` and can therefore bypass every production geofence, tracking-loss, and
separation check. They are retained only for controlled bring-up and historical
reference.

| Tool | Function | Risk |
| --- | --- | --- |
| `cf_diagnostics.py` | Read battery, attitude, decks, and `sys.canfly` | Read-only |
| `inspect_usb_radio.py` | Read Crazyradio configuration EEPROM | Read-only |
| `quick_radio_probe.py` | Probe for acknowledgements | Radio only |
| `flow_hover_test.py` | Brief hover requiring a detected Flow deck | Flight |
| `motor_spin_test.py` | Telemetry-guarded below-liftoff motor test | Motor |
| `quick_motor_spin.py` | Direct motor command with limited checks | High risk |
| `brief_hop_test.py` | Open-loop thrust pulse | High risk |
| `quick_one_second_hop.py` | Open-loop one-second hop | Do not use indoors |

The previous dependency lock is retained as `uv.lock.legacy` for audit history;
it is not an active root-project lock.

They are not installed as ROS executables. If a read-only bring-up tool is
needed later, install its pinned legacy dependency in an isolated environment
and run it explicitly, for example:

```sh
uv run --with cflib==0.1.33 python tools/legacy/cf_diagnostics.py
```

Do not use any motor or flight script as a Crazyswarm2 acceptance test. Start
physical validation with propellers removed and use the gateway workflow in the
main README.
