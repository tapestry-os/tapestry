# Leak sensor: a stationary Tapestry element

A dev board placed on the Cutebot board stands in for a fixed leak
detector. It never moves. When someone presses its button it reports a
leak, and the robots running
[`leak-sensor.choreo.toml`](../cutebot-formation/leak-sensor.choreo.toml)
form a ring around it.

## How the robots understand it

- **It gossips like any element**, with `ELEMENT_HEALTH_STATIONARY` set
  (`tapestry-os/include/tapestry/csm.h`). Robots keep a smaller keep-out
  around it than around each other, and still avoid it when its entry is
  stale, since it cannot have moved. Their auto-ID leaves it out of
  `n_total`, so start positions and the start barrier are the same with or
  without it.
- **It has a fixed ID** (`CONFIG_TAPESTRY_ELEMENT_ID`, default 31) and never
  runs auto-ID. Keep it at the top of the range, where a robot's negotiated
  ID (counting up from 0) can never reach. A second sensor takes 30.
- **It runs the robots' own Choreo.** It has no locomotion, so it lands on
  the script's catch-all track and gossips that. The ring and membership
  events on the robots' track never count it.
- **The leak is a shared fact**: a TARGET fact at the sensor's position. The
  state lives in the fact table every element holds, not in this program.
  A sensor that reboots relearns the leak from the robots.

## Buttons and LEDs

Devicetree aliases, so any board that has them works:

| | |
|---|---|
| `sw0` | Report a leak. Retried each cycle until the fact table has synced with a robot, about 1–2 s after boot (LED2 blinks meanwhile). |
| `sw1` | Clear the leak (retract the fact). The robots form a ring about the centre and wait for the next one. |
| `led0` | On: running. |
| `led1` | On: at least one robot heard. |
| `led2` | On: a leak is held in the fact table. Blinking: report pending. |

On the Apollo330M Plus EVB, `sw0` is BTN0, printed **SW3**, and `sw1` is
BTN1, printed **SW2**. SW1 is reset. SW3 is also the boot-override button:
held down through a reset or power-on, it starts the wired-update loader
instead of this app. Pressing it while the app runs only reports a leak.
At boot the console logs the GPIO pin behind each button and LED.

## Position

`CONFIG_LEAK_SENSOR_X` / `CONFIG_LEAK_SENSOR_Y`, in the robots' world units:
one chessboard square = 10, centre = (50, 50). The default (70, 70) is two
squares right and two up from the centre, on a grid crossing. The robots
ring the configured point, so put the board there. Positions about 2.5–3
squares from the centre keep the waiting robots clear of its keep-out and
the ring inside the play area.

## Build

### Apollo330M Plus EVB

Builds in the Ambiq workspace (see
`tapestry-os/boards/apollo330mP_evb/README.md`), from this checkout:

```sh
west build -b apollo330mP_evb -p always path/to/tapestry/examples/leak-sensor \
    -- -DCONFIG_LEAK_SENSOR_X=70 -DCONFIG_LEAK_SENSOR_Y=70
west flash
```

It runs on USB power or a USB power bank. The console on UART0 logs each
button press, the fact, and robots coming and going.

### Other boards

Add `boards/<board>.conf` with the BLE settings in
`boards/apollo330mP_evb.conf`: extended advertising and two advertising
sets. A board whose controller runs in the application (nRF52/nRF54L)
also needs the controller buffer sizes from
`examples/cutebot-formation/boards/bbc_microbit_v2.conf`
(`CONFIG_BT_CTLR_ADV_DATA_LEN_MAX` / `SCAN_DATA_LEN_MAX`). An nRF5340 DK
needs the network-core controller image too, as `examples/ble-observer`
builds it. None of these is done yet.

## The script

`src/choreo_script.h` is generated from the robots' script, so both sides
always run the same one:

```sh
python3 tapestry/sdk/tools/choreoc.py tapestry/examples/cutebot-formation/leak-sensor.choreo.toml \
    -o tapestry/examples/leak-sensor/src/choreo_script.h
```

Regenerate it whenever the `.toml` changes, and rebuild both.

## Starting clean

Facts never expire. A leak stays held while any element holds it, and a
robot switched on while a leak is held goes straight to the ring. Press
`sw1` before a new run, or power everything off, sensor included.
