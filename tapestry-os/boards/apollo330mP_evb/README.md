# Apollo330M Plus EVB (`apollo330mP_evb`)

A Tapestry element over BLE (on-die controller) and the wired serial bus,
sharing its LED and button substrate with the Apollo510 EVB. Build settings
are in `tapestry-scr-hw/boards/apollo330mP_evb.conf`.

The board is in Ambiq's Zephyr distribution, not yet in upstream Zephyr, so
it builds in its own workspace from the `west.yml` in this folder:

```sh
west init -m https://github.com/tapestry-os/tapestry \
          --mf tapestry-os/boards/apollo330mP_evb/west.yml tapestry-ambiq-workspace
cd tapestry-ambiq-workspace && west update && west zephyr-export
west build -b apollo330mP_evb tapestry/tapestry-scr-hw -- -DCONFIG_TAPESTRY_ELEMENT_ID=1
```

Keep it separate from your upstream workspace, so boards validated on
upstream Zephyr stay on that tree. Once the board is upstream, this folder's
`west.yml` and the separate workspace go away.
