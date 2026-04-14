# Contributing to Tapestry

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html) | 0.17.0+ | Provides `west` and native_sim toolchain |
| [west](https://docs.zephyrproject.org/latest/develop/west/index.html) | 1.2.0+ | Installed with the Zephyr SDK |
| Python | 3.11+ | For the simulation orchestrators |
| CMake | 3.20.0+ | Bundled with the Zephyr SDK |
| ninja | any | Bundled with the Zephyr SDK |

Tested on Raspberry Pi aarch64 (Zephyr 4.4.0-rc1) and Ubuntu 22.04 x86_64.

## First-time setup

```bash
# 1. Initialise a west workspace with Tapestry as the manifest project
west init -m https://github.com/tapestry-os/tapestry --mr main tapestry-workspace
cd tapestry-workspace

# 2. Fetch Zephyr and its modules
west update

# 3. Export the Zephyr CMake package (needed once per workspace)
west zephyr-export

# 4. Set up the Python virtual environment for the simulation orchestrators
cd tapestry
python3 -m venv .venv
source .venv/bin/activate
pip install pandas matplotlib
```

## Building and testing

All commands run from the workspace root (`tapestry-workspace/`) unless
otherwise noted.

```bash
# L4 unit tests
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-csm-sim/build/test-csm \
    tapestry/tapestry-csm-sim/tests
./tapestry/tapestry-csm-sim/build/test-csm/zephyr/zephyr.exe

# L5 unit tests
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-scr-sim/build/tests \
    tapestry/tapestry-scr-sim/tests
./tapestry/tapestry-scr-sim/build/tests/zephyr/zephyr.exe
```

See [README.md](README.md) for simulation element build and run commands.

## Code conventions

**Architectural boundary**

The single most important invariant: no OS-specific types may cross a public
header boundary. `tapestry-os/include/tapestry/` contains the only headers
that consumers should include. Everything in `tapestry-os/subsys/` is internal.

- `#include <tapestry/csm.h>` — L4 surface
- `#include <tapestry/scr.h>` — L4 + L5 surface

Neither header may include any Zephyr header. Verify with:

```bash
grep -r "zephyr" tapestry-os/include/
```

This should return nothing.

**Adding a new layer**

New layers follow the same pattern as L4 and L5:

```
tapestry-os/subsys/<layer>/
    <layer>.h       internal types + API
    <layer>.c       pure C99 implementation (no OS deps)

tapestry-os/include/tapestry/
    <layer>.h       public boundary — includes the layer above and subsys header

tapestry-<layer>-sim/
    tests/          ztest suite (must pass before merge)
    orchestrator/   Python asyncio sim harness
    zephyr/element/ Zephyr native_sim element
```

**C style**

- C99. No C11 or compiler extensions in `tapestry-os/`.
- No dynamic allocation. All data structures are fixed-size and caller-allocated.
- No OS-specific types (`k_mutex`, `osThreadId`, etc.) in `tapestry-os/`.
- Public API functions are prefixed with the subsystem name (`wm_`, `scr_`).

**Python style**

- Follows the style of the existing orchestrator files.
- No external dependencies beyond the standard library, `pandas`, and `matplotlib`.

**Tests**

Every new API function needs at least one ztest. Tests live in
`tapestry-<layer>-sim/tests/` for the relevant layer.
CI runs all ztests on every push and PR — a failing test blocks merge.

## Submitting changes

1. Fork the repository and create a feature branch from `main`.
2. Make your changes. Run both ztest suites locally before pushing.
3. Open a pull request against `main`. Describe what changed and why.
4. CI must pass. A maintainer will review and merge.

For significant changes (new layers, protocol changes, API additions) open
an issue first to discuss the design before writing code.
