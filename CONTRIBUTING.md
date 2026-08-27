# Contributing to Tapestry

## Prerequisites

| Tool | Version | Notes |
|---|---|---|
| [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html) | 0.17.0+ | Provides `west` and native_sim toolchain |
| [west](https://docs.zephyrproject.org/latest/develop/west/index.html) | 1.2.0+ | Installed with the Zephyr SDK |
| Python | 3.11+ | 3.11 works; step 1 installs 3.12 as preferred|
| CMake | 3.20.0+ | Bundled with the Zephyr SDK |
| ninja | any | Bundled with the Zephyr SDK |

Tested on Raspberry Pi aarch64 (Zephyr 4.4.0-rc1) and Ubuntu 22.04 x86_64.

## First-time setup

```bash
# 1. Create workspace and virtual environment
mkdir tapestry-workspace && cd tapestry-workspace
uv venv --prompt tapestry --python 3.12
source .venv/bin/activate
uv pip install west 

# 2. Initialize a west workspace with Tapestry as the manifest project
west init -m https://github.com/tapestry-os/tapestry

# 3. Fetch Zephyr and its modules
west update

# 4. Export the Zephyr CMake package (needed once per workspace)
west zephyr-export

# 5. Set up all Zephyr dependencies
uv pip install -r zephyr/scripts/requirements.txt

# 6. Set up all simulation orchestrator dependencies
uv pip install pandas matplotlib
```

Also install the 
[Zephyr SDK](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html) 
and run the setup script if it is not already on the development computer.

## Building Simulator and Testing

All commands run from the workspace root (`tapestry-workspace/`) unless
otherwise noted. Build the test suits targeting Zephyr's `native_sim` 
simulator as a native Linux executable.  

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

The L3 transport suite puts real bytes through `gossip_send` / `gossip_drain`
via a loopback transceiver, and is built three ways because the auth and
relay code paths are compiled by nothing else. The `--extra-conf` argument
is a bare filename: Zephyr resolves it against the application source
directory, which is where these overlays live.

```bash
# default framing
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-os/build/test-transport \
    tapestry/tapestry-os/tests/transport
./tapestry/tapestry-os/build/test-transport/zephyr/zephyr.exe

# HMAC-authenticated framing
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-os/build/test-transport-auth \
    --extra-conf auth.conf \
    tapestry/tapestry-os/tests/transport
./tapestry/tapestry-os/build/test-transport-auth/zephyr/zephyr.exe

# two-hop opportunistic relay
west build -b native_sim/native/64 \
    --build-dir tapestry/tapestry-os/build/test-transport-relay \
    --extra-conf relay.conf \
    tapestry/tapestry-os/tests/transport
./tapestry/tapestry-os/build/test-transport-relay/zephyr/zephyr.exe
```

See [README.md](README.md) for other simulation element build and run commands.

## Python tests

The Choreographer SDK (`sdk/python/tapestry`) and its tools (`sdk/tools`)
have their own suite, run with pytest. It needs no Zephyr, no west
workspace and no toolchain — only Python 3.11+ — and takes a couple of
seconds, so it is the fastest way to check a change before building
anything.

```bash
# From the repository root (tapestry/).  pytest and ruff are the only
# extras; the Zephyr requirements installed during setup already provide
# both, so often there is nothing to install at all:
#   pip install pytest ruff

pytest sdk/tests       # the whole suite
ruff check --config sdk/pyproject.toml .   # the linter CI runs
pytest sdk/tests/test_script_toml.py -v    # one module
```

Configuration for both tools lives in [sdk/pyproject.toml](sdk/pyproject.toml),
next to the code it governs rather than at the repository root. Both
commands need to be told where it is:

- **ruff** — pass `--config sdk/pyproject.toml`. Without it, files outside
  `sdk/` are linted with whatever rules the installed ruff version defaults
  to, and those defaults change between releases.
- **pytest** — name the test path (or run `pytest` from inside `sdk/`).
  pytest searches upward for its config, so a bare `pytest` at the
  repository root runs the suite without reading it.

The suite covers the L6 BSE geometry and achievement predicate, the L7
lifecycle/script/quorum rules, the `.choreo.toml` parser, the `choreoc`
header compiler (including that every committed `choreo_script.h` still
matches its script) and both `choreo_sim` modes.

`sdk/tests/data/` holds one committed telemetry recording so
`choreo_sim --replay` can be exercised in CI without Webots. It is
generated, not captured — regenerate it after a deliberate engine change
with:

```bash
PYTHONPATH=sdk/python python3 sdk/tests/helpers.py
```

If that changes the file, the Python engine's tick-by-tick behavior
changed; make sure that was intended, and that
`tapestry-os/subsys/bse/bse.c` and `choreo.c` changed to match. The
recording only proves the Python mirror is self-consistent — cross-language
parity against the C engine needs a real capture, see
[sdk/CHOREO_SCRIPTS.md](sdk/CHOREO_SCRIPTS.md).

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
- The SDK and `sdk/tools` are standard library ONLY — they are run from a
  checkout with nothing installed, and `matplotlib` may only be imported
  lazily, inside the plotting function that needs it.
- `ruff check --config sdk/pyproject.toml .` must be clean. The rule set
  (`sdk/pyproject.toml`) is
  deliberately narrow — pyflakes plus the bugbear traps, no style
  rewriting — so anything it reports is a real defect, not a preference.

**Tests**

Every new C API function needs at least one ztest, in
`tapestry-<layer>-sim/tests/` for the relevant layer. L3 transport has no
sim component, so its suite lives beside the code it tests, in
`tapestry-os/tests/transport/`. Every change to the Python SDK or its tools
needs a test in `sdk/tests/`.
CI runs the Python suite and all ztests on every push and PR — a failing
test blocks merge.

**Test the whole chain, not each hop**

A value that crosses layers — computed at L6, published by the app,
serialised by L3, unpacked into the world model, then read back by L7 —
needs at least one test that follows it end to end. Every hop of the
gossiped `achieved` bit had been inspected in isolation and looked correct;
it was still broken twice, in two different hops, because nothing tested
the chain. Prefer a loopback (see `tapestry-os/tests/transport/`) over a
mock whenever the thing you would mock is the thing that broke.

## Submitting changes

1. Fork the repository and create a feature branch from `main`.
2. Make your changes. Run `pytest` and both ztest suites locally before
   pushing.
3. Open a pull request against `main`. Describe what changed and why.
4. CI must pass. A maintainer will review and merge.

For significant changes (new layers, protocol changes, API additions) open
an issue first to discuss the design before writing code.
