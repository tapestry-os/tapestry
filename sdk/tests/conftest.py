"""
conftest.py — import paths for the Tapestry Python test suite.

The SDK is deliberately not installable (no venv, nothing to install —
see sdk/tools/choreoc.py's module docstring), so the tests reach the
package and the CLI tools the same way the tools reach each other: by
putting the two source directories on sys.path.  `tests/` itself goes on
too, so `helpers.py` imports cleanly regardless of pytest's rootdir.
"""

import sys
from pathlib import Path

TESTS_DIR = Path(__file__).resolve().parent
SDK_DIR   = TESTS_DIR.parent
REPO_ROOT = SDK_DIR.parent

for p in (TESTS_DIR, SDK_DIR / "python", SDK_DIR / "tools"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
