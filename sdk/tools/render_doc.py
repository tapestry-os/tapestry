#!/usr/bin/env python3
"""Render a repo Markdown doc to styled HTML and PDF.

    python3 sdk/tools/render_doc.py sdk/CHOREO_AUTHORING.md

Writes <name>.html and <name>.pdf next to the source file (or to -o/--outdir).
HTML rendering uses the stdlib-adjacent `markdown` package (GFM-ish: tables,
fenced code, syntax highlighting, a generated TOC). PDF rendering shells out
to a local Chrome/Chromium in headless print-to-pdf mode -- there is no
Python-only PDF path installed in this environment (no weasyprint/wkhtmltopdf),
and Chrome's print engine handles the CSS below (page breaks, print margins)
more faithfully than the lighter-weight alternatives.

Requires: pip install --user markdown pygments (one-time; see README/CI docs
if this needs to run somewhere without them). Chrome or Chromium must be
installed; searched in common macOS/Linux locations, override with
--chrome /path/to/chrome if not found automatically.
"""
import argparse
import datetime
import pathlib
import shutil
import subprocess
import sys

try:
    import markdown
except ImportError:
    sys.exit(
        "error: the 'markdown' package is required -- "
        "install with: python3 -m pip install --user markdown pygments"
    )

CSS = """
:root {
  --text: #1c1e21; --text-dim: #5b616b; --border: #e2e4e9;
  --bg-code: #f1f2f5; --accent: #7c3aed; --code: #b2298a;
}
@page { margin: 20mm 16mm; }
body {
  font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Helvetica, Arial, sans-serif;
  color: var(--text); line-height: 1.55; font-size: 13px; max-width: 820px;
  margin: 0 auto; padding: 0 8px;
}
h1, h2, h3, h4 { line-height: 1.25; break-after: avoid; }
h1 { font-size: 1.9em; border-bottom: 2px solid var(--border); padding-bottom: 10px; }
h2 { font-size: 1.4em; border-bottom: 1px solid var(--border); padding-bottom: 6px; margin-top: 2em; }
h3 { font-size: 1.15em; margin-top: 1.6em; }
h4 { font-size: 1em; margin-top: 1.2em; color: var(--text-dim); }
p, ul, ol { margin: 0 0 12px; }
li { margin-bottom: 3px; }
a { color: var(--accent); }
code {
  font-family: ui-monospace, SFMono-Regular, "SF Mono", Menlo, Consolas, monospace;
  background: var(--bg-code); color: var(--code);
  padding: 0.08em 0.35em; border-radius: 4px; font-size: 0.88em;
}
pre {
  background: var(--bg-code); border: 1px solid var(--border); border-radius: 8px;
  padding: 12px 14px; overflow-x: auto; break-inside: avoid; font-size: 0.85em;
}
pre code { background: none; color: inherit; padding: 0; }
table { width: 100%; border-collapse: collapse; margin: 0 0 16px; font-size: 0.9em; break-inside: avoid; }
th, td { text-align: left; padding: 6px 10px; border-bottom: 1px solid var(--border); vertical-align: top; }
th { color: var(--text-dim); font-size: 0.85em; text-transform: uppercase; }
blockquote { border-left: 4px solid var(--accent); margin: 0 0 14px; padding: 4px 14px; color: var(--text-dim); }
hr { border: none; border-top: 1px solid var(--border); margin: 2em 0; }
.codehilite { background: var(--bg-code); border-radius: 8px; }
.render-meta {
  color: var(--text-dim); font-size: 0.8em; break-inside: avoid;
}
.render-meta code { font-size: 1em; }
header.render-meta {
  margin-bottom: 1.5em; padding-bottom: 0.75em; border-bottom: 1px solid var(--border);
}
footer.render-meta {
  margin-top: 2.5em; padding-top: 1em; border-top: 1px solid var(--border);
}
"""

RENDER_META = ("Tapestry &middot; generated {date} from commit "
               "<code>{commit}</code> &middot; &copy; 2026 James V Steele "
               "&middot; Apache 2.0 License")

HTML_TEMPLATE = """<!doctype html>
<html><head><meta charset="utf-8"><title>{title}</title>
<style>{css}</style></head><body>
<header class="render-meta">""" + RENDER_META + """</header>
{body}
<footer class="render-meta">""" + RENDER_META + """</footer>
</body></html>
"""

CHROME_CANDIDATES = [
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/Applications/Chromium.app/Contents/MacOS/Chromium",
    "google-chrome", "google-chrome-stable", "chromium", "chromium-browser",
]


def get_commit(md_path: pathlib.Path) -> str:
    """Short HEAD commit for md_path's repo, plus a dirty marker if md_path
    itself has uncommitted changes. Falls back to 'unversioned' outside git."""
    try:
        repo_dir = md_path.resolve().parent
        commit = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"],
            cwd=repo_dir, capture_output=True, text=True, check=True,
        ).stdout.strip()
        dirty = subprocess.run(
            ["git", "diff", "--quiet", "HEAD", "--", str(md_path.resolve())],
            cwd=repo_dir,
        ).returncode != 0
        return f"{commit}+uncommitted" if dirty else commit
    except (subprocess.CalledProcessError, FileNotFoundError):
        return "unversioned"


def find_chrome(explicit: str | None) -> str:
    if explicit:
        return explicit
    for candidate in CHROME_CANDIDATES:
        if pathlib.Path(candidate).exists() or shutil.which(candidate):
            return candidate
    sys.exit(
        "error: no Chrome/Chromium found -- pass --chrome /path/to/chrome, "
        "or skip PDF generation with --html-only"
    )


def render(md_path: pathlib.Path, outdir: pathlib.Path, chrome: str | None, html_only: bool):
    text = md_path.read_text()
    body = markdown.markdown(
        text,
        extensions=["extra", "toc", "sane_lists", "codehilite"],
        extension_configs={"codehilite": {"guess_lang": False}},
    )
    html = HTML_TEMPLATE.format(
        title=md_path.stem, css=CSS, body=body,
        date=datetime.date.today().isoformat(), commit=get_commit(md_path),
    )

    outdir.mkdir(parents=True, exist_ok=True)
    html_path = outdir / f"{md_path.stem}.html"
    html_path.write_text(html)
    print(f"wrote {html_path}")

    if html_only:
        return

    chrome_bin = find_chrome(chrome)
    pdf_path = outdir / f"{md_path.stem}.pdf"
    subprocess.run(
        [
            chrome_bin, "--headless", "--disable-gpu",
            f"--print-to-pdf={pdf_path}",
            "--no-pdf-header-footer",
            f"file://{html_path.resolve()}",
        ],
        check=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    print(f"wrote {pdf_path}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("markdown_file", type=pathlib.Path)
    p.add_argument("-o", "--outdir", type=pathlib.Path, default=None,
                   help="output directory (default: alongside the source file)")
    p.add_argument("--chrome", default=None, help="path to Chrome/Chromium binary")
    p.add_argument("--html-only", action="store_true", help="skip PDF, just write HTML")
    args = p.parse_args()

    if not args.markdown_file.exists():
        sys.exit(f"error: {args.markdown_file} not found")

    outdir = args.outdir or args.markdown_file.parent
    render(args.markdown_file, outdir, args.chrome, args.html_only)


if __name__ == "__main__":
    main()
