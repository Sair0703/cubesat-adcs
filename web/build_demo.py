"""Embeds the compiled WebAssembly module into the demo page as base64,
so docs/index.html is a single self-contained file for GitHub Pages."""
import base64, pathlib, sys

wasm, template, out = map(pathlib.Path, sys.argv[1:4])
html = template.read_text().replace("__WASM_BASE64__", base64.b64encode(wasm.read_bytes()).decode())
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(html)
print(f"wrote {out} ({out.stat().st_size // 1024} KB, wasm {wasm.stat().st_size} bytes)")
