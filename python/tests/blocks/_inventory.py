"""Inventory: every block the API exposes, with role, pins and parameters."""
import simpel as sp

rows = []
for cat in sp.categories():
    for t in sp.blocks(cat):
        bi = sp.block_info(t)
        pins = " ".join(p.name for p in bi.pins)
        params = " ".join(f"{p.name}={p.default}" for p in bi.params)
        rows.append(f"{cat}\t{t}\t{bi.role}\tsim={bi.simulated}\tpins[{pins}]\tparams[{params}]")
with open("inventory.tsv", "w", encoding="utf-8") as f:
    f.write("\n".join(rows) + "\n")
print(len(rows), "blocks")
