from pyfbsdk import *
fm = FBFilterManager()
out = []
for n in fm.FilterTypeNames:
    f = fm.CreateFilter(n)
    if f is None:
        out.append("%s: none" % n); continue
    ps = []
    for p in f.PropertyList:
        try: d = p.Data
        except Exception: d = "?"
        ps.append("%s=%s" % (p.Name, d))
    out.append("%s: %s" % (n, "; ".join(ps)))
    f.FBDelete()
open(r"C:/Projects/Fulcrum/re/mobu/probes/filters.txt", "w").write("\n".join(out))
print("ok")
