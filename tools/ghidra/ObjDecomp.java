import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileOptions;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.util.demangler.DemanglerUtil;
import ghidra.app.util.demangler.DemangledObject;
import ghidra.program.model.listing.Function;
import java.io.File;
import java.io.PrintWriter;

public class ObjDecomp extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        File out = new File(args[0], currentProgram.getName() + ".c");
        int minSize = args.length > 1 ? Integer.parseInt(args[1]) : 24;
        DecompInterface di = new DecompInterface();
        DecompileOptions opt = new DecompileOptions();
        di.setOptions(opt);
        di.openProgram(currentProgram);
        try (PrintWriter w = new PrintWriter(out, "UTF-8")) {
            for (Function f : currentProgram.getFunctionManager().getFunctions(true)) {
                if (f.isThunk() || f.isExternal()) continue;
                long size = f.getBody().getNumAddresses();
                if (size < minSize) continue;
                String raw = f.getName();
                String pretty = raw;
                try {
                    String mangled = f.getSymbol().getName();
                    for (ghidra.program.model.symbol.Symbol s : currentProgram.getSymbolTable().getSymbols(f.getEntryPoint())) {
                        if (s.getName().startsWith("?")) mangled = s.getName();
                    }
                    java.util.List<DemangledObject> d = DemanglerUtil.demangle(currentProgram, mangled, f.getEntryPoint());
                    if (d != null && !d.isEmpty()) pretty = d.get(0).getSignature(false);
                    raw = mangled;
                } catch (Exception e) {
                }
                String ns = f.getParentNamespace() == null ? "" : f.getParentNamespace().getName(true);
                if (pretty.contains("cg_") || pretty.contains("XYZ") || pretty.contains("lhs") || pretty.contains("rhs")) continue;
                DecompileResults r = di.decompileFunction(f, 120, monitor);
                w.println("//==== " + ns + "::" + f.getName() + " @ " + f.getEntryPoint() + " size=" + size);
                w.println("//     " + pretty);
                if (r != null && r.decompileCompleted()) w.println(r.getDecompiledFunction().getC());
                else w.println("//     decompile failed");
            }
        }
        println("dumped " + out);
    }
}
