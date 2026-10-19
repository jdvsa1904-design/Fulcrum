import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import java.io.File;
import java.io.PrintWriter;
import java.util.LinkedHashMap;
import java.util.Map;

public class RefXref extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        String[] addrs = new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(args[0])), "UTF-8").trim().split("\s+");
        int maxFns = Integer.parseInt(args[2]);
        int maxSize = Integer.parseInt(args[3]);
        Map<Function, String> hits = new LinkedHashMap<>();
        for (String s : addrs) {
            Address a = toAddr(Long.parseUnsignedLong(s, 16));
            for (Reference r : getReferencesTo(a)) {
                Function f = getFunctionContaining(r.getFromAddress());
                if (f != null) hits.merge(f, s + "@" + r.getFromAddress() + " ", String::concat);
            }
        }
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        try (PrintWriter w = new PrintWriter(new File(args[1]), "UTF-8")) {
            for (Map.Entry<Function, String> e : hits.entrySet())
                w.println("HIT " + e.getKey().getName() + " @ " + e.getKey().getEntryPoint() + " size=" + e.getKey().getBody().getNumAddresses() + " : " + e.getValue());
            int n = 0;
            for (Function f : hits.keySet()) {
                long size = f.getBody().getNumAddresses();
                if (size > maxSize) continue;
                if (n++ >= maxFns) break;
                DecompileResults r = di.decompileFunction(f, 240, monitor);
                w.println("---- " + f.getName() + " @ " + f.getEntryPoint() + " size=" + size);
                w.println(r != null && r.decompileCompleted() ? r.getDecompiledFunction().getC() : "// decompile failed");
                w.flush();
            }
        }
    }
}
