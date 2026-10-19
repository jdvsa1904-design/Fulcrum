import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Symbol;
import java.io.File;
import java.io.PrintWriter;
import java.util.HashSet;
import java.util.Set;
import java.util.regex.Pattern;

public class VtableDump extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        String rx = new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(args[0])), "UTF-8").trim();
        Pattern pat = Pattern.compile(rx);
        int slots = Integer.parseInt(args[2]);
        int minSize = Integer.parseInt(args[3]);
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        Set<Address> done = new HashSet<>();
        try (PrintWriter w = new PrintWriter(new File(args[1]), "UTF-8")) {
            for (Symbol s : currentProgram.getSymbolTable().getAllSymbols(true)) {
                if (!s.getName().equals("vftable")) continue;
                String cls = s.getParentNamespace().getName(true);
                if (!pat.matcher(cls).matches()) continue;
                w.println("==== " + cls + " vftable @ " + s.getAddress());
                Address a = s.getAddress();
                for (int i = 0; i < slots; i++) {
                    long p = getLong(a.add(i * 8L));
                    Address fa = toAddr(p);
                    Function f = getFunctionAt(fa);
                    if (f == null) { w.println("slot " + i + " -> " + fa + " (no fn)"); if (i > 4) break; continue; }
                    long size = f.getBody().getNumAddresses();
                    w.println("slot " + i + " -> " + f.getName() + " @ " + fa + " size=" + size);
                    if (size < minSize || size > 30000 || done.contains(fa)) continue;
                    done.add(fa);
                    DecompileResults r = di.decompileFunction(f, 240, monitor);
                    w.println("---- " + cls + "[" + i + "] " + f.getName() + " @ " + fa);
                    w.println(r != null && r.decompileCompleted() ? r.getDecompiledFunction().getC() : "// decompile failed");
                    w.flush();
                }
            }
        }
    }
}
