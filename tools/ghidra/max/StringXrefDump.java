import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Data;
import ghidra.program.model.listing.Function;
import ghidra.program.model.symbol.Reference;
import java.io.File;
import java.io.PrintWriter;
import java.util.LinkedHashSet;
import java.util.Set;
import java.util.regex.Pattern;

public class StringXrefDump extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        String rx = args[0].startsWith("@") ? new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(args[0].substring(1))), "UTF-8").trim() : args[0]; Pattern pat = Pattern.compile(rx, Pattern.CASE_INSENSITIVE);
        int maxFns = Integer.parseInt(args[2]);
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        Set<Function> fns = new LinkedHashSet<>();
        try (PrintWriter w = new PrintWriter(new File(args[1]), "UTF-8")) {
            w.println("== " + currentProgram.getName() + " strings matching " + args[0]);
            for (Data d : currentProgram.getListing().getDefinedData(true)) {
                if (!d.hasStringValue()) continue;
                String v = String.valueOf(d.getValue());
                if (!pat.matcher(v).find()) continue;
                StringBuilder users = new StringBuilder();
                for (Reference r : getReferencesTo(d.getAddress())) {
                    Function f = getFunctionContaining(r.getFromAddress());
                    if (f != null) { fns.add(f); users.append(" ").append(f.getName()).append("@").append(f.getEntryPoint()); }
                }
                w.println(d.getAddress() + " \"" + v.replace("\n", "\\n") + "\" <-" + users);
            }
            int n = 0;
            for (Function f : fns) {
                if (n++ >= maxFns) break;
                long size = f.getBody().getNumAddresses();
                if (size > 20000) { w.println("---- " + f.getName() + " @ " + f.getEntryPoint() + " size=" + size + " (skipped, too large)"); continue; }
                DecompileResults r = di.decompileFunction(f, 180, monitor);
                w.println("---- " + f.getName() + " @ " + f.getEntryPoint() + " size=" + size);
                w.println(r != null && r.decompileCompleted() ? r.getDecompiledFunction().getC() : "// decompile failed");
                w.flush();
            }
        }
        println("StringXrefDump wrote " + args[1] + " functions=" + fns.size());
    }
}
