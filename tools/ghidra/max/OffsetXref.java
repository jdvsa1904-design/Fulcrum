import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.scalar.Scalar;
import java.io.File;
import java.io.PrintWriter;
import java.util.LinkedHashMap;
import java.util.Map;

public class OffsetXref extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        String[] offs = new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(args[0])), "UTF-8").trim().split("\s+");
        long[] want = new long[offs.length];
        for (int i = 0; i < offs.length; i++) want[i] = Long.parseLong(offs[i], 16);
        int maxFns = Integer.parseInt(args[2]);
        int maxSize = Integer.parseInt(args[3]);
        Map<Function, String> hits = new LinkedHashMap<>();
        for (Instruction ins : currentProgram.getListing().getInstructions(true)) {
            for (int op = 0; op < ins.getNumOperands(); op++) {
                for (Object o : ins.getOpObjects(op)) {
                    if (!(o instanceof Scalar)) continue;
                    long v = ((Scalar) o).getUnsignedValue();
                    for (long w : want) if (v == w) {
                        Function f = getFunctionContaining(ins.getAddress());
                        if (f != null) hits.merge(f, Long.toHexString(w) + "@" + ins.getAddress() + " ", String::concat);
                    }
                }
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
