import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayDeque;
import java.util.HashSet;
import java.util.Set;

public class AddrDump extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        String[] addrs = new String(java.nio.file.Files.readAllBytes(java.nio.file.Paths.get(args[0])), "UTF-8").trim().split("\\s+");
        int depth = Integer.parseInt(args[2]);
        int maxSize = Integer.parseInt(args[3]);
        DecompInterface di = new DecompInterface();
        di.openProgram(currentProgram);
        Set<Address> done = new HashSet<>();
        ArrayDeque<Object[]> q = new ArrayDeque<>();
        for (String s : addrs) q.add(new Object[]{toAddr(Long.parseUnsignedLong(s, 16)), 0});
        try (PrintWriter w = new PrintWriter(new File(args[1]), "UTF-8")) {
            while (!q.isEmpty()) {
                Object[] it = q.poll();
                Address a = (Address) it[0];
                int d = (Integer) it[1];
                Function f = getFunctionContaining(a);
                if (f == null || done.contains(f.getEntryPoint())) continue;
                done.add(f.getEntryPoint());
                long size = f.getBody().getNumAddresses();
                if (d > 0 && (size > maxSize || size < 40 || f.isThunk() || f.isExternal())) continue;
                DecompileResults r = di.decompileFunction(f, 240, monitor);
                w.println("---- d" + d + " " + f.getName() + " @ " + f.getEntryPoint() + " size=" + size);
                w.println(r != null && r.decompileCompleted() ? r.getDecompiledFunction().getC() : "// decompile failed");
                w.flush();
                if (d < depth) for (Function c : f.getCalledFunctions(monitor)) q.add(new Object[]{c.getEntryPoint(), d + 1});
            }
        }
    }
}
