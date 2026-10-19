import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Data;
import ghidra.program.model.symbol.Symbol;
import ghidra.program.model.symbol.SymbolType;
import java.io.File;
import java.io.PrintWriter;

public class DumpTool extends GhidraScript {
    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        File out = new File(args[0], currentProgram.getName() + ".txt");
        try (PrintWriter w = new PrintWriter(out, "UTF-8")) {
            w.println("== IMPORTS");
            for (Symbol s : currentProgram.getSymbolTable().getExternalSymbols()) {
                w.println(s.getParentNamespace().getName() + "::" + s.getName());
            }
            w.println("== STRINGS");
            for (Data d : currentProgram.getListing().getDefinedData(true)) {
                if (d.hasStringValue()) w.println(d.getAddress() + " " + d.getValue());
            }
            DecompInterface di = new DecompInterface();
            di.openProgram(currentProgram);
            w.println("== FUNCTIONS");
            for (Function f : currentProgram.getFunctionManager().getFunctions(true)) {
                if (f.isThunk() || f.isExternal()) continue;
                long size = f.getBody().getNumAddresses();
                if (size < 60) continue;
                DecompileResults r = di.decompileFunction(f, 60, monitor);
                if (r == null || !r.decompileCompleted()) continue;
                String c = r.getDecompiledFunction().getC();
                w.println("---- " + f.getName() + " @ " + f.getEntryPoint() + " size=" + size);
                w.println(c);
            }
        }
        println("dumped " + out);
    }
}
