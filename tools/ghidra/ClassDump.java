import ghidra.app.script.GhidraScript;
import ghidra.app.decompiler.DecompInterface;
import ghidra.app.decompiler.DecompileResults;
import ghidra.app.cmd.function.CreateFunctionCmd;
import ghidra.program.model.address.Address;
import ghidra.program.model.listing.Function;
import ghidra.program.model.listing.Instruction;
import ghidra.program.model.mem.Memory;
import ghidra.program.model.mem.MemoryBlock;
import ghidra.program.model.symbol.Reference;
import java.io.File;
import java.io.PrintWriter;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class ClassDump extends GhidraScript {
    DecompInterface di;
    PrintWriter w;

    Function fn(Address a) throws Exception {
        Function f = getFunctionAt(a);
        if (f == null) {
            disassemble(a);
            new CreateFunctionCmd(a).applyTo(currentProgram, monitor);
            f = getFunctionAt(a);
        }
        return f;
    }

    String decomp(Function f) {
        DecompileResults r = di.decompileFunction(f, 120, monitor);
        if (r == null || !r.decompileCompleted()) return "// decompile failed: " + (r == null ? "null" : r.getErrorMessage());
        return r.getDecompiledFunction().getC();
    }

    boolean isCode(Address a) {
        MemoryBlock b = currentProgram.getMemory().getBlock(a);
        return b != null && b.isExecute();
    }

    Address readPtr(Address a) throws Exception {
        long v = currentProgram.getMemory().getLong(a);
        return toAddr(v);
    }

    List<Function> nearCallees(Function f, long span) throws Exception {
        List<Function> out = new ArrayList<>();
        Set<Address> seen = new LinkedHashSet<>();
        long me = f.getEntryPoint().getOffset();
        DecompileResults r = di.decompileFunction(f, 120, monitor);
        if (r == null || r.getHighFunction() == null) return out;
        java.util.Iterator<ghidra.program.model.pcode.PcodeOpAST> it = r.getHighFunction().getPcodeOps();
        while (it.hasNext()) {
            ghidra.program.model.pcode.PcodeOpAST op = it.next();
            if (op.getOpcode() != ghidra.program.model.pcode.PcodeOp.CALL) continue;
            Address t = op.getInput(0).getAddress();
            if (Math.abs(t.getOffset() - me) > span || !seen.add(t) || !isCode(t)) continue;
            Function c = fn(t);
            if (c != null) out.add(c);
        }
        return out;
    }

    Address findVtableDeep(Function ctor) throws Exception {
        Address v = findVtable(ctor);
        if (v != null) return v;
        for (Function c : nearCallees(ctor, 0x400000)) {
            v = findVtable(c);
            if (v != null) { w.println("// real ctor " + c.getEntryPoint()); w.println(decomp(c)); return v; }
        }
        return null;
    }

    Address findVtable(Function ctor) throws Exception {
        Address last = null;
        DecompileResults r = di.decompileFunction(ctor, 120, monitor);
        if (r == null || r.getHighFunction() == null) return null;
        java.util.Iterator<ghidra.program.model.pcode.PcodeOpAST> it = r.getHighFunction().getPcodeOps();
        while (it.hasNext()) {
            ghidra.program.model.pcode.PcodeOpAST op = it.next();
            if (op.getOpcode() != ghidra.program.model.pcode.PcodeOp.STORE) continue;
            ghidra.program.model.pcode.Varnode val = op.getInput(2);
            long off;
            if (val.isConstant()) off = val.getOffset();
            else if (val.getDef() != null && val.getDef().getOpcode() == ghidra.program.model.pcode.PcodeOp.COPY && val.getDef().getInput(0).isConstant()) off = val.getDef().getInput(0).getOffset();
            else continue;
            Address t;
            try { t = toAddr(off); } catch (Exception e) { continue; }
            MemoryBlock b = currentProgram.getMemory().getBlock(t);
            if (b == null || b.isExecute() || !b.getName().contains("rdata")) continue;
            try {
                if (isCode(readPtr(t)) && isCode(readPtr(t.add(8))) && isCode(readPtr(t.add(16)))) last = t;
            } catch (Exception e) { }
        }
        return last;
    }

    @Override
    public void run() throws Exception {
        String[] args = getScriptArgs();
        List<String> lines = Files.readAllLines(new File(args[0]).toPath());
        long base = currentProgram.getImageBase().getOffset();
        di = new DecompInterface();
        di.openProgram(currentProgram);
        w = new PrintWriter(new File(args[1]), "UTF-8");
        for (String line : lines) {
            if (line.isBlank() || line.startsWith("#")) continue;
            String[] p = line.split("\t");
            String name = p[0];
            Address a = toAddr(base + Long.decode(p[1]));
            String mode = p.length > 2 ? p[2] : "func";
            int maxSlots = p.length > 3 ? Integer.parseInt(p[3]) : 400;
            Function f = fn(a);
            w.println("######## " + name + " " + mode + " @ " + a);
            if (f == null) { w.println("// no function"); continue; }
            if (mode.equals("func")) {
                w.println(decomp(f));
                int n = 0;
                for (Function c : nearCallees(f, 0x200000)) {
                    if (c.isExternal() || c.isThunk() || n >= 5) continue;
                    long cs = c.getBody().getNumAddresses();
                    if (cs < 40 || cs > 6000) continue;
                    w.println("---- callee of " + name + ": " + c.getName() + " @ " + c.getEntryPoint() + " size=" + cs);
                    w.println(decomp(c));
                    n++;
                }
                w.flush();
                continue;
            }
            w.println("// ctor\n" + decomp(f));
            Address vt = findVtableDeep(f);
            w.println("// vtable " + vt);
            if (vt == null) { w.flush(); continue; }
            Set<Address> seen = new LinkedHashSet<>();
            for (int i = 0; i < maxSlots; i++) {
                Address slot = vt.add(8L * i);
                Address t;
                try { t = readPtr(slot); } catch (Exception e) { break; }
                if (!isCode(t)) break;
                if (i > 0 && getReferencesTo(slot).length > 0 && !getReferencesTo(slot)[0].getFromAddress().equals(slot)) {
                    boolean other = false;
                    for (Reference r : getReferencesTo(slot)) if (!r.getReferenceType().isData() || r.getFromAddress().compareTo(f.getEntryPoint()) < 0) other = true;
                }
                int dataRefs = 0;
                for (Reference r : getReferencesTo(t)) if (r.getReferenceType().isData()) dataRefs++;
                if (dataRefs != 1 || !seen.add(t)) continue;
                Function vf = fn(t);
                if (vf == null) continue;
                long size = vf.getBody().getNumAddresses();
                if (size < 24) continue;
                w.println("---- " + name + " vslot" + i + " @ " + t + " size=" + size);
                w.println(decomp(vf));
                w.flush();
            }
        }
        w.close();
        println("ClassDump wrote " + args[1]);
    }
}
