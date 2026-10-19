import pefile, sys, struct, re, os
from capstone import Cs, CS_ARCH_X86, CS_MODE_64

dll = sys.argv[1]
target = sys.argv[2]
maxlen = int(sys.argv[3]) if len(sys.argv) > 3 else 4000
B = r'C:\Program Files\Autodesk\Maya2027\bin'
path = dll if os.path.isabs(dll) else os.path.join(B, dll)
pe = pefile.PE(path, fast_load=True)
pe.parse_data_directories(directories=[pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_EXPORT'], pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_IMPORT']])
base = pe.OPTIONAL_HEADER.ImageBase
names = {}
for s in pe.DIRECTORY_ENTRY_EXPORT.symbols:
    if s.name:
        names[base + s.address] = s.name.decode()
imps = {}
for e in getattr(pe, 'DIRECTORY_ENTRY_IMPORT', []):
    for i in e.imports:
        imps[i.address] = (e.dll.decode() + '!' + (i.name.decode() if i.name else str(i.ordinal)))
if target.startswith('0x'):
    addr = int(target, 16)
else:
    cands = [a for a, n in names.items() if target in n]
    if len(cands) != 1:
        for a in cands:
            print(hex(a), names[a])
        if not cands:
            sys.exit('none')
        addr = cands[0]
    else:
        addr = cands[0]
print('==', hex(addr), names.get(addr, ''))
data = pe.get_data(addr - base, maxlen)
md = Cs(CS_ARCH_X86, CS_MODE_64)
md.detail = False
def rd(va, n):
    try:
        return pe.get_data(va - base, n)
    except Exception:
        return None
seen_ret = False
for ins in md.disasm(data, addr):
    line = '%x: %-8s %s' % (ins.address, ins.mnemonic, ins.op_str)
    m = re.search(r'\[rip ([+-]) 0x([0-9a-f]+)\]', ins.op_str)
    if m:
        off = int(m.group(2), 16) * (1 if m.group(1) == '+' else -1)
        va = ins.address + ins.size + off
        if va in imps:
            line += '    ; ' + imps[va]
        elif va in names:
            line += '    ; ' + names[va]
        else:
            b = rd(va, 16)
            if b and len(b) >= 8:
                d = struct.unpack('<d', b[:8])[0]
                f = struct.unpack('<f', b[:4])[0]
                line += '    ; %x d=%r f=%r' % (va, d, f)
    if ins.mnemonic in ('call', 'jmp') and ins.op_str.startswith('0x'):
        t = int(ins.op_str, 16)
        if t in names:
            line += '    ; ' + names[t]
    print(line)
    if ins.mnemonic == 'int3' and seen_ret:
        break
    seen_ret = ins.mnemonic in ('ret', 'jmp') or (seen_ret and ins.mnemonic == 'int3')
