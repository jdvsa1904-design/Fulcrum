import pefile, sys, os
for p in sys.argv[1:]:
    pe = pefile.PE(p, fast_load=True)
    pe.parse_data_directories(directories=[pefile.DIRECTORY_ENTRY['IMAGE_DIRECTORY_ENTRY_EXPORT']])
    with open(os.path.basename(p) + '.exports.txt', 'w') as out:
        if hasattr(pe, 'DIRECTORY_ENTRY_EXPORT'):
            for s in pe.DIRECTORY_ENTRY_EXPORT.symbols:
                out.write('%x %s\n' % (pe.OPTIONAL_HEADER.ImageBase + s.address, (s.name or b'').decode()))
