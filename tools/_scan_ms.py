
from pyocd.core.helpers import ConnectHelper
import struct

with ConnectHelper.session_with_chosen_probe(options={"frequency": 4000000}) as session:
    t = session.target
    t.init()
    dwt = t.read32(0xE0001004)
    base = 0x20002db0
    raw = bytes(t.read_memory_block8(base, 772))
    # 全部非零 u32
    print('DWT=0x%08x' % dwt)
    print('--- all nonzero u32 ---')
    for off in range(0, 772, 4):
        v = struct.unpack_from('<I', raw, off)[0]
        f = struct.unpack_from('<f', raw, off)[0]
        # 只打印有意义的（非垃圾大数）
        if v != 0 and (abs(f) < 1e6 or v == dwt or v < 0x1fffffff):
            tag = ''
            if v == dwt: tag = ' <== DWT!'
            elif abs(dwt - v) < 400000000: tag = ' ~DWT-1s内'
            print('  +%4d: 0x%08x f=%12.4f%s' % (off, v, f, tag))
    t.resume()
