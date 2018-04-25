"""LCM type definitions
This file automatically generated by lcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

import lcmtypes.particle_t

class particles_t(object):
    __slots__ = ["utime", "num_particles", "particles"]

    def __init__(self):
        self.utime = 0
        self.num_particles = 0
        self.particles = []

    def encode(self):
        buf = BytesIO()
        buf.write(particles_t._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qi", self.utime, self.num_particles))
        for i0 in range(self.num_particles):
            assert self.particles[i0]._get_packed_fingerprint() == lcmtypes.particle_t._get_packed_fingerprint()
            self.particles[i0]._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != particles_t._get_packed_fingerprint():
            raise ValueError("Decode error")
        return particles_t._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = particles_t()
        self.utime, self.num_particles = struct.unpack(">qi", buf.read(12))
        self.particles = []
        for i0 in range(self.num_particles):
            self.particles.append(lcmtypes.particle_t._decode_one(buf))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if particles_t in parents: return 0
        newparents = parents + [particles_t]
        tmphash = (0xc48afb43c4cd5590+ lcmtypes.particle_t._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + (tmphash>>63)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if particles_t._packed_fingerprint is None:
            particles_t._packed_fingerprint = struct.pack(">Q", particles_t._get_hash_recursive([]))
        return particles_t._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

