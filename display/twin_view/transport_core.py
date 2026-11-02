"""1200-byte UDP chunks plus one XOR repair chunk per eight data chunks.

One lost datagram per group can be repaired without waiting for a retransmission.
Two missing chunks stay incomplete: loss is never hidden or filled with fake data.
"""
import struct, socket
import numpy as np

CHUNK=1200
GROUP=8


class ReliableSender:
    """Persistent framed TCP for Tailscale: loss recovery is handled by TCP."""
    def __init__(self):
        self.connections={}

    def prepare(self,dests):
        for dest in dests:
            if dest not in self.connections:
                conn=socket.create_connection(dest,timeout=3)
                conn.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
                conn.settimeout(.2)
                self.connections[dest]=conn

    def send_frame(self,dests,blob):
        packet=struct.pack('!I',len(blob))+blob
        for dest in dests:
            for attempt in range(2):
                try:
                    self.prepare([dest])
                    self.connections[dest].sendall(packet)
                    break
                except OSError:
                    old=self.connections.pop(dest,None)
                    if old is not None: old.close()
                    if attempt: raise
        return len(packet),1


def read_exact(conn,count):
    data=bytearray()
    while len(data)<count:
        part=conn.recv(count-len(data))
        if not part: return None
        data.extend(part)
    return bytes(data)


def fragments(blob,seq):
    count=(len(blob)+CHUNK-1)//CHUNK
    rows=np.frombuffer(blob+b'\0'*(count*CHUNK-len(blob)),np.uint8).reshape(count,CHUNK)
    packets=[]
    for start in range(0,count,GROUP):
        for i in range(start,min(start+GROUP,count)):
            packets.append(b'TWN1'+struct.pack('!IHH',seq&0xffffffff,i,count)+blob[i*CHUNK:(i+1)*CHUNK])
        parity=np.bitwise_xor.reduce(rows[start:start+GROUP],axis=0).tobytes()
        packets.append(b'TWP1'+struct.pack('!IHHI',seq&0xffffffff,start,count,len(blob))+parity)
    return packets


class Assembler:
    def __init__(self):
        self.pending={}
        self.completed={}
        self.repaired=0

    def feed(self,packet,now):
        if len(packet)<12 or packet[:4] not in (b'TWN1',b'TWP1'): return None
        seq,idx,count=struct.unpack('!IHH',packet[4:12])
        if not 1<=count<=1024 or idx>=count: return None
        self.pending={k:v for k,v in self.pending.items() if now-v['time']<.8}
        self.completed={k:v for k,v in self.completed.items() if now-v<.3}
        if seq in self.completed: return None
        p=self.pending.setdefault(seq,dict(count=count,parts={},parity={},length=None,time=now))
        if p['count']!=count: return None
        if packet[:4]==b'TWN1':
            if not 0<len(packet[12:])<=CHUNK: return None
            p['parts'][idx]=packet[12:]
        else:
            if len(packet)!=CHUNK+16 or idx%GROUP: return None
            length=struct.unpack('!I',packet[12:16])[0]
            if not (count-1)*CHUNK<length<=count*CHUNK: return None
            p['length']=length
            p['parity'][idx]=packet[16:]
        for start,repair in list(p['parity'].items()):
            missing=[i for i in range(start,min(start+GROUP,count)) if i not in p['parts']]
            if len(missing)==1:
                value=np.frombuffer(repair,np.uint8).copy()
                for i in range(start,min(start+GROUP,count)):
                    if i in p['parts']:
                        part=p['parts'][i]
                        value[:len(part)]^=np.frombuffer(part,np.uint8)
                p['parts'][missing[0]]=value.tobytes()
                self.repaired+=1
        if len(p['parts'])!=count: return None
        blob=b''.join(p['parts'][i] for i in range(count))
        if p['length'] is not None: blob=blob[:p['length']]
        del self.pending[seq]; self.completed[seq]=now
        return seq,blob
