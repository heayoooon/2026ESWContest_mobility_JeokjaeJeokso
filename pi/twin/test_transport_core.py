import unittest, random, struct
from transport_core import fragments, Assembler, read_exact, ReliableSender

class TransportTests(unittest.TestCase):
    def exercise(self,drop):
        blob=random.Random(42).randbytes(29137)
        packets=fragments(blob,45)
        packets=[p for p in packets if not (p[:4]==b'TWN1' and struct.unpack('!H',p[8:10])[0] in drop)]
        random.Random(5).shuffle(packets)
        a=Assembler(); outputs=[]
        for i,p in enumerate(packets):
            result=a.feed(p,i*.001)
            if result: outputs.append(result)
        return blob,outputs
    def test_exact_reassembly_out_of_order(self):
        blob,results=self.exercise(set())
        self.assertEqual(results,[(45,blob)])
    def test_missing_chunk_in_each_group_including_tail(self):
        blob,results=self.exercise({0,9,18,24})
        self.assertEqual(results,[(45,blob)])
    def test_two_missing_chunks_remain_missing(self):
        _,results=self.exercise({0,1})
        self.assertEqual(results,[])
    def test_malformed_datagrams_are_ignored(self):
        a=Assembler()
        for p in (b'',b'TWN1',b'TWN1'+struct.pack('!IHH',1,9,2),b'TWP1'+b'\0'*20):
            self.assertIsNone(a.feed(p,0))
    def test_incomplete_frames_expire(self):
        a=Assembler(); a.feed(fragments(b'a'*3000,1)[0],0)
        a.feed(fragments(b'b'*3000,2)[0],1)
        self.assertNotIn(1,a.pending)
    def test_tcp_reads_fragmented_frame(self):
        class Connection:
            pieces=[b'ab',b'c',b'def',b'']
            def recv(self,n): return self.pieces.pop(0)
        c=Connection()
        self.assertEqual(read_exact(c,6),b'abcdef')
        self.assertIsNone(read_exact(c,4))
    def test_tcp_length_prefix(self):
        class Connection:
            def sendall(self,p): self.packet=p
        sender=ReliableSender(); c=Connection(); dest=('test',1)
        sender.connections[dest]=c
        self.assertEqual(sender.send_frame([dest],b'hello'),(9,1))
        self.assertEqual(c.packet,struct.pack('!I',5)+b'hello')

if __name__=='__main__': unittest.main()
