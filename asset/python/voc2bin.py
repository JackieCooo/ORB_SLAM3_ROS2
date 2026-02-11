import sys
import struct
import argparse


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-i', '--input', type=str, default='ORBvoc.txt', required=True)
    parser.add_argument('-o', '--output', type=str, default='ORBvoc.bin')
    parser.add_argument('-l', '--len', type=int, default=32)
    args = parser.parse_args(sys.argv[1:])

    with open(args.input, 'r') as text:
        with open(args.output, 'wb+') as voc:
            k, L, s, w = text.readline().split()
            print('k:', k, 'L:', L, 's:', s, 'w:', w)

            k = int(k)
            L = int(L)
            s = int(s)
            w = int(w)
            if not 0 <= k <= 20 or not 1 <= L <= 10 or not 0 <= s <= 5 or not 0 <= w <= 3:
                print('Vocabulary loading failure: This is not a correct text file!')
                sys.exit(-1)

            voc.write(k.to_bytes(1, sys.byteorder))
            voc.write(L.to_bytes(1, sys.byteorder))
            voc.write(s.to_bytes(1, sys.byteorder))
            voc.write(w.to_bytes(1, sys.byteorder))

            while True:
                node_items = text.readline().split()
                if len(node_items) == 0:
                    break

                voc.write(int(node_items[0]).to_bytes(4, sys.byteorder))
                voc.write(int(node_items[1]).to_bytes(1, sys.byteorder))

                for desc in node_items[2 : args.len + 2]:
                    voc.write(int(desc).to_bytes(1, sys.byteorder))

                voc.write(struct.pack('f', float(node_items[-1])))

            print(f'vocabulary file saved as {args.output}, len: {voc.tell()}')
