#!/usr/bin/env python3

# Convert pickled data produced by scan to .xyz format for mesh generation

import pickle
import sys

def main():
    with open(sys.argv[1], 'rb') as binfile:
        bindata = binfile.read()
    with open(sys.argv[1] + '.xyz', 'w') as outfile:
        outfile.write(to_xyz(bindata))

def to_xyz(bindata) -> str:
    data = pickle.loads(bindata)
    strbuilder = ''
    for (x, y, z) in data:
        strbuilder += "{:.2f} {:.2f} {:.2f}\n".format(x, y, z)
    return strbuilder

if __name__ == '__main__':
    main()
