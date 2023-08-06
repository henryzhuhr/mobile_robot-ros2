

import struct


data: float = 3.123

data_bytes = struct.pack("f", data)

print(" ".join(["%2X" % i for i in data_bytes]))

sum = 0
for b in data_bytes:
    sum += b
    print(type(b), b, sum)

print(sum)