import os

for i in range(1000):
    cmd = "mv r_%d.jpg %d.jpg"%(i, i+1)
    os.system(cmd)
