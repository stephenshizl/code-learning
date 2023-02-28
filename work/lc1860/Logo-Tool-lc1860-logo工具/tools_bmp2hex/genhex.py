import os

if os.path.exists('bmp.bin'):
    os.remove('bmp.bin')

num = 1
while num <= 2189:
    os.system("python.exe bmp2hex.py -kbin {0}.bmp".format(num))
    num += 1