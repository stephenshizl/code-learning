import os

dirname = '.'

fileAllName = os.listdir('.')

num = 0
for name in fileAllName:
	print(name)
	if name.endswith(".bmp"):
		newname = str(num) + ".bmp"
		print(newname),
		num += 1
		os.rename(name,newname);