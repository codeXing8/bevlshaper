# Dependencies
import sys
import os

# Get path
relpath = sys.argv[1].split('raw_data/')
extpath = "".join([relpath[0], 'raw_data/'])
relpath = relpath[1]
date = "_".join(relpath.split('_')[0:3])
number = "_".join(relpath.split('_')[4:5])
relpath = "_".join(relpath.split('_')[0:9])
sync_url = sys.argv[1]
calib_url = "".join([extpath, date, '_calib.zip'])
tracklets_url = "".join([extpath, relpath, '_tracklets.zip'])

# Log
print(sync_url)
print(calib_url)
print(tracklets_url)

# Download files
os.system("".join(['wget ', sync_url]))
os.system("".join(['wget ', tracklets_url]))
os.system("".join(['wget ', calib_url]))