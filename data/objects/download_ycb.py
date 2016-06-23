import os
import sys
import urllib
import urllib2

# Extract all files from the downloaded .tgz, and remove .tgz files.
extract = True

db_url = "http://rll.eecs.berkeley.edu/ycb/export/ycb.tgz"


def download_file(url, filename):
    u = urllib2.urlopen(url)
    f = open(filename, 'wb')
    meta = u.info()
    file_size = int(meta.getheaders("Content-Length")[0])
    print "Downloading: %s (%s MB)" % (filename, file_size/1000000.0)

    file_size_dl = 0
    block_sz = 65536
    while True:
        buffer = u.read(block_sz)
        if not buffer:
            break

        file_size_dl += len(buffer)
        f.write(buffer)
        status = r"%10d  [%3.2f%%]" % (file_size_dl/1000000.0, file_size_dl * 100. / file_size)
        status = status + chr(8)*(len(status)+1)
        print status,
    f.close()

def extract_tgz(filename, dir):
    tar_command = "tar -xzf {filename} ".format(filename=filename)
    os.system(tar_command)
    os.remove(filename)

if __name__ == "__main__":

    download_file(db_url, 'ycb.tgz')
    if extract:
        extract_tgz('ycb.tgz', '.')
