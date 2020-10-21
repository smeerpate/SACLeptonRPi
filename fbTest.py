import numpy as np

fbCont = np.ones((500,500,4),dtype=np.uint8)*255
print(fbCont)
with open('/dev/fb0', 'rb+') as _fBuf:
        print(_fBuf)
        _fBuf.write(fbCont)

