from pyb import UART
pyb.repl_uart(None)
import time

if __name__ == "__main__":
    print('\033c')
    print('User Input Test (VCP)')

    vcp = pyb.USB_VCP()

    print(vcp.isconnected())
    while(True):
        if(vcp.any()):
            buf = vcp.read()
            vcp.write(buf)



        