from serial import Serial

def main():
    ser = Serial(
        port = '/dev/tty.usbmodem81365401',
        baudrate=115200
    )
    if(not ser.isOpen()):
        print('error opening serial!')
        return
    while True:
        out = '';
        while ser.inWaiting() > 0:
            out += ser.read(1);
        
        if out != '':
            print(out)

if __name__ == '__main__':
    main()
