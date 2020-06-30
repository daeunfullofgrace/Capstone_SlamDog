import os

qrcode_data = list()
def data_loader():
    f = open("/home/daeun/catkin_ws/src/qrcode/data/qr_code_data.txt", mode='rt')

    while True:
        line = f.readline()
        if not line: break
        if line.startswith('%'):
            print(line.split('%')[1])

    f.close()


if __name__ == '__main__':
    data_loader()
