#!/usr/bin/python
# -*- coding:utf-8 -*-

# UART protocol
#
#   [0]00
#   [1]FF
#   [2]len_upper
#   [3]len_lower
#   [4]0 - (len_upper + len_lower)
#   [5]command
#   [6]data[0: len]
#   [6+len]0 - sum(data)
#   [6+len+1]EF
#
#check
#   (([2]+[3]) & 0xff) == 0x00
#   (([4]+[4+len]) & 0xff) == 0x00

import random
import serial
import subprocess
import time
import sys
import threading
import traceback
import struct
import configparser

config = configparser.ConfigParser()
config.read('/home/pi/Prog/bin/rpi_config.ini')

sys.path.append(config.get('PATH', 'EPAPERDIR'))

import epaper

PROGDIR = config.get('PATH', 'PROGDIR')
NODEDIR = config.get('PATH', 'NODEDIR')
UART_DEV = config.get('UART', 'DEV')
UART_BPS = config.getint('UART', 'BPS')
FILE_LOCALMSAT = NODEDIR + '/local_msat.txt'
FILE_INVOICE = PROGDIR + '/invoice.txt'

UART_INITWT = [0xaa, 0x55, 0x55, 0x55, 0x55, 0x55, 0x00, 0x12, 0x34, 0x56, 0x78, 0x9a]
UART_INITRD = [0x9a, 0x78, 0x56, 0x34, 0x12]

TIME_UART_TIMEOUT = 10
TIME_POLL_TIMEOUT = 30

CMD_GETBALANCE = 0x01
CMD_GETNEWADDRESS = 0x02
CMD_SETFEERATE = 0x03

CMD_INVOICE = 0x40
CMD_GETLASTINVOICE = 0x41

CMD_EPAPER = 0x7d
CMD_POLL = 0x7e
CMD_STOP = 0x7f

CMD_REPLY = 0x80

uart_dev = None
time_poll = None


def linux_cmd_exec(cmd):
    print('cmd:', cmd)
    try:
        ret = subprocess.check_output(cmd).strip()
    except subprocess.CalledProcessError as e:
        print('!!! error happen(errcode=%d) !!!' % e.returncode)
        ret = ''
    return ret


def disp_qrcode(png_file, title_str, clear_frame):
    epaper.disp_qrcode(png_file, title_str, clear_frame)


def uart_init():
    global uart_dev
    uart_dev = serial.Serial(UART_DEV, UART_BPS, timeout=TIME_UART_TIMEOUT)


def uart_term():
    global uart_dev
    uart_dev.close()


def uart_handshake():
    global uart_dev

    print('write INITWT')
    uart_dev.write(UART_INITWT)
    print('wait: 0xaa...')
    while True:
        try:
            rd = uart_dev.read(1)
            if (len(rd) == 0):
                return False
            # print('     ', rd.hex())
            if rd[0] == 0xaa:
                break
        except:
            print('traceback.format_exc():\n%s' % traceback.format_exc())
            return False
    print('wait: !0xaa...')
    while True:
        try:
            rd = uart_dev.read(1)
            # print('     ', rd.hex())
            if rd[0] != 0xaa:
                break
        except:
            return False
    print('read: reply...')
    rd = uart_dev.read(len(UART_INITRD))
    cnt = 0
    for r in rd:
        if r != UART_INITRD[cnt]:
            return False
        cnt = cnt + 1

    print('handshake fin!')

    return True


def uart_packet_read():
    """
    Receive from Arduino UART

    Parameters
    ----------
    none

    Returns
    -------
    cmd : int
        command
    data : str[]
        data
    """

    rd = uart_dev.read(6)
    if (rd is None) or (len(rd) == 0):
        return None, None
    if (rd[0] != 0x00) or (rd[1] != 0xff):
        readout = True
        for d in rd:
            if d != 0x5a:
                readout = False
                break
        if readout:
            uart_dev.reset_input_buffer()
        return None, None
    if ((rd[2] + rd[3] + rd[4]) & 0xff) != 0x00:
        return None, None

    pktlen = rd[2] << 8 | rd[3]
    cmd = rd[5]

    data = []
    if pktlen > 0:
        data = uart_dev.read(pktlen)
    tail = uart_dev.read(2)
    dcs = 0
    for d in data:
        dcs = dcs + d
    dcs = dcs + tail[0]
    if ((dcs & 0xff) != 0x00) or (tail[1] != 0xef):
        return None, None
    return cmd, data


def uart_packet_write(cmd, data):
    datalen = len(data)
    datalen_u = datalen >> 8
    datalen_d = datalen & 0xff
    head = [0x00, 0xff, datalen_u, datalen_d, (0 - datalen_u - datalen_d) & 0xff, cmd]
    dcs = 0
    for d in data:
        dcs = dcs + d
    tail = [(0 - dcs) & 0xff, 0xef]
    uart_dev.write(head)
    uart_dev.write(data)
    uart_dev.write(tail)
    if cmd != CMD_POLL | CMD_REPLY:
        print('  [W]cmd=', hex(cmd))
        #datastr = ''.join(data).hex() if len(data) > 0 else ''
        #print '  [W]data=[', datastr, ']'


def uart_loop():
    global time_poll

    while True:
        rep_data = []
        cmd, data = uart_packet_read()
        if cmd is None:
            if time.time() > time_poll + TIME_POLL_TIMEOUT:
                print('fail: poll timeout')
                return False
            continue
        if cmd != CMD_POLL:
            print('[R]cmd=', hex(cmd))
            datastr = data.hex() if len(data) > 0 else ''
            print('[R]data=[', datastr, ']')

        # command process
        if cmd == CMD_GETBALANCE:
            rep_data = struct.pack('>Q', 0x123456789abcdef0)
        if cmd == CMD_GETNEWADDRESS:
            rep_data = 'none'.encode()
        if cmd == CMD_SETFEERATE:
            pass
        if cmd == CMD_INVOICE:
            msat = struct.unpack('>Q', data[0:8])[0]
            print('msat=', msat)
            desc = ''
            if len(data) > 8:
                desc = data[8:].decode()
            th = threading.Thread(target=linux_cmd_exec,
                        args=(['bash', PROGDIR + '/bin/get_invoice.sh', str(msat), '"' + desc + '"'],),
                        name='invoice')
            th.start()
        if cmd == CMD_GETLASTINVOICE:
            with open(FILE_INVOICE) as f:
                rep_data = f.read()
                print('invoice=' + rep_data)
                rep_data = rep_data.encode()
        if cmd == CMD_EPAPER:
            if not evt_epaper:
                th = threading.Thread(target=disp_qrcode, args=('', [data], False), name='epaper')
                th.start()
                rep_data = [1]
            else:
                rep_data = [0]
        if cmd == CMD_POLL:
            time_poll = time.time()
            msat = 0
            try:
                with open(FILE_LOCALMSAT) as f:
                    msat_str = f.read()
                    msat = int(msat_str)
            except:
                pass
            rep_data = struct.pack('>Q', msat)
        if cmd == CMD_STOP:
            print(' [CMD]stop')
            disp_qrcode('', ['good-bye!'], False)
            linux_cmd_exec(['sudo', 'halt'])
            return False

        # reply to Arduino
        uart_packet_write(cmd | CMD_REPLY, rep_data)

    return True


def main():
    global time_poll

    uart_init()
    if uart_dev is None:
        print('fail: UART cannot initialized.')
        return
    ret = uart_handshake()
    if not ret:
        print('fail: handshake')
        return

    print('event loop...')
    time_poll = time.time()
    while True:
        ret = uart_loop()
        if not ret:
            break
    uart_term()


if __name__ == '__main__':
    while True:
        main()
        print('restart')
