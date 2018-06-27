#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial

class MCPC(object):
    """
    >>> cpc = mcpc.io.MCPC()
    >>>
    >>> cpc.connect(port='/dev/tty.usbserial', baudrate=9600)
    """
    def __init__(self):
        self.cnxn = None

    def connect(self, port, baudrate=9600, timeout=2, **kwargs):
        """Connect to a serial device using pyserial"""
        try:
            self.cnxn = serial.Serial(port=port, baudrate=baudrate, timeout=timeout, **kwargs)
        except ValueError as e:
            print (e)
            return False

        return True

    def close(self):
        """Close the serial connection"""
        self.cnxn.close()

        return

    def _write(self, data):
        """Write a serial command to the MCPC"""
        self.cnxn.write(data)

        return

    def _read(self, num_bytes=64):
        return self.cnxn.read(num_bytes)

    def get_status(self):
        self._write(b'status\r\n')

        resp = self._read(num_bytes=254).decode('utf-8')

        # Strip
        resp = resp.strip()

        # Split on return char
        resp = resp.split('\r')

        resp = [x.split(' ') for x in resp]

        # Flatten the list
        resp = [item.strip() for sublist in resp for item in sublist]

        # Split on '=' and set attribute to value
        res = dict()

        for item in resp:
            item = item.split('=')

            res[item[0]] = float(item[1])

        return res

    def get_settings(self):
        self._write(b'settings\r\n')

        resp = self._read(num_bytes=254).decode('utf-8')

        # Strip
        resp = resp.strip()

        # Split on return
        resp = resp.split('\r')

        # Strip
        resp = [x.strip() for x in resp]

        res = {
            'PUMP': resp[0].split('=')[-1],
            'AUTORPT': resp[1].split('=')[-1],
            'RPTLABEL': resp[2].split('=')[-1],
            'BAUDRATE': resp[3].split('=')[-1]
        }

	return res