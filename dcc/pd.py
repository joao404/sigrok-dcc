##
# This file is part of the libsigrokdecode project.
##
# Copyright (C) 2020 Marcel Maage <marcel@maage.online>
##
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
##
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
##
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
##

import sigrokdecode as srd


class SamplerateError(Exception):
    pass


class Decoder(srd.Decoder):
    api_version = 3
    id = 'dcc'
    name = 'DCC'
    longname = 'Digital Command Control'
    desc = 'DCC bitstream translated in single command messages'
    license = 'gplv2+'
    inputs = ['logic']
    outputs = []
    tags = ['Encoding']
    channels = (
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    options = (
        {'id': 'polarity', 'desc': 'Polarity', 'default': 'active-high',
            'values': ('active-low', 'active-high')},
    )
    annotations = (
        ('logic', 'Logic'),
        ('period', 'Period'),
        ('state', 'State'),
    )
    annotation_rows = (
        ('logic', 'Logic', (0,)),
        ('period', 'Period', (1,)),
        ('state', 'State', (2,)),
    )
    binary = (
        ('raw', 'RAW file'),
    )

    def __init__(self):
        self.reset()

    def reset(self):
        self.samplerate = None
        self.bit_start_samplenum = self.bit_end_samplenum = None

    def metadata(self, key, value):
        if key == srd.SRD_CONF_SAMPLERATE:
            self.samplerate = value

    def start(self):
        self.out_ann = self.register(srd.OUTPUT_ANN)
        self.out_binary = self.register(srd.OUTPUT_BINARY)
        self.out_average = \
            self.register(srd.OUTPUT_META,
                          meta=(float, 'Average', 'PWM base (cycle) frequency'))
        self.out_python = self.register(srd.OUTPUT_PYTHON)

    def putx(self, data):
        self.put(self.bit_start_samplenum, self.bit_end_samplenum, self.out_ann, data)

    def putp(self, period_t):
        # Adjust granularity.
        if period_t == 0 or period_t >= 1:
            period_s = '%.1f s' % (period_t)
        elif period_t <= 1e-12:
            period_s = '%.1f fs' % (period_t * 1e15)
        elif period_t <= 1e-9:
            period_s = '%.1f ps' % (period_t * 1e12)
        elif period_t <= 1e-6:
            period_s = '%.1f ns' % (period_t * 1e9)
        elif period_t <= 1e-3:
            period_s = '%.1f μs' % (period_t * 1e6)
        else:
            period_s = '%.1f ms' % (period_t * 1e3)

        self.put(self.bit_start_samplenum, self.bit_end_samplenum, self.out_ann, [1, [period_s]])

    def putb(self, data):
        self.put(self.bit_start_samplenum, self.bit_end_samplenum, self.out_binary, data)

    def output_bittime(self):
        # Calculate the period, the duty cycle, and its ratio.
        period = self.bit_end_samplenum - self.bit_start_samplenum

        # Report the period in units of time.
        period_t = float(period / self.samplerate)
        self.putp(period_t)

    def handle_bit_read(self):
        # Get the next two edges. Setup some variables that get
        # referenced in the calculation and in put() routines.
        first_edge_samplenum = self.bit_end_samplenum
        self.wait({0: 'e'})
        second_edge_samplenum = self.samplenum
        self.wait({0: 'e'})
        dif1_samplenum = second_edge_samplenum - first_edge_samplenum
        dif2_samplenum = self.samplenum - second_edge_samplenum 
        while abs(dif1_samplenum - dif2_samplenum) > self.jitter_edge_samplenum:
            # reset telegram and preamble detection
            first_edge_samplenum = second_edge_samplenum
            second_edge_samplenum = self.samplenum
            self.wait({0: 'e'})
            dif1_samplenum = second_edge_samplenum - first_edge_samplenum
            dif2_samplenum = self.samplenum - second_edge_samplenum 
            
        
        self.bit_start_samplenum = first_edge_samplenum
        self.bit_end_samplenum = self.samplenum

        self.output_bittime()

        return self.bit_end_samplenum - self.bit_start_samplenum

    def handle_getting_bit(self):
        bit_samplenum = self.handle_bit_read()
        # check if bit is 0
        if bit_samplenum in range(self.bit_zero_min_samplenum, self.bit_zero_max_samplenum):
            self.put(self.bit_start_samplenum, self.bit_end_samplenum, self.out_ann, [0, ['0']])
            self.last_bit = 0
            return 0
        # check if bit is 1
        elif bit_samplenum in range (self.bit_one_min_samplenum, self.bit_one_max_samplenum):
            self.put(self.bit_start_samplenum, self.bit_end_samplenum, self.out_ann, [0, ['1']])
            self.last_bit = 1
            return 1
        else:
            self.state = 'undef'
            self.last_bit = None
            return 0

    def handle_preamble(self):
        if self.preamble_count == 0:
            self.preamble_start = self.samplenum
        bit = self.handle_getting_bit()
        if bit == 1:
            self.preamble_end = self.samplenum
            self.preamble_count += 1
            return
        elif bit == 0:
            if self.preamble_count > 15:
                #preamble finished
                self.put(self.preamble_start, self.preamble_end, self.out_ann, [2, ['PREAMBLE ' + str(self.preamble_count), 'PRE', 'P']])
                self.preamble_count = 0
                self.data_bit_count = 1
                self.data_start = self.samplenum
                self.data.clear()
                self.databyte = 0
                self.state = 'DATA'
                return
        self.preamble_count = 0
        self.state = 'FIND PREAMBLE'         

    def handle_data(self):
        bit = self.handle_getting_bit()
        if bit == None:
            self.state = 'FIND PREAMBLE'
            return
        if self.data_bit_count == 0:
            if bit == 0:
                #init bit of byte
                self.data_bit_count += 1
            else:
                self.state = 'FIND PREAMBLE'
                return
        else:
            self.databyte = (self.databyte << 1) | bit
            self.data_bit_count += 1

        if self.data_bit_count > 8:
            self.data_bit_count = 0
            self.data.append(self.databyte)
            self.databyte = 0   

            adr=0
            speed=0
            dir=0

            #recieved two bytes    
            datalen = len(self.data)
            if datalen == 2:
                if self.data[0] == 0xFF and self.data[1] == 0:
                    self.state = 'FIND PREAMBLE'
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['idle']])
                    return

            elif datalen == 3: 
                if (self.data[0] ^ self.data[1]) == self.data[2]:
                    #data is correct
                    if (self.data[0] & 0xC0 == 0x00):
                        #short address => else needs 4 bytes
                        adr = self.data[0] & 0x7F
                        dir = (self.data[1] & 0x20) >> 5
                        speed = self.data[1] & 0x1F
                        self.put(self.data_start, self.samplenum, self.out_ann, [2, ['A:' + str(adr) + ' S:' + str(speed) + '/' + str(dir), 'A:' + str(adr)]])
                    elif (self.data[0] & 0xC0 == 0x80) and (self.data[1] & 0x80 == 0x80):
                        adr = ((self.data[0] & 0x3F) | (~(self.data[1] | 0x8F)) << 1) & 0xFF
                        br = (self.data[1] & 0x06)>>1
                        self.put(self.data_start, self.samplenum, self.out_ann, [2, [str(adr) + '/' + str(br) + '/' + str(self.data[1] & 0x01), str(adr)]])
                    else:
                        self.put(self.data_start, self.samplenum, self.out_ann, [2, ['Unknown']])
                    #telegram for accessories
                elif self.data[2] == 0xFF:                        
                    #byte3 was wrong and we have a preamble already
                    self.preamble_count = 8
                    self.state = 'FIND PREAMBLE'
               
            
            elif datalen == 4:
                if (self.data[0] ^ self.data[1] ^ self.data[2]) == self.data[3]:
                    #data is correct
                    adr = self.data[0] & 0x3F
                    if (self.data[0] & 0xC0 == 0xC0):
                        adr = (adr << 8) + self.data[1]
                        dir = (self.data[2] & 0x20) >> 5
                        speed = self.data[2] & 0x1F
                    elif self.data[1] == 0x3F:
                        speed = self.data[2] & 0x7F - 1
                        dir = (self.data[2] & 0x80) >> 7

                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['A:' + str(adr) + ' S:' + str(speed) + '/' + str(dir), 'A:' + str(adr)]])
                elif self.data[3] == 0xFF:
                    #byte4 was wrong and we have a preamble already
                    self.preamble_count = 8
                    self.state = 'FIND PREAMBLE'
                    
            elif datalen == 5:
                if (self.data[0] ^ self.data[1] ^ self.data[2] ^ self.data[3]) == self.data[4]:
                    #data is correct
                    adr = self.data[0] & 0x3F
                    if (self.data[0] & 0xC0 == 0xC0):
                        adr = (adr << 8) + self.data[1]
                        if self.data[2] == 0x3F:
                            speed = self.data[3] & 0x7F - 1
                            dir = (self.data[3] & 0x80) >> 7
                    elif self.data[2] == 0x3F:
                        speed = self.data[2] & 0x7F - 1
                        dir = (self.data[2] & 0x80) >> 7
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['A:' + str(adr) + ' S:' + str(speed) + '/' + str(dir), 'A:' + str(adr)]])
                elif self.data[4] == 0xFF:
                    #byte4 was wrong and we have a preamble already
                    self.preamble_count = 8
                    self.state = 'FIND PREAMBLE'
                else:
                    #byte5 was wrong
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, [str(self.data[0] ^ self.data[1] ^ self.data[2] ^ self.data[3])+' !=', str(self.data[4]) ]])
                    self.state = 'FIND PREAMBLE'

            elif datalen > 5:
                self.put(self.data_start, self.samplenum, self.out_ann, [2, ['Undefined Size']])
                self.state = 'FIND PREAMBLE'
        

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')



        self.state = 'undef'
        self.preamble_count = 0
        self.data_bit_count = 0
        self.data = [0]
        self.preamble_start = self.samplenum
        self.jitter_edge_samplenum = int(10e-6 * self.samplerate)
        self.bit_one_min_samplenum = int(100e-6 * self.samplerate)
        self.bit_one_max_samplenum = int(130e-6 * self.samplerate)
        self.bit_zero_min_samplenum = int(190e-6 * self.samplerate)
        self.bit_zero_max_samplenum = int(250e-6 * self.samplerate)
 
        self.wait({0: 'e'})
        self.bit_end_samplenum = self.samplenum

        while True:
            if (self.state == 'undef') or (self.state == 'FIND PREAMBLE'):
                self.handle_preamble()
            elif self.state == 'DATA':
                self.handle_data()
