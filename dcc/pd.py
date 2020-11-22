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
    outputs = ['dcc']
    tags = ['Encoding']
    channels = (
        {'id': 'data', 'name': 'Data', 'desc': 'Data line'},
    )
    options = (
        {'id': 'jitter', 'desc': 'Jitter[us]', 'default': '10'},
        {'id': 'one_min', 'desc': 'Logic One Min[us]', 'default': '100'},
        {'id': 'one_max', 'desc': 'Logic One Max[us]', 'default': '130'},
        {'id': 'zero_min', 'desc': 'Logic Zero Min[us]', 'default': '190'},
        {'id': 'zero_max', 'desc': 'Logic Zero Max[us]', 'default': '250'},
    )
    annotations = (
        ('logic', 'Logic'),
        ('period', 'Period'),
        ('type', 'Type'),
        ('adr', 'Address'),
        ('func', 'Function'),
    )
    annotation_rows = (
        ('logic', 'Logic', (0,)),
        ('period', 'Period', (1,)),
        ('type', 'Type', (2,)),
        ('adr', 'Address', (3,)),
        ('func', 'Function', (4,)),
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
            self.state = 'FIND_PREAMBLE'
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
        self.state = 'FIND_PREAMBLE'         

    def handle_data(self):
        bit = self.handle_getting_bit()
        if bit == None:
            self.state = 'FIND_PREAMBLE'
            return
        if self.data_bit_count == 0:
            if bit == 0:
                #init bit of byte
                self.data_bit_count += 1
            else:
                self.state = 'FIND_PREAMBLE'
                return
        else:
            self.databyte = (self.databyte << 1) | bit
            self.data_bit_count += 1

        if self.data_bit_count > 8:
            self.data_bit_count = 0
            self.data.append(self.databyte)
            self.databyte = 0   
            self.handle_telegram()
        
        
    def handle_telegram(self):        
        adr=0

        #recieved two bytes    
        datalen = len(self.data)
        if datalen == 2:
            if self.data[0] == 0xFF and self.data[1] == 0:
                self.state = 'FIND_PREAMBLE'
                self.put(self.data_start, self.samplenum, self.out_ann, [2, ['IDLE', 'I']])
                return

        elif datalen == 3: 
            if (self.data[0] ^ self.data[1]) == self.data[2]:
                #data is correct
                if (self.data[0] & 0x80 == 0x00):
                    #short address => else needs 4 bytes
                    adr = self.data[0] & 0x7F
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['LOCO', 'L']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(adr)]])
                    self.handle_single_command_byte(self.data[1])
                elif (self.data[0] & 0xC0 == 0x80) and (self.data[1] & 0x80 == 0x80):
                    #function decoder
                    adr = ((self.data[0] & 0x3F) | (~(self.data[1] | 0x8F)) << 1) & 0xFF
                    br = (self.data[1] & 0x06)>>1
                    ind = self.data[1] & 0x01
                    if ((self.data[1] & 0x08)>>3) == 1:
                        act = 'on'
                    else:
                        act = 'off'
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['FUNC', 'F']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(adr)]])
                    self.put(self.data_start, self.samplenum, self.out_ann, [4, ['Mod:' + str(br) + ',Inductor:' + str(ind) + '/' + act, str(br) + '/' + str(ind) + '/' + act]])
                else:
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['UNKNOWN', 'U']])
                #telegram for accessories
            elif self.data[2] == 0xFF:                        
                #byte3 was wrong and we have a preamble already
                self.preamble_count = 8
                self.state = 'FIND_PREAMBLE'
               
            
        elif datalen == 4:
            if (self.data[0] ^ self.data[1] ^ self.data[2]) == self.data[3]:
                #data is correct 
                if (self.data[0] & 0xC0 == 0xC0):
                    adr = ((self.data[0] & 0x3F) << 8) + self.data[1]
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['LOCO', 'L']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(adr)]])
                    self.handle_single_command_byte(self.data[2])
                elif (self.data[1] & 0xD0) == 0xC0 or (self.data[1] & 0xD0) == 0x20:
                    #short address and long function or speed command
                    adr = self.data[0] & 0x3F
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['LOCO', 'L']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(adr)]])
                    self.handle_two_command_byte(self.data[1], self.data[2])
                elif (self.data[0] & 0xC0 == 0x80) and (self.data[1] & 0x89 == 0x01):
                    #improved function command
                    adr = ((self.data[0] & 0x3F) | (~(self.data[1] | 0x8F)) << 1) & 0xFF
                    br = (self.data[1] & 0x06)>>1
                    func = self.data[2]
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['FUNC', 'F']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(adr)]])
                    self.put(self.data_start, self.samplenum, self.out_ann, [4, ['Mod:' + str(br) + ',Func:' + str(func), str(br) + '/' + str(func)]])
                elif (self.data[0] & 0xE0 == 0x20):
                    #analog output command
                    #potential bug is analog command which is equal to short loco address higher than 32 and Function 0x3F
                    command = self.data[0] & 0x1F
                    cmd_type = self.data[1]
                    cmd_value = self.data[2]
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['FUNC', 'F']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(command)]])
                    self.put(self.data_start, self.samplenum, self.out_ann, [4, ['Type:' + str(cmd_type) + ',Value:' + str(cmd_value), str(cmd_type) + '/' + str(cmd_value)]])
                else:
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['UNKNOWN', 'U']])
            elif self.data[3] == 0xFF:
                #byte4 was wrong and we have a preamble already
                self.preamble_count = 8
                self.state = 'FIND_PREAMBLE'
                    
        elif datalen == 5:
            if (self.data[0] ^ self.data[1] ^ self.data[2] ^ self.data[3]) == self.data[4]:
                #data is correct
                if (self.data[0] & 0xC0 == 0xC0):
                    adr = self.data[0] & 0x3F
                    adr = (adr << 8) + self.data[1]
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['LOCO', 'L']])
                    self.put(self.data_start, self.samplenum, self.out_ann, [3, [str(adr)]])
                    self.handle_two_command_byte(self.data[2], self.data[3])
                else:
                    self.put(self.data_start, self.samplenum, self.out_ann, [2, ['UNKNOWN', 'U']])
            elif self.data[4] == 0xFF:
                #byte4 was wrong and we have a preamble already
                self.preamble_count = 8
                self.state = 'FIND_PREAMBLE'
            else:
                #byte5 was wrong
                self.put(self.data_start, self.samplenum, self.out_ann, [2, ['WRONG CHECKSUM:' + str(self.data[0] ^ self.data[1] ^ self.data[2] ^ self.data[3])+' !=', str(self.data[4]), 'WRONG CHECK' ]])
                self.state = 'FIND_PREAMBLE'
        elif datalen > 5:
            self.put(self.data_start, self.samplenum, self.out_ann, [2, ['UNDEFINED SIZE']])
            self.state = 'FIND_PREAMBLE'
        
    def handle_single_command_byte(self, databyte):
        if (databyte & 0xC0) == 0x40:
            #28 speed steps
            dir = (databyte & 0x20) >> 5
            speed = databyte & 0x1F                   
            self.put(self.data_start, self.samplenum, self.out_ann, [4, ['S:' + str(speed) + ',D:' + str(dir), str(speed) + '/' + str(dir)]])
        elif (databyte & 0xE0) == 0x80:
            fout = ''
            if (databyte & 0x10) == 0x10:
                fout += 'FL/'
            if (databyte & 0x01) == 0x01:
                fout += 'F1/'
            if (databyte & 0x02) == 0x02:
                fout += 'F2/'
            if (databyte & 0x03) == 0x03:
                fout += 'F3/'
            if (databyte & 0x04) == 0x04:
                fout += 'F4/'
            self.put(self.data_start, self.samplenum, self.out_ann, [4, [fout]])
        elif (databyte & 0xF0) == 0xB0:
            fout = ''
            if (databyte & 0x01) == 0x01:
                fout += 'F5/'
            if (databyte & 0x02) == 0x02:
                fout += 'F6/'
            if (databyte & 0x03) == 0x03:
                fout += 'F7/'
            if (databyte & 0x04) == 0x04:
                fout += 'F8/'
            self.put(self.data_start, self.samplenum, self.out_ann, [4, [fout]])
        elif (databyte & 0xF0) == 0xA0:
            fout = ''
            if (databyte & 0x01) == 0x01:
                fout += 'F9/'
            if (databyte & 0x02) == 0x02:
                fout += 'F10/'
            if (databyte & 0x03) == 0x03:
                fout += 'F11/'
            if (databyte & 0x04) == 0x04:
                fout += 'F12/'
            self.put(self.data_start, self.samplenum, self.out_ann, [4, [fout]])
        else:
            #additional command not allowed for only this amount of bytes
            self.put(self.data_start, self.samplenum, self.out_ann, [4, ['UNDEFINED CMD']])
            self.state = 'FIND_PREAMBLE'

    def handle_two_command_byte(self, cmdbyte1, cmdbyte2):
        if (cmdbyte1 & 0xE0) == 0x20:
            if (cmdbyte1 & 0x1F) == 0x1F:
                #128 speed steps
                speed = cmdbyte2 & 0x7F - 1
                dir = (cmdbyte2 & 0x80) >> 7                 
                self.put(self.data_start, self.samplenum, self.out_ann, [4, ['S:' + str(speed) + ',D:' + str(dir), str(speed) + '/' + str(dir)]])
            else:
                self.put(self.data_start, self.samplenum, self.out_ann, [4, ['NOT SUPPORTED']])
        elif cmdbyte1 == 0xDE:
            #F13 to F20
            fout = ''
            if (cmdbyte2 & 0x01) == 0x01:
                fout += 'F13/'
            if (cmdbyte2 & 0x02) == 0x02:
                fout += 'F14/'
            if (cmdbyte2 & 0x04) == 0x04:
                fout += 'F15/'
            if (cmdbyte2 & 0x08) == 0x04:
                fout += 'F16/'
            if (cmdbyte2 & 0x10) == 0x10:
                fout += 'F17/'
            if (cmdbyte2 & 0x20) == 0x20:
                fout += 'F18/'
            if (cmdbyte2 & 0x40) == 0x40:
                fout += 'F19/'
            if (cmdbyte2 & 0x80) == 0x80:
                fout += 'F20/'
            self.put(self.data_start, self.samplenum, self.out_ann, [4, [fout]])
        elif cmdbyte1 == 0xDF:
            #F21 to F28
            fout = ''
            if (cmdbyte2 & 0x01) == 0x01:
                fout += 'F21/'
            if (cmdbyte2 & 0x02) == 0x02:
                fout += 'F22/'
            if (cmdbyte2 & 0x04) == 0x04:
                fout += 'F23/'
            if (cmdbyte2 & 0x08) == 0x04:
                fout += 'F24/'
            if (cmdbyte2 & 0x10) == 0x10:
                fout += 'F25/'
            if (cmdbyte2 & 0x20) == 0x20:
                fout += 'F26/'
            if (cmdbyte2 & 0x40) == 0x40:
                fout += 'F27/'
            if (cmdbyte2 & 0x80) == 0x80:
                fout += 'F28/'
            self.put(self.data_start, self.samplenum, self.out_ann, [4, [fout]])
        else:
            #additional command not allowed for only this amount of bytes
            self.put(self.data_start, self.samplenum, self.out_ann, [4, ['UNDEFINED CMD']])
            self.state = 'FIND_PREAMBLE'

    def decode(self):
        if not self.samplerate:
            raise SamplerateError('Cannot decode without samplerate.')



        self.state = 'FIND_PREAMBLE'
        self.preamble_count = 0
        self.data_bit_count = 0
        self.data = [0]
        self.preamble_start = self.samplenum
        opt = self.options
        self.jitter_edge_samplenum = int(float(opt['jitter'])*1e-6 * self.samplerate)
        self.bit_one_min_samplenum = int(float(opt['one_min'])*1e-6 * self.samplerate)
        self.bit_one_max_samplenum = int(float(opt['one_max'])*1e-6 * self.samplerate)
        self.bit_zero_min_samplenum = int(float(opt['zero_min'])*1e-6 * self.samplerate)
        self.bit_zero_max_samplenum = int(float(opt['zero_max'])*1e-6 * self.samplerate)
        self.wait({0: 'e'})
        self.bit_end_samplenum = self.samplenum

        while True:
            if (self.state == 'FIND_PREAMBLE'):
                self.handle_preamble()
            elif self.state == 'DATA':
                self.handle_data()
