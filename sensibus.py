#-----------------------------------------------------------------------------------
# Basic operations on the sensibus variant of the I2C bus,
# as used by sensirion SHT11 temperature and humidity sensors.
# see https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/2_Humidity_Sensors/Sensirion_Humidity_Sensors_SHT1x_Datasheet_V5.pdf
# Based on ideas from the sensirion application note and modified for raspberry pi
# The sensirion protocol is similar I2C, but the start/stop sequence is different.
#
# Originally for Tuxgraphics server board: Guido Socher
# Reworked by : Mark Pearse, Apr 2009
# Python version for Raspberry Pi: Mark Pearse, Jan 2017
#
# Copyright (c) 2017 Mark Pearse
#
#-----------------------------------------------------------------------------------
from RPi.GPIO import BOARD, IN, OUT, HIGH, LOW, PUD_UP, FALLING, setmode, setup, input, output, wait_for_edge, cleanup
from time import sleep

SEND_ACK = 0x00
NO_ACK   = 0x80

class Sensibus(object):


    def __init__(self, data, clock):
        self.clock = clock
        self.data = data
        setmode(BOARD)	
        self.tx_sequence("aI",0) # Set clock pin to output, data to input with pullup
#-----------------------------------------------------------------------------------
    def tx_sequence(self, sequence, outByte):
# Output operations send successive bits of outputSR, starting with MSB
# Input is shifted into inputSR and returned as the result
        inputSR = 0
        outputSR = outByte

        def shift_out():
            nonlocal outputSR
            outBit = 1 if (outputSR & 0x80) else 0
            outputSR <<= 1
            return outBit
            
        def shift_in(inBit):
            nonlocal inputSR
            inputSR <<= 1
            inputSR += inBit
            
        for cmd in sequence: 
            if   cmd == 'a': setup (self.clock, OUT)          # configure clock pin as output (_a_bitrary letter)
            elif cmd == 'O': setup (self.data,  OUT)          # set data port to _O_utput mode
            elif cmd == 'I': setup (self.data,  IN, pull_up_down=PUD_UP)  #set data port to _I_nput mode and pull up to 1
            elif cmd == 'c': output(self.clock, LOW)          # set _c_lock 0, data can change now
            elif cmd == 'C': output(self.clock, HIGH)         # set _C_lock 1, data valid on this edge
            elif cmd == '0': output(self.data,  LOW)          # output _0_ to data port
            elif cmd == '1': output(self.data,  HIGH)         # output _1_ to data port
            elif cmd == 'p': output(self.data, shift_out())   # _p_ut next bit from outputSR onto bus and shift outputSR
            elif cmd == 'g': shift_in(input(self.data))       # _g_et one bit from the bus and shift it onto inputSR
            sleep(3e-6)  #gap of at least 3 micro seconds between operations
    
        return inputSR
#-----------------------------------------------------------------------------------
    def write_byte(self, value):
    # writes a byte on the Sensibus, reads and returns acknowledge bit
        return self.tx_sequence("OpCcpCcpCcpCcpCcpCcpCcpCcICgc", value);
            #happily, valid ACK == 0 == OK
#-----------------------------------------------------------------------------------
    # reads a byte form the Sensibus and sends an acknowledge iff ack=SEND_ACK
    # "I" - set data line to _I_nput mode
    # "Cgc" - clock one bit of input into inputSR repeat 8 times with shift to get a byte
    # "O" - set data line to _O_utput mode
    # sensibus ack is sent by pulling data line to zero, ie ack=0 non-ack=1
    # "p" sequence sends MSBit(7) first, so ack/non-ack bit goes in there
    # "Cc" pulses clock to transfer the ack/non-ack.
    # SEND_ACK = 0x00
    # NO_ACK   = 0x80
    
    def read_byte(self, ack):
        return self.tx_sequence("ICgcCgcCgcCgcCgcCgcCgcCgcOpCcI", ack)
#-----------------------------------------------------------------------------------
    # Generates a sensibus specific transmission start
    # This is the point where sensibus departs from the I2C standard
    #       _____         ________
    # DATA:      |_______|
    #           ___     ___
    # SCK : ___|   |___|   |______
    def transstart(self):
        self.tx_sequence("cO1C0cC1cI", 0)
#-----------------------------------------------------------------------------------
    # Communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
    #      _____________________________________________________         ________
    # DATA:                                                     |_______|
    #          _    _    _    _    _    _    _    _    _        ___    ___
    # SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|  |___|   |______
    def connectionreset(self):
        self.tx_sequence("cO1CcCcCcCcCcCcCcCcCc", 0)
        self.transstart()
#-----------------------------------------------------------------------------------
    # while conversion is in progress, data is high - wait for low or end of patience (mSecs)
    def wait_for_ack(self, patience):
        x = wait_for_edge(self.data, FALLING, timeout=patience)
        return x is not None

#        for _ in xrange(patience):
#            _delay_ms(1.0)
#            if getPBt(self.data) == 0:
#                return True;
#        return False;            # timed out
#-----------------------------------------------------------------------------------
def test(data, clock):
    sb = Sensibus(data, clock)
    sb.tx_sequence("O0",0)
    print("data",sb.data,input(sb.data))
    sb.tx_sequence("O1",0)
    print("data",sb.data,input(sb.data))
    sb.tx_sequence("c",0)
    print("clock",sb.clock,input(sb.clock))
    sb.tx_sequence("C",0)
    print("clock",sb.clock,input(sb.clock))
    STATUS_REG_R = 0x07 # 000 0011 1
    print('write_byte(STATUS_REG_R)', sb.write_byte(STATUS_REG_R))
    print('read_byte(STATUS_REG_R)', sb.read_byte(SEND_ACK))
    
if __name__ == "__main__":
    test(3,5)
    