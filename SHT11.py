
# This is code for the sensibus variant of the I2C bus
# as used by the Sensirion SHT11 temperature and humidity sensor.
# The sensibus protocol looks similar but the start/stop
# sequence is different.
# References
# [1] Sensirion_Humidity_Sensors_SHT1x_Datasheet_V5.pdf
# [2] sht10_CRC_Calculation_Humidity_Sensor_E.pdf

from sensibus import Sensibus, SEND_ACK, NO_ACK
from math import log, exp

# Commands for SHT11    # adr cmd  r/w
STATUS_REG_W = 0x06     # 000 0011 0
STATUS_REG_R = 0x07     # 000 0011 1
MEASURE_TEMP = 0x03     # 000 0001 1
MEASURE_HUMI = 0x05     # 000 0010 1
RESET        = 0x1e     # 000 1111 0

# Masks to define bits in Status Register, see Ref1 Table 5
SM_LOW_RES      = 0x01  # 0 => temp 14bit, hum 12bit; 1 => temp 12bit, hum 8 bit
                        # 14/12/8 bit measurements take 320/80/20 mS (Ref1 3.3)
SM_NO_LOAD_OTP  = 0x02  # 1 => do not reload calibration data before measurement
                        # saves about 10mS on each measurement
SM_HEATER       = 0x04  # 1 => switch on heater for testing
SM_VDD_LOW      = 0x80  # 1 => Vdd supply below 2.47V (+/- 0.05V)

                        # only bits 0-2 writable, see ref [2]
SM_WRITABLE = SM_LOW_RES | SM_NO_LOAD_OTP | SM_HEATER
#-----------------------------------------------------------------------------------
class CRC(object):

    def __init__(self, seed):
        self.value = seed     
#-----------------------------------------------------------------------------------
    def process(inByte):
        for _ in range(8):
            if ((self.value ^ inByte) & 0x01) != 0:
                self.value ^= 0x118
            self.value >>= 1
            inByte   >>= 1
#-----------------------------------------------------------------------------------
def computeCRC8(inByte, seedByte):
# one step of the Cyclic Redundancy Check algorithm
    for _ in range(8):
        if seedByte & 0b1 != inByte & 0b1:
            seedByte ^= 0x118
        seedByte >>= 1
        inByte   >>= 1
    return seedByte
#-----------------------------------------------------------------------------------
def reverse_bits(byte):
    result = 0
    for _ in range(8):
        result = (result << 1) + (byte & 1)
        byte >>= 1
    return result
#-----------------------------------------------------------------------------------
def join_bytes(first,*rest):
    for byte in rest:
        first = (first<<8) + (byte&0xFF)
    return first
#-----------------------------------------------------------------------------------
class SHT11_Error(Exception): pass
class SHT11_WriteFail(SHT11_Error): pass
class SHT11_Timeout(SHT11_Error): pass
class SHT11_CRCFail(SHT11_Error): pass
#-----------------------------------------------------------------------------------
class SHT11(Sensibus):

    def __init__(self, dataPin, sckPin, status=0, useCRC=True):
        super(SHT11, self).__init__(dataPin, sckPin)
        self.useCRC = useCRC
        self.write_status(status)    # put SHT11 in known state. checks tx OK
#-----------------------------------------------------------------------------------
    def CRCbytes(self, *bytes):
    # as ref [2] section 2.1 5), but implemented here with CRC reversed
        crc = self.status_written & 0b1111
        for byte in bytes:
            crc = computeCRC8(reverse_bits(byte), crc)
        return crc
#-----------------------------------------------------------------------------------
    def check_CRC(self, *bytes, rcvd_crc):
    # as ref [2] section 2.1 5), but implemented here with CRC reversed
        crc_mill = CRC(self.status_written & 0b1111)
        for byte in bytes:
            crc_mill.process(reverse_bits(byte))
        return crc_mill.value == rcvd_crc
#-----------------------------------------------------------------------------------
    def get_bytes(self, mode, n, timeout=0):
        self.transstart();
        if self.write_byte(mode):  raise SHT11_WriteFail
        if timeout > 0:
            if not self.wait_for_ack(timeout): raise SHT11_Timeout
        if self.useCRC:
            bytes   = [self.read_byte(SEND_ACK) for _ in range(n)]
            checksum = self.read_byte(  NO_ACK);     # NO_ACK => end of transfer
            if self.CRCbytes(mode,*bytes) != checksum: raise SHT11_CRCFail
        else:
            acks  = [SEND_ACK]*(n-1) + [NO_ACK] 
            bytes = [self.read_byte(ack) for ack in acks]
        return join_bytes(*bytes);
#-----------------------------------------------------------------------------------
    def send_bytes(self, mode, bytes):
        self.transstart();
        for byte in [mode] + bytes:
            if self.write_byte(byte):  raise SHT11_WriteFail
#-----------------------------------------------------------------------------------
    def softreset(self):
        self.connectionreset()
        if self.write_byte(RESET):  raise SHT11_WriteFail
        sleep(11e-3)      # prevent any other communication before reset complete, see Ref1 Table 4
#-----------------------------------------------------------------------------------
    def get_SO_temp(self):
        return self.get_bytes(MEASURE_TEMP, 2, 400) # 2 bytes with 320mS+25% allowed for conversion
#-----------------------------------------------------------------------------------
    def get_SO_humi(self):
        return self.get_bytes(MEASURE_HUMI, 2, 120) # 2 bytes with 80mS+25%  allowed for conversion
#-----------------------------------------------------------------------------------
    def get_status(self):
        return self.get_bytes(STATUS_REG_R, 1)  
#-----------------------------------------------------------------------------------
    def write_status_bit(self, mask, value):
        if value:
            new_status = self.status_written | mask
        else:
            new_status = self.status_written & ~mask
        self.write_status(new_status)
#-----------------------------------------------------------------------------------
    def write_status(self, status_to_write):
    # write status register and check
        status_to_write &= SM_WRITABLE  # only bits 0-2 writable, see ref [2]
        try:
            self._write_status_checked(status_to_write)
            return
        except SHT11_Error: # may be tx error, CRC fail, or wrong echoed value
            pass
        self.softreset()
        self._write_status_checked(status_to_write)
#-----------------------------------------------------------------------------------
    def _write_status_checked(self, status_to_write):
        self._write_status(status_to_write)
        status_echoed = self.get_status()
        if status_to_write != (status_echoed & SM_WRITABLE):
            raise SHT11Error
#-----------------------------------------------------------------------------------
    def _write_status(self, newstatus):
        self.status_written = newstatus
        self.send_bytes(STATUS_REG_W,[newstatus])
#-----------------------------------------------------------------------------------
    def isHighRes(self):
        return self.status_written & SM_LOW_RES == 0
#-----------------------------------------------------------------------------------
    def isHeating(self):
        return self.status_written & SM_HEATER != 0
#-----------------------------------------------------------------------------------
    def isSupplyLow(self):
        return self.status_written & SM_VDD_LOW != 0
#-----------------------------------------------------------------------------------
    def celsius(self):
        soT = self.get_SO_temp()
        if self.isHighRes():
            d1,d2 = (-39.7,0.01)    # [1] Table 8 3,3V
        else:
            d1,d2 = (-39.7,0.04)    # [1] Table 8 3,3V
        return d1 + d2*soT
#-----------------------------------------------------------------------------------
    def rel_humidity(self, temp=25):
        soRH = self.get_SO_humi()
        if self.isHighRes():
            c1,c2,c3 = (-2.0468, 0.0367, -1.5955E-6)  # 12 bit [1] Table 6
        else:
            c1,c2,c3 = (-2.0468, 0.5872, -4.0845E-8)  #  8 bit
        linear_hum = c1 + (c2 + c3*soRH)*soRH
        T1 =  0.01      # for 14 Bit @ 5V
        T2 =  0.00008   # for 14 Bit @ 5V
        return (temp - 25.0 ) * (T1 + T2*soRH) + linear_hum            
#-----------------------------------------------------------------------------------
    def temp_humidity(self):
        temp = self.celsius()
        humidity = self.rel_humidity(temp)
        return (temp, humidity)
#-----------------------------------------------------------------------------------
def calculate_dew_point2(temperature, rel_humidity):
# August-Roche-Magnus approximation.  see http://andrew.rsmas.miami.edu/bmcnoldy/Humidity.html
    LN, RH, T = log, rel_humidity, temperature
    return 243.04*(LN(RH/100)+((17.625*T)/(243.04+T)))/(17.625-LN(RH/100)-((17.625*T)/(243.04+T))) #pasted untouched 
#-----------------------------------------------------------------------------------
def calculate_dew_point(temperature, rel_humidity):
    # see Ref1 section 4.4
    # August-Roche-Magnus approximation.  see http://andrew.rsmas.miami.edu/bmcnoldy/Humidity.html
    if temperature > 0:      #see Table 9
        tn,m = (243.12, 17.62)  
    else:
        tn,m = (272.62, 22.46)
    # equation of section 4.4 is manipulated to extract larger common subexpression
    subexp = (tn + temperature)*log(rel_humidity/100.0)/m
    return (temperature + subexp) / (1 - subexp/tn)
#-----------------------------------------------------------------------------------
def absolute_humidity(rel_humidity, temp):
# convert relative humidity to absolute humidity in grams/m^3 at pressure 760 mmHg
# see https://carnotcycle.wordpress.com/2012/08/04/how-to-convert-relative-humidity-to-absolute-humidity/
# rel_humidity is a percentage, temp is Celsius.
    sat_humidity = exp((17.67*temp)/(temp+243.5)) * 1324.7 / (273.15+temp)
    return sat_humidity*(rel_humidity/100.0)                                                                               
#-----------------------------------------------------------------------------------
#Calculation formulas for humidity                                                                                        
#http://www.vaisala.com/Vaisala%20Documents/Application%20notes/Humidity_Conversion_Formulas_B210973EN-F.pdf
#-----------------------------------------------------------------------------------
#-----------------------------------------------------------------------------------
def test_status_bits(sht11):
    sht11.write_status(0)
    print(sht11.get_status())
    sht11.write_status_bit(SM_LOW_RES,1)
    print(sht11.get_status())
    sht11.write_status_bit(SM_NO_LOAD_OTP,1)
    print(sht11.get_status())
    sht11.write_status_bit(SM_HEATER,1)
    print(sht11.get_status())
    sht11.write_status_bit(SM_LOW_RES,0)
    print(sht11.get_status())
    sht11.write_status_bit(SM_NO_LOAD_OTP,0)
    print(sht11.get_status())
    sht11.write_status_bit(SM_HEATER,0)
    print(sht11.get_status())
#-----------------------------------------------------------------------------------
def test_calculate_dew_point(data, clock):
    sht11 = SHT11(data, clock)
    #http://www.dpcalc.org
    layout = 'calculate_dew_point({}°C, {}%) = {:2.1f}°C  cf  {:2.1f}°C, {:2.1f}°C'
    for (temp, humi, dp) in (0,20,-21), (0,50,-9), (0,90,-2), (20,20,-4),(20,50,9),(20,90,18),(40,20,13),(40,50,28),(40,90,38):
        print (layout.format(temp,humi, calculate_dew_point(temp,humi), dp, calculate_dew_point2(temp,humi)))
#-----------------------------------------------------------------------------------
def test_absolute_humidity(data, clock):
    sht11 = SHT11(data, clock)
    #http://planetcalc.com/2167/
    layout = 'absolute_humidity({}°C, {}%) = {:2.3f}g/m3  cf  {:2.3f}g/m3 '
    for (temp, humi, ah) in (0.0,20,0.0010), (0.0,50,0.0024), (0.0,90,0.0044), (20,20,0.0035),(20,50,0.0087),(20,90,0.0156),(40,20,0.0102),(40,50,0.0256),(40,90,0.0461):
        print (layout.format(temp,humi, absolute_humidity(humi, temp), ah*1000))
#-----------------------------------------------------------------------------------
def test_many_reads(data, clock):
#    sht11 = SHT11(data, clock, useCRC=False)
    sht11 = SHT11(data, clock, status=SM_HEATER)
#    sht11.softreset()
#    return
    
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.rel_humidity(),'%')
    
    print(sht11.write_status(SM_LOW_RES))    #12 bit
    print(sht11.get_status())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.rel_humidity(),'%')
    
    print(sht11.write_status(0))    #14 bit
    print(sht11.get_status())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.celsius())
    print(sht11.rel_humidity(),'%')
    print(sht11.get_status())
#-----------------------------------------------------------------------------------
def test(data, clock):
#    test_many_reads(data, clock)
#    test_calculate_dew_point(data, clock)
#    test_absolute_humidity(data, clock)
    sht11 = SHT11(data, clock)
    test_status_bits(sht11)
    
#-----------------------------------------------------------------------------------

if __name__ == "__main__":
    test(3,5)
