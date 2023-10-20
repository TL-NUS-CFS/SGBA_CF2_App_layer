from cflib.drivers.crazyradio import Crazyradio
import time

cr = Crazyradio(devid=0) #devid = radio dongle id (0 being the first dongle)

cr.set_channel(60)
cr.set_data_rate(cr.DR_2MPS)

while True:


    # Send multicast packet to P2P port 7
    cr.set_address((0xff,0xe7,0xe7,0xe7,0xe7)) # sets destination address for outgoing packets
    cr.set_ack_enable(False) # disable acknowledgement for outgoing packets
    cr.send_packet( (0xff, 0x80, 0x63, 0x00, 0xff) ) # sends packet to destination address via radio link 
    print('send')

    time.sleep(0.01)

'''
0xff = broadcast address
0xe7 = multicast address (vendor specific address)
0x80 = 
0x63 = command to control whether crazyflies should keep flying
0x00 = 
'''