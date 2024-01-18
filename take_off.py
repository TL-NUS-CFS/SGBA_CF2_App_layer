from cflib.drivers.crazyradio import Crazyradio
import time

cr = Crazyradio(devid=0)

cr.set_channel(60)
cr.set_data_rate(cr.DR_2MPS)

for i in range(5):


    # Send multicast packet to P2P port 7
    cr.set_address((0xff,0xe7,0xe7,0xe7,0xe7))
    cr.set_ack_enable(False)
    cr.send_packet( (0xff, 0x80, 0x63, 0x01, 0xff) )
    #                      data  [0]   [1]   [2]
    #                            src   cmd   dst
    print('send')

    time.sleep(0.01)
