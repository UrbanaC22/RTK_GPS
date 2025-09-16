#!/usr/bin/env python3

from pynmeagps import NMEAReader
from nmea_msgs.msg import Sentence
import rclpy
from rclpy.node import Node
from . import coordTransform_utils as transform 
import time

#GPSDSocket creates a GPSD socket connection & request/retrieve GPSD output
class ConvertedCoordinate(Node):
    def __init__(self):
        super().__init__('nmea_subscriber')
        self.nmea_sub=self.create_subscription(Sentence, 'nmea_data', self.callback, 10)
	#DataStream unpacks the streamed gpsd data into python dictionaries.
        gcj02_lng_lat = [0.0,0.0]
        bd09_lng_lat = [0.0,0.0]

        print('gps device make wgs84 coordinate\r\ngcj02 coordinate is for amap or google map\r\nbd09 coordinate is for baidu map\r\n\033[1;31m Please press Ctrl+c if want to exit \033[0m')
	
	
    def callback(self, msg):
        nmr= msg.sentence       
        if nmr.startswith("$GNGGA"):
            data_stream=NMEAReader.parse(nmr)       
            gcj02_lng_lat = transform.wgs84_to_gcj02(float(data_stream.lon),float(data_stream.lat))
            bd09_lng_lat = transform.wgs84_to_bd09(float(data_stream.lon),float(data_stream.lat))
            print('altitude       = ', data_stream.alt,'M',end='\r\n',flush=True) 
            print('wgs84 lon,lat  = ',data_stream.lon,',',data_stream.lat,end='\r\n',flush=True)                   # gps device make
            print('google lon.lat = %.9f,%.9f' %    (gcj02_lng_lat[1],gcj02_lng_lat[0]),end='\r\n',flush=True)         # google map could use
            print('amap lon.lat   = %.9f,%.9f' %(gcj02_lng_lat[0],gcj02_lng_lat[1]),end='\r\n',flush=True)         # amap soso map could use
            print('bd09 lon,lat   = %.9f,%.9f' %(bd09_lng_lat[0],bd09_lng_lat[1]),end='\r\n',flush=True)           # baidu map could use

            for i in range(5, 0, -1):
                print('\r\033[Kupdate after {} seconds'.format(i), end='', flush=True)
                time.sleep(1)
            print('\r\033[K', end="", flush=True)
            print('\n')

        else:
            print('\033[1;32m Module not ready,please move the antenna to outdoors \033[0m',end='\r',flush=True)
         
         
def main(args=None):
    rclpy.init(args=args)
    node=ConvertedCoordinate()
    
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
    
        

		    
