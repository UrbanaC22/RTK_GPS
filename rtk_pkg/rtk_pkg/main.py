#!/usr/bin/env python3 
"""  
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
#!/home/username/env/bin/python3(add this if the above shebang does not work(in coordinate.py as well))
"""

from rclpy.node import Node
from nmea_msgs.msg import Sentence
from optparse import OptionParser
from pynmeagps import NMEAReader
import socket
import sys
import datetime
import base64
import time
import rclpy
import ssl 
import threading
import serial


version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

factor=2
maxReconnect=1
maxReconnectTime=1200
sleepTime=1


class NtripClient(Node):
    def __init__(self,
                 buffer=1024,
                 user="",
                 out=sys.stdout,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0, 
                 fileOutput=False,
                 gps_file_path=None
                ):
        super().__init__('nmea_publisher')
        self.nmea_pub = self.create_publisher(Sentence, 'nmea_sentence', 10)
        self.buffer = buffer
        self.user = base64.b64encode(bytes(user, 'utf-8')).decode("utf-8")
        self.out = out
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.height = height
        self.verbose = verbose
        self.ssl = ssl
        self.host = host
        self.UDP_Port = UDP_Port
        self.V2 = V2
        self.headerFile = headerFile
        self.headerOutput = headerOutput
        self.maxConnectTime = maxConnectTime
        self.maxReconnect = maxReconnect
        self.fileOutput = fileOutput
        self.socket = None
        self.latest_gga = None          
        self.latest_gga_lock = threading.Lock() 
        self.setPosition(lat, lon)

        try:
            self.stream = serial.Serial('/dev/ttyS0', 115200, timeout=1)  
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)
        self.nmr = NMEAReader(self.stream)

        self.gps_file_path = gps_file_path
        if self.gps_file_path:
            self.gps_file = open(self.gps_file_path, 'ab')
        else:
            self.gps_file = None

        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('', 0))
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket = None

        # Dedicated thread for continuous NMEA reading at 1Hz
        threading.Thread(target=self._nmea_reader_loop, daemon=True).start()
        threading.Thread(target=self.readData, daemon=True).start()

    def _nmea_reader_loop(self):

        while rclpy.ok():
            try:
                raw_data, parsed_data = self.nmr.read()
                if not raw_data:
                    continue

                if b"GNGGA" in raw_data:
                   
                    with self.latest_gga_lock:
                        self.latest_gga = raw_data

                    msg = Sentence()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    try:
                        msg.sentence = raw_data.decode('ascii').strip()
                    except UnicodeDecodeError:
                        msg.sentence = raw_data.decode('utf-8', errors='replace').strip()
                    self.nmea_pub.publish(msg)

                    print(raw_data)
                    location = self.parse_gngga_sentence(raw_data.decode('ascii', errors='replace'))
                    if location:
                        lat, lon, _, fix_quality = location
                        fix_status = {
                            0: "Invalid",
                            1: "GPS fix",
                            2: "DGPS fix",
                            3: "PPS fix",
                            4: "RTK fixed",
                            5: "RTK float",
                            6: "Estimated",
                            7: "Manual input",
                            8: "Simulation"
                        }.get(fix_quality, "Unknown")
                        print(f"Parsed Location: Latitude={lat:.8f}, Longitude={lon:.8f}, Fix Status={fix_status}")

                    if self.gps_file:
                        self.gps_file.write(raw_data)
                        self.gps_file.flush()

            except Exception as e:
                self.get_logger().warn(f"NMEA read error: {e}")
                time.sleep(0.1)

    def setPosition(self, lat, lon):
        self.flagN = "N"
        self.flagE = "E"
        if lon > 180:
            lon = (lon - 360) * -1
            self.flagE = "W"
        elif (lon < 0 and lon >= -180):
            lon = lon * -1
            self.flagE = "W"
        elif lon < -180:
            lon = lon + 360
            self.flagE = "E"
        self.lon = lon
        if lat < 0:
            lat = lat * -1
            self.flagN = "S"
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon - self.lonDeg) * 60
        self.latMin = (lat - self.latDeg) * 60

    def getMountPointBytes(self):
        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (
            self.mountpoint, useragent, self.user)
        if self.host or self.V2:
            hostString = "Host: %s:%i\r\n" % (self.caster, self.port)
            mountPointString += hostString
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        if self.verbose:
            print(mountPointString)
        return bytes(mountPointString, 'ascii')

    def getGGABytes(self):
        timeout = 10  
        start = time.time()
        while time.time() - start < timeout:
            with self.latest_gga_lock:
                if self.latest_gga is not None:
                    return self.latest_gga
            time.sleep(0.1)
        self.get_logger().warn("Timeout waiting for GGA sentence")
        return b""

    def parse_gngga_sentence(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        if len(parts) < 10:
            return None
        try:
            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat = -lat
            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                lon = -lon
            alt = float(parts[9]) if parts[9] else 0.0
            fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        except ValueError:
            return None
        return lat, lon, alt, fix_quality

    def readData(self):
        reconnectTry = 1
        sleepTime = 1
        if self.maxConnectTime > 0:
            EndConnect = datetime.timedelta(seconds=self.maxConnectTime)

        while rclpy.ok() and reconnectTry <= self.maxReconnect:
            found_header = False
            if self.verbose:
                sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry, self.maxReconnect))

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.ssl:
                self.socket = ssl.wrap_socket(self.socket)

            error_indicator = self.socket.connect_ex((self.caster, self.port))
            if error_indicator == 0:
                sleepTime = 1
                connectTime = datetime.datetime.now()
                self.socket.settimeout(10)
                self.socket.sendall(self.getMountPointBytes())

                while not found_header:
                    casterResponse = self.socket.recv(4096)
                    header_lines = casterResponse.decode('utf-8').split("\r\n")

                    for line in header_lines:
                        if line == "":
                            if not found_header:
                                found_header = True
                                if self.verbose:
                                    sys.stderr.write("End Of Header\n")
                        else:
                            if self.verbose:
                                sys.stderr.write("Header: " + line + "\n")
                        if self.headerOutput:
                            self.headerFile.write(line + "\n")

                    for line in header_lines:
                        if line.find("SOURCETABLE") >= 0:
                            sys.stderr.write("Mount point does not exist")
                            sys.exit(1)
                        elif line.find("401 Unauthorized") >= 0:
                            sys.stderr.write("Unauthorized request\n")
                            sys.exit(1)
                        elif line.find("404 Not Found") >= 0:
                            sys.stderr.write("Mount Point does not exist\n")
                            sys.exit(2)
                        elif line.find("ICY 200 OK") >= 0:
                            self.socket.sendall(self.getGGABytes())
                        elif line.find("HTTP/1.0 200 OK") >= 0:
                            self.socket.sendall(self.getGGABytes())
                        elif line.find("HTTP/1.1 200 OK") >= 0:
                            self.socket.sendall(self.getGGABytes())

                data = "Initial data"
                while data:
                    try:
                        data = self.socket.recv(self.buffer)
                        if not data:
                            break
                        self.stream.write(data)

                        if self.UDP_socket:
                            self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))

                        if self.maxConnectTime:
                            if datetime.datetime.now() > connectTime + EndConnect:
                                if self.verbose:
                                    sys.stderr.write("Connection Timed exceeded\n")
                                sys.exit(0)

                    except socket.timeout:
                        if self.verbose:
                            sys.stderr.write('Connection TimedOut\n')
                        data = False
                    except socket.error:
                        if self.verbose:
                            sys.stderr.write('Connection Error\n')
                        data = False

                if self.verbose:
                    sys.stderr.write('Closing Connection\n')
                self.socket.close()
                self.socket = None

                if reconnectTry < self.maxReconnect:
                    sys.stderr.write("%s No Connection to NtripCaster. Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                    time.sleep(sleepTime)
                    sleepTime *= factor
                    if sleepTime > maxReconnectTime:
                        sleepTime = maxReconnectTime
                else:
                    sys.exit(1)

                reconnectTry += 1
            else:
                self.socket = None
                if self.verbose:
                    print("Error indicator: ", error_indicator)

                if reconnectTry < self.maxReconnect:
                    sys.stderr.write("%s No Connection to NtripCaster. Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                    time.sleep(sleepTime)
                    sleepTime *= factor
                    if sleepTime > maxReconnectTime:
                        sleepTime = maxReconnectTime
                reconnectTry += 1

    def shutdown(self):
        if self.socket:
            self.socket.close()
        if self.fileOutput and self.out not in (sys.stdout, None):
            self.out.close()
        if self.headerOutput and self.headerFile not in (sys.stderr, None):
            self.headerFile.close()
        if self.stream and self.stream.is_open:
            self.stream.close()
        sys.exit(0)


def main(args=None):
    usage = "NtripClient.py [options] [caster] [port] mountpoint"
    parser = OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS")
    parser.add_option("-o", "--org", type="string", dest="org")
    parser.add_option("-b", "--baseorg", type="string", dest="baseorg")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09)
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66)
    parser.add_option("-e", "--height", type="float", dest="height", default=1200)
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=True)
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False)
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False)
    parser.add_option("-r", "--Reconnect", type="int", dest="maxReconnect", default=1)
    parser.add_option("-D", "--UDP", type="int", dest="UDP", default=None)
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False)
    parser.add_option("-f", "--outputFile", type="string", dest="outputFile", default=None)
    parser.add_option("-m", "--maxtime", type="int", dest="maxConnectTime", default=None)
    parser.add_option("--Header", action="store_true", dest="headerOutput", default=False)
    parser.add_option("--HeaderFile", type="string", dest="headerFile", default=None)
    (options, args) = parser.parse_args()

    ntripArgs = {}
    ntripArgs['lat'] = options.lat
    ntripArgs['lon'] = options.lon
    ntripArgs['height'] = options.height
    ntripArgs['host'] = options.host

    if options.ssl:
        import ssl
        ntripArgs['ssl'] = True
    else:
        ntripArgs['ssl'] = False

    if options.org:
        if len(args) != 1:
            print("Incorrect number of arguments for IBSS.\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user + "." + options.org + ":" + options.password
        if options.baseorg:
            ntripArgs['caster'] = options.baseorg + ".ibss.trimbleos.com"
        else:
            ntripArgs['caster'] = options.org + ".ibss.trimbleos.com"
        ntripArgs['port'] = 52101 if options.ssl else 2101
        ntripArgs['mountpoint'] = args[0]
    else:
        if len(args) != 3:
            print("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user + ":" + options.password
        ntripArgs['caster'] = args[0]
        ntripArgs['port'] = int(args[1])
        ntripArgs['mountpoint'] = args[2]

    if ntripArgs['mountpoint'][0:1] != "/":
        ntripArgs['mountpoint'] = "/" + ntripArgs['mountpoint']

    ntripArgs['V2'] = options.V2
    ntripArgs['verbose'] = options.verbose
    ntripArgs['headerOutput'] = options.headerOutput

    if options.UDP:
        ntripArgs['UDP_Port'] = int(options.UDP)

    maxReconnect = options.maxReconnect
    maxConnectTime = options.maxConnectTime

    if options.verbose:
        print("Server: " + ntripArgs['caster'])
        print("Port: " + str(ntripArgs['port']))
        print("User: " + ntripArgs['user'])
        print("Mountpoint: " + ntripArgs['mountpoint'])
        print("Reconnects: " + str(maxReconnect))
        print("Max Connect Time: " + str(maxConnectTime))
        print("NTRIP: V2" if ntripArgs['V2'] else "NTRIP: V1")
        print("SSL Connection" if ntripArgs["ssl"] else "Unencrypted Connection")
        print("")

    fileOutput = False
    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out'] = f
        ntripArgs['gps_file_path'] = options.outputFile
        fileOutput = True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile'] = h
        ntripArgs['headerOutput'] = True
        print("Max Connect Time: " + str(maxConnectTime))
        print("NTRIP: V2" if ntripArgs['V2'] else "NTRIP: V1")
        print("SSL Connection" if ntripArgs["ssl"] else "Unencrypted Connection")
        print("")

    fileOutput = False
    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out'] = f
        ntripArgs['gps_file_path'] = options.outputFile
        fileOutput = True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile'] = h
        ntripArgs['headerOutput'] = True

    rclpy.init(args=None)
    node = NtripClient(**ntripArgs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.init(args=None)
    node = NtripClient(**ntripArgs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3 
"""  
This is heavily based on the NtripPerlClient program written by BKG.
Then heavily based on a unavco original.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
#!/home/username/env/bin/python3(add this if the above shebang does not work(in coordinate.py as well))
"""

from rclpy.node import Node
from nmea_msgs.msg import Sentence
from optparse import OptionParser
from pynmeagps import NMEAReader
import socket
import sys
import datetime
import base64
import time
import rclpy
import ssl 
import threading
import serial


version=0.2
useragent="NTRIP JCMBsoftPythonClient/%.1f" % version

# reconnect parameter (fixed values):
factor=2
maxReconnect=1
maxReconnectTime=1200
sleepTime=1


class NtripClient(Node):
    def __init__(self,
                 buffer=1024,
                 user="",
                 out=sys.stdout,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=46,
                 lon=122,
                 height=1212,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0, 
                 fileOutput=False,
                 gps_file_path=None
                ):
        super().__init__('nmea_publisher')
        self.nmea_pub = self.create_publisher(Sentence, 'nmea_sentence', 10)
        self.buffer = buffer
        self.user = base64.b64encode(bytes(user, 'utf-8')).decode("utf-8")
        self.out = out
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.height = height
        self.verbose = verbose
        self.ssl = ssl
        self.host = host
        self.UDP_Port = UDP_Port
        self.V2 = V2
        self.headerFile = headerFile
        self.headerOutput = headerOutput
        self.maxConnectTime = maxConnectTime
        self.maxReconnect = maxReconnect
        self.fileOutput = fileOutput
        self.socket = None
        self.latest_gga = None          
        self.latest_gga_lock = threading.Lock() 
        self.setPosition(lat, lon)

        try:
            self.stream = serial.Serial('/dev/ttyS0', 115200, timeout=1)  
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            sys.exit(1)
        self.nmr = NMEAReader(self.stream)

        self.gps_file_path = gps_file_path
        if self.gps_file_path:
            self.gps_file = open(self.gps_file_path, 'ab')
        else:
            self.gps_file = None

        if UDP_Port:
            self.UDP_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.UDP_socket.bind(('', 0))
            self.UDP_socket.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        else:
            self.UDP_socket = None

        # Dedicated thread for continuous NMEA reading at 1Hz
        threading.Thread(target=self._nmea_reader_loop, daemon=True).start()
        threading.Thread(target=self.readData, daemon=True).start()

    def _nmea_reader_loop(self):

        while rclpy.ok():
            try:
                raw_data, parsed_data = self.nmr.read()
                if not raw_data:
                    continue

                if b"GNGGA" in raw_data:
                   
                    with self.latest_gga_lock:
                        self.latest_gga = raw_data

                    msg = Sentence()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    try:
                        msg.sentence = raw_data.decode('ascii').strip()
                    except UnicodeDecodeError:
                        msg.sentence = raw_data.decode('utf-8', errors='replace').strip()
                    self.nmea_pub.publish(msg)

                    print(raw_data)
                    location = self.parse_gngga_sentence(raw_data.decode('ascii', errors='replace'))
                    if location:
                        lat, lon, _, fix_quality = location
                        fix_status = {
                            0: "Invalid",
                            1: "GPS fix",
                            2: "DGPS fix",
                            3: "PPS fix",
                            4: "RTK fixed",
                            5: "RTK float",
                            6: "Estimated",
                            7: "Manual input",
                            8: "Simulation"
                        }.get(fix_quality, "Unknown")
                        print(f"Parsed Location: Latitude={lat:.8f}, Longitude={lon:.8f}, Fix Status={fix_status}")

                    if self.gps_file:
                        self.gps_file.write(raw_data)
                        self.gps_file.flush()

            except Exception as e:
                self.get_logger().warn(f"NMEA read error: {e}")
                time.sleep(0.1)

    def setPosition(self, lat, lon):
        self.flagN = "N"
        self.flagE = "E"
        if lon > 180:
            lon = (lon - 360) * -1
            self.flagE = "W"
        elif (lon < 0 and lon >= -180):
            lon = lon * -1
            self.flagE = "W"
        elif lon < -180:
            lon = lon + 360
            self.flagE = "E"
        self.lon = lon
        if lat < 0:
            lat = lat * -1
            self.flagN = "S"
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon - self.lonDeg) * 60
        self.latMin = (lat - self.latDeg) * 60

    def getMountPointBytes(self):
        mountPointString = "GET %s HTTP/1.1\r\nUser-Agent: %s\r\nAuthorization: Basic %s\r\n" % (
            self.mountpoint, useragent, self.user)
        if self.host or self.V2:
            hostString = "Host: %s:%i\r\n" % (self.caster, self.port)
            mountPointString += hostString
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        if self.verbose:
            print(mountPointString)
        return bytes(mountPointString, 'ascii')

    def getGGABytes(self):
        timeout = 10  
        start = time.time()
        while time.time() - start < timeout:
            with self.latest_gga_lock:
                if self.latest_gga is not None:
                    return self.latest_gga
            time.sleep(0.1)
        self.get_logger().warn("Timeout waiting for GGA sentence")
        return b""

    def parse_gngga_sentence(self, nmea_sentence):
        parts = nmea_sentence.split(',')
        if len(parts) < 10:
            return None
        try:
            lat = float(parts[2][:2]) + float(parts[2][2:]) / 60.0
            if parts[3] == 'S':
                lat = -lat
            lon = float(parts[4][:3]) + float(parts[4][3:]) / 60.0
            if parts[5] == 'W':
                lon = -lon
            alt = float(parts[9]) if parts[9] else 0.0
            fix_quality = int(parts[6]) if parts[6].isdigit() else 0
        except ValueError:
            return None
        return lat, lon, alt, fix_quality

    def readData(self):
        reconnectTry = 1
        sleepTime = 1
        if self.maxConnectTime > 0:
            EndConnect = datetime.timedelta(seconds=self.maxConnectTime)

        while rclpy.ok() and reconnectTry <= self.maxReconnect:
            found_header = False
            if self.verbose:
                sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry, self.maxReconnect))

            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            if self.ssl:
                self.socket = ssl.wrap_socket(self.socket)

            error_indicator = self.socket.connect_ex((self.caster, self.port))
            if error_indicator == 0:
                sleepTime = 1
                connectTime = datetime.datetime.now()
                self.socket.settimeout(10)
                self.socket.sendall(self.getMountPointBytes())

                while not found_header:
                    casterResponse = self.socket.recv(4096)
                    header_lines = casterResponse.decode('utf-8').split("\r\n")

                    for line in header_lines:
                        if line == "":
                            if not found_header:
                                found_header = True
                                if self.verbose:
                                    sys.stderr.write("End Of Header\n")
                        else:
                            if self.verbose:
                                sys.stderr.write("Header: " + line + "\n")
                        if self.headerOutput:
                            self.headerFile.write(line + "\n")

                    for line in header_lines:
                        if line.find("SOURCETABLE") >= 0:
                            sys.stderr.write("Mount point does not exist")
                            sys.exit(1)
                        elif line.find("401 Unauthorized") >= 0:
                            sys.stderr.write("Unauthorized request\n")
                            sys.exit(1)
                        elif line.find("404 Not Found") >= 0:
                            sys.stderr.write("Mount Point does not exist\n")
                            sys.exit(2)
                        elif line.find("ICY 200 OK") >= 0:
                            self.socket.sendall(self.getGGABytes())
                        elif line.find("HTTP/1.0 200 OK") >= 0:
                            self.socket.sendall(self.getGGABytes())
                        elif line.find("HTTP/1.1 200 OK") >= 0:
                            self.socket.sendall(self.getGGABytes())

                data = "Initial data"
                while data:
                    try:
                        data = self.socket.recv(self.buffer)
                        if not data:
                            break
                        self.stream.write(data)

                        if self.UDP_socket:
                            self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))

                        if self.maxConnectTime:
                            if datetime.datetime.now() > connectTime + EndConnect:
                                if self.verbose:
                                    sys.stderr.write("Connection Timed exceeded\n")
                                sys.exit(0)

                    except socket.timeout:
                        if self.verbose:
                            sys.stderr.write('Connection TimedOut\n')
                        data = False
                    except socket.error:
                        if self.verbose:
                            sys.stderr.write('Connection Error\n')
                        data = False

                if self.verbose:
                    sys.stderr.write('Closing Connection\n')
                self.socket.close()
                self.socket = None

                if reconnectTry < self.maxReconnect:
                    sys.stderr.write("%s No Connection to NtripCaster. Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                    time.sleep(sleepTime)
                    sleepTime *= factor
                    if sleepTime > maxReconnectTime:
                        sleepTime = maxReconnectTime
                else:
                    sys.exit(1)

                reconnectTry += 1
            else:
                self.socket = None
                if self.verbose:
                    print("Error indicator: ", error_indicator)

                if reconnectTry < self.maxReconnect:
                    sys.stderr.write("%s No Connection to NtripCaster. Trying again in %i seconds\n" % (
                        datetime.datetime.now(), sleepTime))
                    time.sleep(sleepTime)
                    sleepTime *= factor
                    if sleepTime > maxReconnectTime:
                        sleepTime = maxReconnectTime
                reconnectTry += 1

    def shutdown(self):
        if self.socket:
            self.socket.close()
        if self.fileOutput and self.out not in (sys.stdout, None):
            self.out.close()
        if self.headerOutput and self.headerFile not in (sys.stderr, None):
            self.headerFile.close()
        if self.stream and self.stream.is_open:
            self.stream.close()
        sys.exit(0)


def main(args=None):
    usage = "NtripClient.py [options] [caster] [port] mountpoint"
    parser = OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user", type="string", dest="user", default="IBS")
    parser.add_option("-p", "--password", type="string", dest="password", default="IBS")
    parser.add_option("-o", "--org", type="string", dest="org")
    parser.add_option("-b", "--baseorg", type="string", dest="baseorg")
    parser.add_option("-t", "--latitude", type="float", dest="lat", default=50.09)
    parser.add_option("-g", "--longitude", type="float", dest="lon", default=8.66)
    parser.add_option("-e", "--height", type="float", dest="height", default=1200)
    parser.add_option("-v", "--verbose", action="store_true", dest="verbose", default=True)
    parser.add_option("-s", "--ssl", action="store_true", dest="ssl", default=False)
    parser.add_option("-H", "--host", action="store_true", dest="host", default=False)
    parser.add_option("-r", "--Reconnect", type="int", dest="maxReconnect", default=1)
    parser.add_option("-D", "--UDP", type="int", dest="UDP", default=None)
    parser.add_option("-2", "--V2", action="store_true", dest="V2", default=False)
    parser.add_option("-f", "--outputFile", type="string", dest="outputFile", default=None)
    parser.add_option("-m", "--maxtime", type="int", dest="maxConnectTime", default=None)
    parser.add_option("--Header", action="store_true", dest="headerOutput", default=False)
    parser.add_option("--HeaderFile", type="string", dest="headerFile", default=None)
    (options, args) = parser.parse_args()

    ntripArgs = {}
    ntripArgs['lat'] = options.lat
    ntripArgs['lon'] = options.lon
    ntripArgs['height'] = options.height
    ntripArgs['host'] = options.host

    if options.ssl:
        import ssl
        ntripArgs['ssl'] = True
    else:
        ntripArgs['ssl'] = False

    if options.org:
        if len(args) != 1:
            print("Incorrect number of arguments for IBSS.\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user + "." + options.org + ":" + options.password
        if options.baseorg:
            ntripArgs['caster'] = options.baseorg + ".ibss.trimbleos.com"
        else:
            ntripArgs['caster'] = options.org + ".ibss.trimbleos.com"
        ntripArgs['port'] = 52101 if options.ssl else 2101
        ntripArgs['mountpoint'] = args[0]
    else:
        if len(args) != 3:
            print("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user'] = options.user + ":" + options.password
        ntripArgs['caster'] = args[0]
        ntripArgs['port'] = int(args[1])
        ntripArgs['mountpoint'] = args[2]

    if ntripArgs['mountpoint'][0:1] != "/":
        ntripArgs['mountpoint'] = "/" + ntripArgs['mountpoint']

    ntripArgs['V2'] = options.V2
    ntripArgs['verbose'] = options.verbose
    ntripArgs['headerOutput'] = options.headerOutput

    if options.UDP:
        ntripArgs['UDP_Port'] = int(options.UDP)

    maxReconnect = options.maxReconnect
    maxConnectTime = options.maxConnectTime

    if options.verbose:
        print("Server: " + ntripArgs['caster'])
        print("Port: " + str(ntripArgs['port']))
        print("User: " + ntripArgs['user'])
        print("Mountpoint: " + ntripArgs['mountpoint'])
        print("Reconnects: " + str(maxReconnect))
        print("Max Connect Time: " + str(maxConnectTime))
        print("NTRIP: V2" if ntripArgs['V2'] else "NTRIP: V1")
        print("SSL Connection" if ntripArgs["ssl"] else "Unencrypted Connection")
        print("")

    fileOutput = False
    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out'] = f
        ntripArgs['gps_file_path'] = options.outputFile
        fileOutput = True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile'] = h
        ntripArgs['headerOutput'] = True
        print("Max Connect Time: " + str(maxConnectTime))
        print("NTRIP: V2" if ntripArgs['V2'] else "NTRIP: V1")
        print("SSL Connection" if ntripArgs["ssl"] else "Unencrypted Connection")
        print("")

    fileOutput = False
    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out'] = f
        ntripArgs['gps_file_path'] = options.outputFile
        fileOutput = True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile'] = h
        ntripArgs['headerOutput'] = True

    rclpy.init(args=None)
    node = NtripClient(**ntripArgs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.init(args=None)
    node = NtripClient(**ntripArgs)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.shutdown()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

