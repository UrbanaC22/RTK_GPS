#!/usr/bin/env -S python3 -u
"""
RTK NTRIP Client with WebSocket transmitter.
Sends parsed GPS/RTK data as JSON over WebSocket to a receiver node.

Based on the original NtripPerlClient by BKG / UNAVCO.
GPL v3 License.
"""

import socket
import sys
import datetime
import base64
import time
import json
import threading
import asyncio
import websockets

from optparse import OptionParser

import serial
from pynmeagps import NMEAReader
from pynmeagps.exceptions import NMEAParseError

version = 0.3
useragent = "NTRIP JCMBsoftPythonClient/%.1f" % version

# Reconnect parameters
factor = 3
maxReconnect = 1
maxReconnectTime = 1200
sleepTime = 3

# ── WebSocket configuration ───────────────────────────────────────────────────
WS_HOST = "192.168.1.100"   # Receiver IP
WS_PORT = 9090              # Must match receiver


class WebSocketSender:
    """
    Maintains a persistent async WebSocket connection and exposes a
    thread-safe send_gps() method so the synchronous NTRIP loop can
    push messages without blocking.
    """

    def __init__(self, host: str, port: int):
        self.uri = f"ws://{host}:{port}"
        self._loop = asyncio.new_event_loop()
        self._ws = None
        self._queue: asyncio.Queue = None          # created inside the loop
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    # ── internal async helpers ────────────────────────────────────────────

    async def _connect_and_drain(self):
        self._queue = asyncio.Queue()
        while True:                                # outer: reconnect loop
            try:
                print(f"[WS] Connecting to {self.uri} …")
                async with websockets.connect(self.uri) as ws:
                    self._ws = ws
                    print(f"[WS] Connected to {self.uri}")
                    while True:                    # inner: drain queue
                        payload = await self._queue.get()
                        try:
                            await ws.send(payload)
                        except websockets.ConnectionClosed:
                            # Put the failed message back and reconnect
                            await self._queue.put(payload)
                            break
            except Exception as e:
                self._ws = None
                print(f"[WS] Connection failed: {e}. Retrying in 3 s …")
                await asyncio.sleep(3)

    def _run_loop(self):
        self._loop.run_until_complete(self._connect_and_drain())

    # ── public thread-safe API ────────────────────────────────────────────

    def send_gps(self, payload: dict):
        """Called from the main (synchronous) thread."""
        if self._queue is None:
            return                                 # not ready yet
        msg = json.dumps(payload)
        # Schedule the enqueue on the asyncio loop from outside
        self._loop.call_soon_threadsafe(self._queue.put_nowait, msg)


# ── NTRIP client ──────────────────────────────────────────────────────────────

class NtripClient(object):
    def __init__(self,
                 buffer=50,
                 user="",
                 out=sys.stdout,
                 port=2101,
                 caster="",
                 mountpoint="",
                 host=False,
                 lat=50,
                 lon=19,
                 height=300,
                 ssl=False,
                 verbose=False,
                 UDP_Port=None,
                 V2=False,
                 headerFile=sys.stderr,
                 headerOutput=False,
                 maxConnectTime=0,
                 gps_file_path=None,
                 ws_sender: WebSocketSender = None
                 ):
        self.buffer = buffer
        self.user = base64.b64encode(bytes(user, 'utf-8')).decode("utf-8")
        self.out = out
        self.port = port
        self.caster = caster
        self.mountpoint = mountpoint
        self.setPosition(lat, lon)
        self.height = height
        self.verbose = verbose
        self.ssl = ssl
        self.host = host
        self.UDP_Port = UDP_Port
        self.V2 = V2
        self.headerFile = headerFile
        self.headerOutput = headerOutput
        self.maxConnectTime = maxConnectTime
        self.socket = None
        self.ws_sender = ws_sender

        self.stream = serial.Serial('/dev/ttyTHS1', 115200, timeout=3)
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
        else:
            self.lon = lon
        if lat < 0:
            lat = lat * -1
            self.flagN = "S"
        self.lonDeg = int(lon)
        self.latDeg = int(lat)
        self.lonMin = (lon - self.lonDeg) * 60
        self.latMin = (lat - self.latDeg) * 60

    def getMountPointBytes(self):
        mountPointString = (
            "GET %s HTTP/1.1\r\n"
            "User-Agent: %s\r\n"
            "Authorization: Basic %s\r\n"
        ) % (self.mountpoint, useragent, self.user)
        if self.host or self.V2:
            mountPointString += "Host: %s:%i\r\n" % (self.caster, self.port)
        if self.V2:
            mountPointString += "Ntrip-Version: Ntrip/2.0\r\n"
        mountPointString += "\r\n"
        if self.verbose:
            print(mountPointString)
        return bytes(mountPointString, 'ascii')

    def getGGABytes(self):
        while True:
            (raw_data, parsed_data) = self.nmr.read()
            if bytes("GNGGA", 'ascii') in raw_data:
                return raw_data

    def __del__(self):
        if self.gps_file:
            self.gps_file.close()

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
            num_sats = int(parts[7]) if parts[7].isdigit() else 0
            hdop = float(parts[8]) if parts[8] else 0.0
            # UTC time string e.g. "033728.000"
            utc_str = parts[1]
        except (ValueError, IndexError):
            return None
        return lat, lon, alt, fix_quality, num_sats, hdop, utc_str

    def readData(self):
        reconnectTry = 1
        sleepTime = 1
        if self.maxConnectTime > 0:
            EndConnect = datetime.timedelta(seconds=maxConnectTime)
        try:
            while reconnectTry <= maxReconnect:
                time.sleep(1)
                found_header = False
                if self.verbose:
                    sys.stderr.write('Connection {0} of {1}\n'.format(reconnectTry, maxReconnect))

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
                            elif (line.find("ICY 200 OK") >= 0
                                  or line.find("HTTP/1.0 200 OK") >= 0
                                  or line.find("HTTP/1.1 200 OK") >= 0):
                                self.socket.sendall(self.getGGABytes())

                    data = "Initial data"
                    while data:
                        try:
                            data = self.socket.recv(self.buffer)
                            self.stream.write(data)

                            try:
                                raw_data, parsed_data = self.nmr.read()
                            except NMEAParseError:
                                continue
                            except Exception as e:
                                print("NMEA read error:", e)
                                continue

                            if bytes("GNGGA", 'ascii') in raw_data:
                                print(raw_data)
                                location = self.parse_gngga_sentence(raw_data.decode('ascii'))
                                if location:
                                    lat, lon, alt, fix_quality, num_sats, hdop, utc_str = location
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

                                    print(f"Parsed Location: Latitude={lat}, "
                                          f"Longitude={lon}, Fix Status={fix_status}")

                                    # ── Send over WebSocket ───────────────
                                    if self.ws_sender:
                                        payload = {
                                            "latitude":    lat,
                                            "longitude":   lon,
                                            "altitude":    alt,
                                            "fix_quality": fix_quality,
                                            "fix_status":  fix_status,
                                            "num_sats":    num_sats,
                                            "hdop":        hdop,
                                            "utc":         utc_str,
                                            "raw_nmea":    raw_data.decode('ascii').strip(),
                                            "timestamp":   time.time()
                                        }
                                        self.ws_sender.send_gps(payload)

                                if self.gps_file:
                                    self.gps_file.write(raw_data)
                                    self.gps_file.flush()

                            if self.UDP_socket:
                                self.UDP_socket.sendto(data, ('<broadcast>', self.UDP_Port))

                            if maxConnectTime:
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

                    if reconnectTry < maxReconnect:
                        sys.stderr.write(
                            "%s No Connection to NtripCaster. Trying again in %i seconds\n"
                            % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime = factor
                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime
                    else:
                        sys.exit(1)

                    reconnectTry += 1

                else:
                    self.socket = None
                    if self.verbose:
                        print("Error indicator: ", error_indicator)
                    if reconnectTry < maxReconnect:
                        sys.stderr.write(
                            "%s No Connection to NtripCaster. Trying again in %i seconds\n"
                            % (datetime.datetime.now(), sleepTime))
                        time.sleep(sleepTime)
                        sleepTime = factor
                        if sleepTime > maxReconnectTime:
                            sleepTime = maxReconnectTime
                    reconnectTry += 1

        except KeyboardInterrupt:
            if self.socket:
                self.socket.close()
            self.stream.close()
            sys.exit()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == '__main__':
    usage = "rtk_check.py [options] [caster] [port] mountpoint"
    parser = OptionParser(version=version, usage=usage)
    parser.add_option("-u", "--user",         type="string", dest="user",           default="IBS")
    parser.add_option("-p", "--password",     type="string", dest="password",       default="IBS")
    parser.add_option("-o", "--org",          type="string", dest="org")
    parser.add_option("-b", "--baseorg",      type="string", dest="baseorg")
    parser.add_option("-t", "--latitude",     type="float",  dest="lat",            default=50.09)
    parser.add_option("-g", "--longitude",    type="float",  dest="lon",            default=8.66)
    parser.add_option("-e", "--height",       type="float",  dest="height",         default=1200)
    parser.add_option("-v", "--verbose",      action="store_true", dest="verbose",  default=True)
    parser.add_option("-s", "--ssl",          action="store_true", dest="ssl",      default=False)
    parser.add_option("-H", "--host",         action="store_true", dest="host",     default=False)
    parser.add_option("-r", "--Reconnect",    type="int",    dest="maxReconnect",   default=1)
    parser.add_option("-D", "--UDP",          type="int",    dest="UDP",            default=None)
    parser.add_option("-2", "--V2",           action="store_true", dest="V2",       default=False)
    parser.add_option("-f", "--outputFile",   type="string", dest="outputFile",     default=None)
    parser.add_option("-m", "--maxtime",      type="int",    dest="maxConnectTime", default=None)
    parser.add_option("--Header",             action="store_true", dest="headerOutput", default=False)
    parser.add_option("--HeaderFile",         type="string", dest="headerFile",     default=None)
    # WebSocket options
    parser.add_option("--ws-host", type="string", dest="ws_host", default=WS_HOST,
                      help="WebSocket receiver IP. Default: %default")
    parser.add_option("--ws-port", type="int",    dest="ws_port", default=WS_PORT,
                      help="WebSocket receiver port. Default: %default")
    parser.add_option("--no-ws",   action="store_true", dest="no_ws", default=False,
                      help="Disable WebSocket sending")

    (options, args) = parser.parse_args()

    ntripArgs = {}
    ntripArgs['lat']    = options.lat
    ntripArgs['lon']    = options.lon
    ntripArgs['height'] = options.height
    ntripArgs['host']   = options.host

    if options.outputFile:
        ntripArgs['gps_file_path'] = options.outputFile

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
        ntripArgs['caster'] = (options.baseorg or options.org) + ".ibss.trimbleos.com"
        ntripArgs['port']   = 52101 if options.ssl else 2101
        ntripArgs['mountpoint'] = args[0]
    else:
        if len(args) != 3:
            print("Incorrect number of arguments for NTRIP\n")
            parser.print_help()
            sys.exit(1)
        ntripArgs['user']       = options.user + ":" + options.password
        ntripArgs['caster']     = args[0]
        ntripArgs['port']       = int(args[1])
        ntripArgs['mountpoint'] = args[2]

    if ntripArgs['mountpoint'][0:1] != "/":
        ntripArgs['mountpoint'] = "/" + ntripArgs['mountpoint']

    ntripArgs['V2']           = options.V2
    ntripArgs['verbose']      = options.verbose
    ntripArgs['headerOutput'] = options.headerOutput

    if options.UDP:
        ntripArgs['UDP_Port'] = int(options.UDP)

    maxReconnect    = options.maxReconnect
    maxConnectTime  = options.maxConnectTime

    # ── Start WebSocket sender thread ─────────────────────────────────────
    if not options.no_ws:
        ws_sender = WebSocketSender(options.ws_host, options.ws_port)
        ntripArgs['ws_sender'] = ws_sender
        print(f"[WS] Sender targeting ws://{options.ws_host}:{options.ws_port}")
    else:
        print("[WS] WebSocket sending disabled.")

    if options.verbose:
        print("Server: "           + ntripArgs['caster'])
        print("Port: "             + str(ntripArgs['port']))
        print("User: "             + ntripArgs['user'])
        print("mountpoint: "       + ntripArgs['mountpoint'])
        print("Reconnects: "       + str(maxReconnect))
        print("Max Connect Time: " + str(maxConnectTime))
        print("NTRIP: V2" if ntripArgs['V2'] else "NTRIP: V1")
        print("SSL Connection" if ntripArgs["ssl"] else "Unencrypted Connection")
        print("")

    fileOutput = False
    if options.outputFile:
        f = open(options.outputFile, 'wb')
        ntripArgs['out'] = f
        fileOutput = True

    if options.headerFile:
        h = open(options.headerFile, 'w')
        ntripArgs['headerFile']   = h
        ntripArgs['headerOutput'] = True

    n = NtripClient(**ntripArgs)
    try:
        n.readData()
    finally:
        if fileOutput:
            f.close()
        if options.headerFile:
            h.close()
