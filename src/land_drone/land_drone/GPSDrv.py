import serial

class GPSDrv:
    serial_port = "/dev/ttyS0"
    baud_rate = 9600

    def __init__(self):
        pass

    def parse_gngga(self, gngga_sentence):
        data = gngga_sentence.split(',')
        if data[2] and data[4] and data[9] and data[11]:
            latitude = float(data[2][:2]) + float(data[2][2:]) / 60.0
            if data[3] == 'S':
                latitude = -latitude
            longitude = float(data[4][:3]) + float(data[4][3:]) / 60.0
            if data[5] == 'W':
                longitude = -longitude
            altitude = float(data[9])
            return latitude, longitude, altitude
        

    def get_coordinates(self):
        while True:
            with serial.Serial(GPSDrv.serial_port, GPSDrv.baud_rate, timeout=1) as ser:
                try:
                    while True:
                        line = ser.readline().decode('utf-8').strip()
                        # print("line decode: {}".format(line))
                        if line.startswith('$GNGGA'):
                            coordinates = self.parse_gngga(line)
                            if coordinates:
                                return ("Latitude: {}, Longitude: {}, Altitude: {} meters".format(*coordinates))
                except UnicodeDecodeError:
                    # En cas d'erreur de décodage, ignorez la ligne et continuez
                    continue
