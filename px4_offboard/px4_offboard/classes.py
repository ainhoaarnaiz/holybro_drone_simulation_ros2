from strenum import StrEnum

class SetpointType(StrEnum):
    ALTITUDE = 'altitude'
    POSITION = 'position'
    VELOCITY = 'velocity'
    ATTITUDE = 'attitude'
    GPS = 'gps'
