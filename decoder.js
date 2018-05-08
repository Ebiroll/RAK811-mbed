function Decoder(bytes, port) {
    // Decode an uplink message from a buffer
    // (array) of bytes to an object of fields.
    var decoded = {};

    switch(port) {
    case 2:
        decoded.lat = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
        decoded.lat = (decoded.lat / 16777215.0 * 180) - 90;
        decoded.lon = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
        decoded.lon = (decoded.lon / 16777215.0 * 360) - 180;
        var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
        var sign = bytes[6] & (1 << 7);
        if(sign) {
            decoded.alt = 0xFFFF0000 | altValue;
        } else {
            decoded.alt = altValue;
        }
        decoded.hdop = bytes[8] / 10.0;
	decoded.sats = bytes[9];
        break;

    case 3:
        var bat = (bytes[0] | bytes[1]<<8 | (bytes[1] & 0x80 ? 0xFFFF<<16 : 0)) * 10.0;

        decoded ={
            "battery":{
                "bat":bat
            }
        };
        break;

    case 4:
        var aX = (bytes[0] | bytes[1]<<8 | (bytes[1] & 0x80 ? 0xFFFF<<16 : 0)) / 10.0;
        var aY = (bytes[2] | bytes[3]<<8 | (bytes[3] & 0x80 ? 0xFFFF<<16 : 0)) / 10.0;
        var aZ = (bytes[4] | bytes[5]<<8 | (bytes[5] & 0x80 ? 0xFFFF<<16 : 0)) / 10.0;

        decoded ={
            "acceleration":{
                "aX":aX,
                "aY":aY,
                "aZ":aZ,
            }
        };
        break;
    }
    return decoded;
}
