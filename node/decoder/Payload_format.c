function Decoder(bytes, port) {
  // Decode an uplink message from a buffer
  // (array) of bytes to an object of fields.
  var decoded = {};
//DO 
dis =  (bytes[2]<<16) | (bytes[1]<<8) |bytes[0];
decoded.DO = dis*0.01;
//turbidity
vol=  (bytes[6]<<16) | (bytes[5]<<8) |bytes[4];
vol1= vol*0.01;decoded.value = vol1;
x= (0.327309*vol1)+2.493455;decoded.val=x;
tur = (-1120.4*(x*x))+(5742.3*x)-(4352.9);
decoded.turbidity = tur ;
//temp & hum
t1 = (bytes[10]<<16) | (bytes[9]<<8) |bytes[8];
h1 = (bytes[14]<<16) | (bytes[13]<<8) |bytes[12];
if (t1< 180 && h1 << 10000)
{
  decoded.temp =t1;
  decoded.hum =h1;
  }
  else {decoded.temp = 20;
  decoded.hum =60;}

  return decoded;
}
