function Decoder(bytes, port) { 
  var epochtime = ((bytes[3] << 24) | (bytes[2] << 16) | (bytes[1] << 8) | bytes[0] );
  var temp = ((bytes[5] << 8) | bytes[4])/10.0;
  var humidity = ((bytes[7] << 8) | bytes[6])/10.0;
  var baropress = ((bytes[9] << 8) | bytes[8])/10.0 + 1000.0;
  var rainfall = ((bytes[11] << 8) | bytes[10])/10.0;
  var windspeed = ((bytes[13] << 8) | bytes[12])/10.0;
  var winddirn = (bytes[15] << 8) | bytes[14];
  var lat = -38.293;
  var long = 144.996;
  var height = 105;


  return {
    field1: epochtime,
    field2: temp,
    field3: humidity,
    field4: baropress,
    field5: rainfall,
    field6: windspeed,
    field7: winddirn
  };
}
