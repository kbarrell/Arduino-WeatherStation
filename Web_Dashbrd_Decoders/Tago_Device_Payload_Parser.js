/* This is a default example for payload parser.
** The ignore_vars variable in this code should be used to ignore variables
** from the device that you don't want.
**
** Testing:
** You can do manual tests to this parse by using the Device Emulator. Copy and Paste the following code:
** [{ "variable": "payload", "value": "0109611395" }]
**
*/
// Add ignorable variables in this array (blacklist).
const black_list = ['port','data_rate','frequency','dev_id','counter','is_retry','snr','decoded_payload','downlink_url','rssi','ttn_payload','hardware_serial','gtw_id','coding_rate','location_source','altitude'];
//
//  Leaves the following variables (whitelist - info only) to be passed through filter
//  variables white_list = ['field1', 'field2', 'field3', 'field4', 'field5', 'field6', 'field7', 'field8', 'field9','timestamp','devicetemp','latitude', 'longitude' ]

// Remove the unwanted variables.
payload = payload.filter(x => !black_list.includes(x.variable));

// Set up map locations to be used for plotting wind directions on map widget
const compassPts = [
  ['N',	-38.24760362,	144.996206],
  ['NNE',	-38.25102673,	145.0181455],
  ['NE',	-38.26077578,	145.0367449],
  ['ENE', -38.27536873,	145.0491727],
  ['E', -38.29258605,	145.0535367],
  ['ESE', -38.30980746,	145.0491727],
  ['SE', -38.32441027,	145.0367449],
  ['SSE', -38.33416919,	145.0181455],
  ['S', -38.33759638,	144.996206],
  ['SSW', -38.33416919,	144.9742665],
  ['SW', -38.32441027,	144.9556671],
  ['WSW', -38.30980746,	144.9432393],
  ['W', -38.29258605,	144.9388753],
  ['WNW', -38.27536873,	144.9432393],
  ['NW', -38.26077578,	144.9556671],
  ['NNW', -38.25102673,	144.9742665],
  ['GTW', -38.292618, 144.99622000]
];
var index = 0;
//  Find compass point for reported wind direction 
const windDir = payload.find(x => x.variable === "field7");
const gtw_lat = payload.find(x => x.variable === "latitude");
const gtw_lng = payload.find(x => x.variable === "longitude");
var normalDir = windDir.value;
if (normalDir < 0) {
  normalDir = normalDir + 360; 
}
if (normalDir > 360) {
  normalDir = normalDir -360;
}
if (normalDir < 349) {
  index = Math.round((normalDir)/22.5);
}

//  Add the mapping point variables to the decoded payload output
payload.push({"variable":"winddirloc","value": compassPts[index][0], "location":{"lat":compassPts[index][1],"lng":compassPts[index][2]}});
// As site location is static, following could easily be set from compassPts array, but leave it general as delivered through TTN
payload.push({"variable":"gtw_location","value": compassPts[16][0], "location":{"lat":gtw_lat.value,"lng":gtw_lng.value}});
