/* ADLA3 AD-Node Decoder Base
	Includes Base Diagnostic, Analog, Digital and Accelerometer Values
	Suitable for AD-NODE Model revision ADLA3
	NOTE: Not suitable for ADLA1 and ADLA2 due to major hardware and firmware changes.
*/

//Change log
/*
20231128: R2-0
Toby Partridge ICT International
* Changed structure to little-endian to match new ADLA3 model with FW and change to ADC input designation, supported in firmware 2.4.1 and newer.

20250312: R2-1
Toby Partridge ICT International
* Removed reference to decoding port 0. port0 is use only for non-application data so must not be use in user data decoding.
* Removed array push by commenting out for x,y,z raw value lines with preference for the x-tilt-offset and y-tilt-offset instead.

20260128: R2-2
Toby Partridge ICT International
* Updated description for Analog input labels
* Removed reference to //Analog Eq, equations must run after at the database/dashboard platform.
* Commented out arr.push(["digital-count", 3, digital2, src]); and arr.push(["digital-count", 4, digital1, src]); because typically only two digital pulse counters are frequently used.
* Removed reference to //Digital Eq, equations must run after at the database/dashboard platform.
* Updated Port10, Port100 and unknown response sections to remove product-id, batchnumber and raw-payload. //arr.push(["raw-payload", 0, buf.slice(0, buf.length), "unknown"]);
*/

// Structure Type Define, 'nested' or 'flat'
var TYPE = 'nested';

/*Class Buffer
	Purpose: A psuedo buffer class for accessing packet data,
		allows for uniformity between decoder types
*/
function Buf(buf){this.pl=buf; this.length=this.pl.length;}
Buf.prototype.readUInt8=function(ofs){return ((this.pl[ofs]<<24)>>>24);};
Buf.prototype.readUInt16BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16)>>>16);};
Buf.prototype.readUInt32BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16|this.pl[ofs++]<<8|this.pl[ofs++])>>>0);};
Buf.prototype.readUInt16LE=function(ofs){return ((this.pl[ofs+1]<<24|this.pl[ofs--]<<16)>>>16);};
Buf.prototype.readUInt32LE=function(ofs){return ((this.pl[ofs+3]<<24|this.pl[ofs+2]<<16|this.pl[ofs+1]<<8|this.pl[ofs])>>>0);};
Buf.prototype.readInt8=function(ofs){return ((this.pl[ofs]<<24)>>24);};
Buf.prototype.readInt16BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16)>>16);};
Buf.prototype.readInt32BE=function(ofs){return ((this.pl[ofs++]<<24|this.pl[ofs++]<<16|this.pl[ofs++]<<8|this.pl[ofs++])>>0);};
Buf.prototype.readInt16LE=function(ofs){return ((this.pl[ofs+1]<<24|this.pl[ofs--]<<16)>>16);};
Buf.prototype.readInt32LE=function(ofs){return ((this.pl[ofs+3]<<24|this.pl[ofs+2]<<16|this.pl[ofs+1]<<8|this.pl[ofs])>>0);};
Buf.prototype.readFloatBE=function(ofs){return B2Fl(this.readUInt32BE(ofs));};
Buf.prototype.readFloatLE=function(ofs){return B2Fl(this.readUInt32LE(ofs));};
Buf.prototype.slice=function(s,e){return this.pl.slice(s,e);};
Buf.prototype.length=function(){return this.pl.length;};

/*Function Bytes2Float32(bytes)
	Purpose: Decodes an array of bytes(len 4(32 bit) into a float.
	Args:	bytes - an array of bytes, 4 bytes long
	Returns: 32bit Float representation
*/
function B2Fl(b){
	var sign =(b>>31)?-1:1;
	var exp=((b>>23)&0xFF)-127;
	var sig=(b&~(-1<<23));
	if(exp==128) return sign*((sig)?Number.NaN:Number.POSITIVE_INFINITY);
  if(exp==-127){
		if(sig===0) return sign*0.0;
		exp=-126;
		sig/=(1<<22);
	} else sig=(sig|(1<<23))/(1<<23);
	return sign*sig*Math.pow(2,exp);
}

/*Function buildNested(a)
	Purpose: Takes an array and parses them into a clean and succinct object of nested parameter sets
	Args: a - An array of arrays containing Parameter Sets
	Returns: An Object containing nested Parameter Sets
*/
function buildNested(a){
	var exc=["main","diagnostic","downlink","device_info","unknown"];
	var ret=[];
	for(var el in a){
		var e=a[el];
		var par={};
		par['label']=e[0];
		par['channelId']=e[1];
		par['value']=e[2];
		var e_length = e.length;
		if(e_length>3&&exc.indexOf(e[3])<0) par['source']=e[3];
		if(e_length>4) par['unit']=e[4];
		if(e_length>5) par['address']=e[5];
		ret.push(par);
	} return ret;}

/*Function buildFlat(a)
	Purpose: Takes an array and parses them into a clean and succinct object of flat parameters
	Args: a - An array of arrays containing Parameter Sets
	Returns: An Object containing nested Parameter Sets
*/
function buildFlat(a){
	var exc=["main","diagnostic","downlink","device_info","unknown"];
	var ret={};
	for(var el in a){
		var e=a[el];
		var label = '';
		if(e.length==6){
			e[0] = e[0]+e[5]+'_';
		}
		if(e.length<=4){
			label=(exc.indexOf(e[3])<0) ? (e[0]+e[1]) : (e[0]);
		} else{
			label=(exc.indexOf(e[3])<0) ? (e[0]+e[1]+'_'+e[4]) : (e[0]+'_'+e[4]);

		} ret[label]=e[2];
	} return ret;}

//Function - Decode, Wraps the primary decoder function for Chirpstack
function Decode(fPort, bytes){
	var buf = new Buf(bytes);
	var decoded = {};
	var readingsArr = primaryDecoder(buf, fPort);
	if(TYPE == 'flat'){ decoded = buildFlat(readingsArr); }
	else decoded['data'] = buildNested(readingsArr);
	
	return decoded;
}

//Function - parseDeviceMsg, Wraps the primary decoder function for NNNCo
function parseDeviceMsg(buf, lMsg){
	var p = lMsg.loraPort;
	var readingArr = primaryDecoder(buf, p);
	var readingList = buildNested(readingArr);
	return readingList;
}

//Function - Decoder, Wraps the primary decoder function for TTNv3
function Decoder(b, p){
	var buf = new Buf(b);
	var decoded = {};
	var readingsArr = primaryDecoder(buf, p);
	if(TYPE == 'flat'){ decoded = buildFlat(readingsArr); }
	else decoded['data'] = buildNested(readingsArr);
	
	return decoded;
}

/*Function primaryDecoder
	Purpose: Main Entry point of TTN Console Decoder
	Args:	bytes - An array of bytes from LoRaWan raw payload(Hex Represented)
			port - LoRaWan Port that the message came through(set by Definium firmware)
	Returns: decoded - An object with data fields as decoded parameter values
*/
function primaryDecoder(buf,p){
	var arr = [];
	var byte = 0;

	// Data Packet Response Received
	if(p === 1){
		var src = "main";
		arr.push(['packet-type', 0, "DATA_PACKET", src]);
		src = "diagnostic";
		
		arr.push(["uptime", 0, +(buf.readUInt32LE(byte)), src, "s"]);
		arr.push(["battery-voltage", 0, +((buf.readUInt16LE(byte = byte+4)/1000).toFixed(3)), src, "V"]);
		
		src = "adc";
		var current0 = +(buf.readUInt16LE(byte = byte+2)); //var to store uA  4-20mA input
		var voltage0 = +(buf.readUInt32LE(byte = byte+2)); //var to store uV0 ADC microVolt input "ADCV0"
		var voltage1 = +(buf.readUInt32LE(byte = byte+4)); //var to store uV1 ADC microVolt input "ADCV1"
		arr.push(["current-adc", 0, current0, src, "uA"]); //array pushed uA  4-20mA input
		arr.push(["voltage-adc", 0, voltage0, src, "uV"]); //array pushed uV0 ADC microVolt input "ADCV0 0-3.0v"
		arr.push(["voltage-adc", 1, voltage1, src, "uV"]); //array pushed uV1 ADC microVolt input "ADCV1 0-3.0v"
		arr.push(["temperature", 1, +((buf.readInt32LE(byte = byte+4)/1000).toFixed(3)), src, "C"]); //thermistor1 "T1" 
		//Note: T0 input is skipped on ADLA3 models, the channel is repurposed for ADC voltage input "V1" and firmware locked.
		
		src = "digital";
		var digital4 = +(buf.readUInt32LE(byte=byte+4));
		var digital3 = +(buf.readUInt32LE(byte=byte+4));
		var digital2 = +(buf.readUInt32LE(byte=byte+4));
		var digital1 = +(buf.readUInt32LE(byte=byte+4));
		arr.push(["digital-count", 1, digital4, src]);
		arr.push(["digital-count", 2, digital3, src]);
		//arr.push(["digital-count", 3, digital2, src]); // Uncomment this line if pulse counter 3 is required.
		//arr.push(["digital-count", 4, digital1, src]); // Uncomment this line if pulse counter 4 is required.
		
		// Accelerometer values
		var mult = (2.0 / 32678.0) * 1000;
		src = "main";	
	    var x = +((buf.readInt16LE(byte = byte+4) * mult).toFixed(3));
		var y = +((buf.readInt16LE(byte = byte+2) * mult).toFixed(3));
		var z = +((buf.readInt16LE(byte = byte+2) * mult).toFixed(3));
		//arr.push(["accelerometer-x", 0, x, src, "mG"]); Commented out to reduce MQTT payload unnecessary values.  
		//arr.push(["accelerometer-y", 0, y, src, "mG"]); Commented out to reduce MQTT payload unnecessary values.  
		//arr.push(["accelerometer-z", 0, z, src, "mG"]); Commented out to reduce MQTT payload unnecessary values.  
		var roll = Math.abs(Math.atan((x)/(y))*(180/Math.PI));
		var pitch = Math.abs(Math.atan((x)/(z))*(180/Math.PI));
		var x_offset = +((y < 0) ? (0 + (90-roll)).toFixed(3) : (0 - (90-roll)).toFixed(3));
		var y_offset = +((z < 0) ? (0 + (90-pitch)).toFixed(3) : (0 - (90-pitch)).toFixed(3));
		arr.push(["x-tilt-offset", 0, x_offset, src, "Degrees"]);
		arr.push(["y-tilt-offset", 0, y_offset, src, "Degrees"]);
	} else if(p === 10){
		//Device Info Packet Recieved
		var src = "device_info";
		arr.push(["packet-type", 0, "DEVICE_INFO", "main"]);
		var productid = (["product-id", 0, buf.readUInt32BE(byte), src]);
		var batchnumber = (["batch-number", 0, buf.readUInt32BE(byte=byte+4), src]);
		arr.push(["software-version", 0, buf.readUInt32BE(byte=byte+4), src]);
	} else if(p === 100){
		//Downlink Response Packet Recieved
		arr.push(["packet-type", 0, "DOWNLINK_RESPONSE", "main"]);
		arr.push(["downlink-response", 0, String.fromCharCode.apply(String, buf.slice(0, buf.length)), "downlink"]);
	} else{
		// Unknown Response Recieved
		arr.push(["packet-type", 0, "UNKNOWN_RESPONSE", "main"]);
		//arr.push(["raw-payload", 0, buf.slice(0, buf.length), "unknown"]);
	}
	return arr;
}