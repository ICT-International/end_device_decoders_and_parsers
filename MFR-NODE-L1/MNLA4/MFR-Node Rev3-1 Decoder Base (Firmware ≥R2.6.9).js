/* MFR-Node Payload V2 Decoder Base
	Includes Base Diagnostic, Analog, Digital and SDI Section
	Suitable for MNLA3 & MNLA4 & MNLA5 800-900Mhz Global Nodes 
	& MNLB3 running 2.6.9 inclusive and newer firmware with Signed ADC readings.
	
	NOTE: For Eratos Connnections, comment out line 66: "if(e_length>5) par['address']=e[5];"
*/

//Change Log:
/* 
20260128: 
* Removed array push of payload version, charger fault, and charger state by commenting out.
* Added if/else statement to CS506 Frequency input code so that when there is no sensor present it does not parse a zero/0: var frequency = +(buf.readUInt32BE(byte=byte+2));.
* Updated Port10, Port100 and unknown response sections to remove product-id, batchnumber and raw-payload. //arr.push(["raw-payload", 0, buf.slice(0, buf.length), "unknown"]);
* Removed reference to //Analog Eq, equations must run after at the database/dashboard platform.
* Removed reference to //Digital Eq, equations must run after at the database/dashboard platform.
*/

// Structure Type Define, 'nested' or 'flat'
var TYPE = 'nested'; //Use nested for Eratos
//var Type = 'flat'; //Use flat for Meshed. Note: decoded data contains no Unit of Measure. 

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
		if(e_length>5) par['address']=e[5];		//Comment this segment out for Eratos Senaps Integration.
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

	//Data Packet Recieved
	if(p === 1){
		// Test Payload Bytes: 1010000170F40FB04E7200000000
		var src = "main";
		var charge_fault = buf.readUInt8(byte);
		var header = buf.readUInt8(byte=byte+1);
		byte = 2;
		arr.push(['packet-type', 0, "DATA_PACKET", src]);
		//arr.push(["payload-version", 0, +((charge_fault & 0xf0) >> 4), src]);
		//arr.push(["charging-state", 0, +(charge_fault & 1), src]);
		//arr.push(["fault", 0, +((charge_fault & 2) >> 1), src]);
		arr.push(["header", 0, +((((header / 16) >> 0) * 10) + (header % 16)), src]);
		
		if(buf.length < 3){
			return arr;
		}
		
		//Diagnostics Packet
		if(header == 0x10){
			src = "diagnostic";
			arr.push(["uptime", 0, +(buf.readUInt32BE(byte)), src, "s"]);
			arr.push(["battery-voltage", 0, +((buf.readUInt16BE(byte = byte+4)/1000).toFixed(3)), src, "V"]);
			arr.push(["solar-voltage", 0, +((buf.readUInt16BE(byte = byte+2)/1000).toFixed(3)), src, "V"]);
			var frequency = +(buf.readUInt32BE(byte=byte+2));
			if(frequency > 1){
			arr.push(["frequency", 0, frequency, src, "ns/pulse"]);
			}
			else {} // decode nothing, because there is no CS506 Sensor attached assuming zero/0 ns/pulse
		}
		//Analog Packet
		else if(header == 0x20){
			src = "adc";
            // Data are sent as signed + calibrated microvolt values.
			var voltage1 = +(buf.readInt32BE(byte));        //ADC Single Ended Channel 1
			var voltage2 = +(buf.readInt32BE(byte=byte+4)); //ADC Single Ended Channel 2
			var voltage3 = +(buf.readInt32BE(byte=byte+4)); //ADC Single Ended Channel 3
			var voltage4 = +(buf.readInt32BE(byte=byte+4)); //ADC Single Ended Channel 4
			
			arr.push(["voltage-adc", 1, voltage1, src, "uV"]);
			arr.push(["voltage-adc", 2, voltage2, src, "uV"]);
			arr.push(["voltage-adc", 3, voltage2, src, "uV"]);
			arr.push(["voltage-adc", 4, voltage2, src, "uV"]);
		}
		//Digital Packet
		else if(header == 0x40){
			src = "digital";
			var digital4 = +(buf.readUInt32BE(byte));
			var digital3 = +(buf.readUInt32BE(byte=byte+4));
			var digital2 = +(buf.readUInt32BE(byte=byte+4));
			var digital1 = +(buf.readUInt32BE(byte=byte+4));
			
			//arr.push(["digital-count", 4, digital4, src]); //Uncomment line 2nd when 4th digital pulse counter input is needed.
			//arr.push(["digital-count", 3, digital3, src]); //Uncomment line 1st when 3rd digital pulse counter input is needed.
			arr.push(["digital-count", 2, digital2, src]);
			arr.push(["digital-count", 1, digital1, src]);
		}
		//SDI Packet
		else if(header & 0x80){
			src = "main";
			var addr = 0;
			var com = +(header & 0x0f);
			arr.push(["command", 0, com, src]);
			src = "sdi_" + com.toString();
			/*
			0-9 represents the 10 programming slots for SDI-12 onboard the device.
			Adjustment of SDI address in segement must match the physical real address of the sensor.
			*/
			//SDI 0
			if(com === 0){
				//Place Segment Code here.
			}
			//SDI 1
			else if(com == 1){
				//Place Segment Code here.
			}
			//SDI 2
			else if(com == 2){
				//Place Segment Code here.
			}
			//SDI 3
			else if(com == 3){
				//Place Segment Code here.
			}
			//SDI 4
			else if(com == 4){
				//Place Segment Code here.
			}
			//SDI 5
			else if(com == 5){
				//Place Segment Code here.
			}
			//SDI 6
			else if(com == 6){
				//Place Segment Code here.
			}
			//SDI 7
			else if(com == 7){
				//Place Segment Code here.
			}
			//SDI 8
			else if(com == 8){
				//Place Segment Code here.
			}
			//SDI 9
			else if(com == 9){
				//Place Segment Code here.
			}
		}
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


