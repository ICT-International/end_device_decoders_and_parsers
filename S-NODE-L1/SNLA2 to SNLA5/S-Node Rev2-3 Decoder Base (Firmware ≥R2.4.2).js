/* ICT S-NODE LoRa Decoder Base
	Suitable for SNLA2, SNLA3, SNLA4 and SNLA5 models running firmware 2.4.2 and above.
	SNLA = 900-930Mhz LoRaWan & 865-870Mhz LoRaWan
	Includes Base Diagnostic and SDI Section
*/

//Change Log:
/*
20241030 R1-0
Toby Partridge ICT International
* GNSS Updated to be represented as Double Precision Float instead of Boolean function. 1 = True, 0 = False

20241030 R2-0 
* Introduced var steps in GNSS posistioning to allow skipping of array for products such as 
SNLA2 and SNLA4 that do not contain GNSS hardware.

20241105: R2-1
* Introduces automatic parsing of gps posistioning or omission 
to reduce wasted data for payloads that contain false GNSS positioning coordinates.

20260114: R2-3
* Added all lines for sdi12 slots (0-9) in decoded segment.

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
		if(e_length>5) par['address']=e[5];		//Comment this segment out for Senaps Integration until advised otherwise
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

	// Data Packet Received
	if(p === 1){
		var src = "diagnostic";
		arr.push(['packet-type', 0, "DATA_PACKET", src]);
		arr.push(["uptime", 0, +(buf.readUInt32BE(byte)), src, "s"]);
		arr.push(["battery-voltage", 0, +((buf.readUInt16BE(byte = byte+4)/1000).toFixed(3)), src, "V"]);
		arr.push(["solar-voltage", 0, +((buf.readUInt16BE(byte = byte+2)/1000).toFixed(3)), src, "V"]);
		var charge_fault = buf.readUInt8(byte=byte+2); //9th Byte with Charge, Fault and GNSS True/False all in one.
		arr.push(["charging-state", 0, +(charge_fault & 1), src]);
		arr.push(["fault", 0, +((charge_fault & 2) >> 1), src]);
		var gnss = +((charge_fault & 4) >> 2) > 0 ? 1 : 0;		//1 = True, 0 = False
		arr.push(["gnss", 0, gnss, src]);
		
		if(gnss){ //If GNSS payload type is True, then it contains a payload that may have GNSS data:
			var latitude_raw = +((buf.readInt32BE(byte=byte+1)/10000000).toFixed(6)); 
			var longitude_raw = +((buf.readInt32BE(byte=byte+4)/10000000).toFixed(6));
			
				if((latitude_raw != 0) && (longitude_raw != 0)){ //If GNSS lat AND long are not equal to zero, therefore real positioning data thus push real readings to push array to parser.
				arr.push(["latitude", 0, latitude_raw, src, "Degrees"]);
				arr.push(["longitude", 0, longitude_raw, src, "Degrees"]);
				return arr;
				}
				else{
					return arr;
				} //else if Lat AND Long = 0, no lat or long are parsed to message, therefore complete parising of only power diagnostics
		}
		
		if(buf.length < 10){
			return arr;
		}

		src = "main";
		var addr = 0;
		var com = +(buf.readUInt8(byte = byte+1));
		arr.push(["command", 0, com, src]);
		src = "sdi_" + com.toString();
		byte = 10;

		// SDI 0
		if(com == 0){
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
	}else if(p === 10){
		//Device Info Packet Recieved
		var src = "device_info";
		arr.push(["packet-type", 0, "DEVICE_INFO", "main"]);
		arr.push(["product-id", 0, buf.readUInt32BE(byte), src]);
		arr.push(["batch-number", 0, buf.readUInt32BE(byte=byte+4), src]);
		arr.push(["software-version", 0, buf.readUInt32BE(byte=byte+4), src]);
	}else if(p === 100){
		//Downlink Response Packet Recieved
		arr.push(["packet-type", 0, "DOWNLINK_RESPONSE", "main"]);
		arr.push(["downlink-response", 0, String.fromCharCode.apply(String, buf.slice(0, buf.length)), "downlink"]);
	}else{
		// Unknown Response Recieved
		arr.push(["packet-type", 0, "UNKNOWN_RESPONSE", "main"]);
		arr.push(["raw-payload", 0, buf.slice(0, buf.length), "unknown"]);
	}

	return arr;

}
