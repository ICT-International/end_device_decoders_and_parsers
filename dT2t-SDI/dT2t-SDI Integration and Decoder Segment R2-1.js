//dT2t-SDI Integration and Decoder Segment

//Wiring:
/* 
Wiring
Brown: Pwr+
Black: Data/SDI
Blue : GND
*/

//Change log:
/*
2024Oct24 (ICT International Toby Partridge):
* Creation of Integration and Decoder Segment, testing of commands.

2026Jun23 (ICT International Toby Partridge):
R2-0
* Changed command used for ICT nodes from Measurement mode to Concurrent mode (aM! to aC!).
* Heading Change to dT2t-SDI Integration and Decoder Segment.
* File name change and revision number added.
* Changed parsed labels to include -upper and -lower in the name to reflect the phy location of the sensing elements.
* Added an application note specifying that air-temperature-upper-dt requires post-processing when an actual temperature value is required.
* Fixed spelling.
*/


''''
			/* Suggested Commands: 
			Measurement Mode: aM! followed by aD0!, 2 variables returned after 1 second, dT and Lower Temperature in °C
			Concurrent  Mode: aC! followed by aD0!, 2 variables returned after 1 second, dT and Lower Temperature in °C
			// For ICT S-NODE and MFR-NODE: $ sdi12 add <a> M <a>C! <a>D0! 11 dt2t-<a>
			*/
			addr = 0; //address register - change to match actual sensor address for correct MFR and S-NODE LoRa decoding.
			arr.push(["air-temperature-upper-dt", 0, +(buf.readFloatBE(byte).toFixed(3)), src, "C", addr]);
			arr.push(["air-temperature-lower", 0, +(buf.readFloatBE(byte=byte+4).toFixed(3)), src, "C", addr]);
			/* 
			Note: air-temperature-upper-dt is the difference compared to air-temperature-lower, the user may require~
			post-processing to achieve an actual temperature.
			*/
''''
