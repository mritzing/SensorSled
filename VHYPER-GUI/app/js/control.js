// JavaScript Document
'use strict'

var net = require('net');

const fileName = 'vac_test.txt';

var connected = false;

var acc_x_guage = new JustGage({
	id: "acc_x",
	value: 0,
	min: -2.5,
	max: 2.5,
	title: 'Acceleration',
	symbol: ' m/s\u00B2',	
	pointer: true,
	gaugeColor: '#37C8AB',	
	levelColors: ['#37C8AB'],
	decimals: 2,
});

var vel_x_guage = new JustGage({
	id: "vel_x",
	value: 0,
	min: 0,
	max: 100,
	title: 'Velocity',
	symbol: ' m/s',	
    pointer: true,
	decimals: 2,
});

var pos_x_guage = new JustGage({
	id: "pos_x",
	value: 0,
	min: 0,
	max: 100,
	title: 'Position',
	symbol: ' m',
	pointer: true,
	decimals: 2,
});

var strip_count_guage = new JustGage({
	id: "strip_count",
	value: 0,
	min: 0,
	max: 100,
	title: 'Optical Marking Count',
	pointer: true,
});

var pitch_guage = new JustGage({
	id: "pitch",
	value: 0,
	min: -5,
	max: 5,
	title: 'Pitch',
	symbol: ' \u00B0',
	pointer: true,
	gaugeColor: '#37C8AB',	
	levelColors: ['#37C8AB'],
	decimals: 2,
});

var roll_guage = new JustGage({
	id: "roll",
	value: 0,
	min: -5,
	max: 5,
	title: 'Roll',
	symbol: ' \u00B0',
	pointer: true,	
	gaugeColor: '#37C8AB',	
	levelColors: ['#37C8AB'],
	decimals: 2,
});

var bounds = {};
bounds['acc_x'] = [-2, 2, 0.2];
bounds['acc_y'] = [-2, 2, 0.2];
bounds['acc_z'] = [-2, 2, 0.2];
bounds['vel_x'] = [-10, 10, 1];
bounds['vel_y'] = [-10, 10, 1];
bounds['vel_z'] = [-10, 10, 1];
bounds['pos_x'] = [-10, 10, 1];
bounds['pos_y'] = [-10, 10, 1];
bounds['pos_z'] = [-10, 10, 1];
bounds['pitch'] = [-5, 5, 1];
bounds['roll'] = [-5, 5, 1];
bounds['front_right'] = [150, 210, 5];
bounds['front_center'] = [120, 180, 5];
bounds['front_left'] = [150, 210, 5];
bounds['back_right'] = [150, 210, 5];
bounds['back_center'] = [120, 180, 5];
bounds['back_left'] = [150, 210, 5];
bounds['pressure_high'] = [500, 3000, 200];
bounds['pressure_low'] = [0, 125, 5];
bounds['raspberry_pi'] = [0, 70, 5];
bounds['dc_dc_converter'] = [0, 70, 5];
bounds['cerebot'] = [0, 85, 5];
bounds['current'] = [0, 4, 0.2];
bounds['battery_1'] = [0, 60, 5];
bounds['battery_2'] = [0, 60, 5];
bounds['battery_3'] = [0, 60, 5];
bounds['battery_4'] = [0, 60, 5];

var podStates = ['Initialize', 'Startup', 'Idle', 'Confirm Start', 'Ready To Push', 'Acceleration', 'Braking', 'Post Test Idle', 'Error']

var updateDisplay = function(gagePackage, tablePackage, timePackage, statePackage, podState) {
	document.getElementById('pod_state').innerHTML = podStates[podState];
	
	acc_x_guage.refresh(gagePackage['acc_x']);
	vel_x_guage.refresh(gagePackage['vel_x']);
	pos_x_guage.refresh(gagePackage['pos_x']);
	strip_count_guage.refresh(gagePackage['strip_count']);
	pitch_guage.refresh(gagePackage['pitch']);
	roll_guage.refresh(gagePackage['roll']);
	
	for (var i in statePackage) {
		document.getElementById(i).innerHTML = statePackage[i];
	}
	
	for (var i in timePackage) {
		document.getElementById(i).innerHTML = timePackage[i];
	}
	
	for (var i in tablePackage) {
		var min = bounds[i][0];
		var max = bounds[i][1];
		var diff = bounds[i][2];
		var value = tablePackage[i];
		
		
		document.getElementById(i).innerHTML = value;
		if (value > max || value < min) {
			document.getElementById(i).parentNode.style.background = '#E64C66';
		} else if (value > (max - diff) || value < (min + diff)) {
			document.getElementById(i).parentNode.style.background = "#F2D336";
		} else {
			document.getElementById(i).parentNode.style.background = '#37C8AB';
		}
	}
};
 

var client = new net.Socket();
client.connect(10000, '192.168.0.6', function() {
	console.log('Connected');
	document.getElementById('alert_message').innerHTML = 'Connected: True';
	connected = true;
});

client.on('error', function() {
	connected = false;
});

client.on('data', function(data) {	
	client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x60]));
	
	if (data[2] === 0x05) {
		console.log('received');
		
		
		var statePackage = {};
		var ahrsPackage = {};
		var timePackage = {};
		var gagePackage = {};
		var tablePackage = {};
		var u8 = new Uint8Array(data);
		var u16 = new Uint16Array((u8.length - 10) / 2);
		for (var i = 0; i < u16.length; i++) {
			u16[i] = (u8[i * 2 + 10] << 0) + (u8[i * 2 + 11] << 8);
		}
		var i16 = new Int16Array(u16);
		var u32 = new Uint32Array(3);
		u32[0] = (u16[u16.length - 6] << 0) + (u16[u16.length - 5] << 16);
		u32[1] = (u16[u16.length - 4] << 0) + (u16[u16.length - 3] << 16);
		u32[2] = (u16[u16.length - 2] << 0) + (u16[u16.length - 1] << 16);
		
		var podState = u8[6];
		statePackage['pusher_status'] = u8[7];
		statePackage['brake_status'] = u8[8];
		statePackage['low_speed_status'] = u8[9];
		
		gagePackage['acc_x'] = (i16[0] / 100).toFixed(2);
		gagePackage['vel_x'] = (i16[3] / 100).toFixed(2);
		gagePackage['pos_x'] = (i16[6] / 100).toFixed(2);
		gagePackage['pitch'] = (i16[9] / 100).toFixed(2);
		gagePackage['roll'] = (i16[10] / 100).toFixed(2);
		gagePackage['strip_count'] = u16[11];
		
		ahrsPackage['acc_y'] = (i16[1] / 1000).toFixed(3);
		ahrsPackage['acc_z'] = (i16[2] / 1000).toFixed(3);		
		ahrsPackage['vel_y'] = (i16[4] / 1000).toFixed(3);
		ahrsPackage['vel_z'] = (i16[5] / 1000).toFixed(3);		
		ahrsPackage['pos_y'] = (i16[7] / 1000).toFixed(3);
		ahrsPackage['pos_z'] = (i16[8] / 1000).toFixed(3);
		
		tablePackage['front_right'] = (u16[14] / 100).toFixed(2);
		tablePackage['front_center'] = (u16[15] / 100).toFixed(2);
		tablePackage['front_left'] = (u16[16] / 100).toFixed(2);
		tablePackage['back_right'] = (u16[17] / 100).toFixed(2);
		tablePackage['back_center'] = (u16[18] / 100).toFixed(2);
		tablePackage['back_left'] = (u16[19] / 100).toFixed(2);
		tablePackage['pressure_high'] = u16[20];
		tablePackage['pressure_low'] = u16[21];
		tablePackage['raspberry_pi'] = (u16[25] / 10).toFixed(1);
		tablePackage['dc_dc_converter'] = (u16[26] / 10).toFixed(1);
		tablePackage['cerebot'] = (u16[27] / 10).toFixed(1);
		tablePackage['current'] = (u16[36] / 100).toFixed(3);
		tablePackage['battery_1'] = (u16[39] / 10).toFixed(1);
		tablePackage['battery_2'] = (u16[39] / 10).toFixed(1);
		tablePackage['battery_3'] = (u16[42] / 10).toFixed(1);
		tablePackage['battery_4'] = (u16[43] / 10).toFixed(1);
		
		timePackage['pod_run_time'] = (u32[0] / 1000).toFixed(1);
		timePackage['test_run_time'] = (u32[1] / 1000).toFixed(1);
		timePackage['last_strip_time'] = (u32[2] / 1000).toFixed(1);
		
		updateDisplay(gagePackage, tablePackage, timePackage, statePackage, podState);
	}
});

client.on('close', function() {
	console.log('Connection closed');
	document.getElementById('connected').innerHTML = 'Connected: False';
	connected = false;
});

document.getElementById('brakes_on_button').addEventListener('click', function() {
	client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x11]));
}, false);

document.getElementById('brakes_off_button').addEventListener('click', function() {
	client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x10]));
}, false);

document.getElementById('low_speed_on_button').addEventListener('click', function() {
	client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x21]));
}, false);

document.getElementById('low_speed_off_button').addEventListener('click', function() {
	client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x20]));
}, false);

document.getElementById('start_test').addEventListener('click', function() {
	if (window.confirm('Start the test?')) {			
		client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x30]));
	}
}, false);

document.getElementById('stop_test').addEventListener('click', function() {
	client.write(new Buffer([0xAA, 0x55, 0x00, 0x01, 0x01, 0x00, 0x31]));
}, false);
