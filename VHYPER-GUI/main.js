'use strict'

const {app} = require('electron');
const {BrowserWindow} = require('electron');
const {globalShortcut} = require('electron');
const {dialog} = require('electron');

var mainWindow = null;

app.on('ready', function() {
	mainWindow = new BrowserWindow({
		frame: false,
		fullscreen: true,
	});	
	mainWindow.loadURL('file://' + __dirname + '/app/index.html');
	
	globalShortcut.register('esc', function() {
		var a = dialog.showMessageBox(mainWindow, {type: 'warning', title: 'VHYPER Controls', message: 'Are you sure you want to quit?', buttons: ['Yes', 'No']});
		if (a == 0) {
			mainWindow.close();
		}
	});
});