const WebSocket = require('ws');

const socket = new WebSocket('ws://localhost:8888');

// Handle open event
socket.on('open', function () {
    console.log('Socket connection established.');
    
});

// Handle message event
socket.on('message', function (message) {
    if (Buffer.isBuffer(message)) {
        const messageString = message.toString();
        const coordinates = messageString.split(',');
        const longitude = parseFloat(coordinates[0]);
        const latitude = parseFloat(coordinates[1]);

        // Use the longitude and latitude as needed

        console.log('Received coordinates:', longitude, latitude);
        
    }
    
});

// Handle close event
socket.on('close', function () {
    console.log('Socket connection closed.');
});
