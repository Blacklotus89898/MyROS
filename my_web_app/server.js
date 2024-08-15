const WebSocket = require('ws');
const { exec } = require('child_process');

const wss = new WebSocket.Server({ port: 3000 });

wss.on('connection', (ws) => {
    console.log('Client connected');

    ws.on('message', (message) => {
        let command;

        // Handle both string and buffer types
        if (typeof message === 'string') {
            command = message.trim();
        } else if (Buffer.isBuffer(message)) {
            command = message.toString().trim();
        } else {
            console.error('Unexpected message type:', typeof message);
            return;
        }

        console.log('Received command:', command);

        // Execute the command
        exec(command, (error, stdout, stderr) => {
            if (error) {
                console.error('Command error:', error.message);
                ws.send(`Error: ${error.message}`);
                return;
            }
            if (stderr) {
                console.error('Command stderr:', stderr);
                ws.send(`Stderr: ${stderr}`);
                return;
            }
            console.log('Command output:', stdout);
            ws.send(stdout); // Send the command output
        });
    });

    ws.on('close', () => {
        console.log('Client disconnected');
    });
});

console.log('WebSocket server started on ws://localhost:3000');

