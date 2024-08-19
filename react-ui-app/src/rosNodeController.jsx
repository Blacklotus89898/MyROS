import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext'; 

// Assuming my_controller/terminal_node.py is running
const RosNodeController = () => {
  const [command, setCommand] = useState('');
  const [terminalOutput, setTerminalOutput] = useState('');
  const { ros } = useRos();
  const [publisher, setPublisher] = useState(null);
  const [subscriber, setSubscriber] = useState(null);

  useEffect(() => {
    if (ros) {
      // Initialize the publisher for sending commands
      const commandPublisher = new ROSLIB.Topic({
        ros: ros,
        name: '/command_input',
        messageType: 'std_msgs/String',
      });

      // Initialize the subscriber for receiving terminal output
      const outputSubscriber = new ROSLIB.Topic({
        ros: ros,
        name: '/terminal_output',
        messageType: 'std_msgs/String',
      });

      // Handle incoming messages on the terminal output topic
      outputSubscriber.subscribe((message) => {
        console.log('Received terminal output:', message.data);
        setTerminalOutput(message.data);
      });

      setPublisher(commandPublisher);
      setSubscriber(outputSubscriber);

      // Cleanup function to unsubscribe and reset publishers/subscribers
      return () => {
        if (outputSubscriber) {
          outputSubscriber.unsubscribe();
          console.log('Unsubscribed from /terminal_output');
        }
        setPublisher(null);
        setSubscriber(null);
      };
    }
  }, [ros]);

  const handlePublishCommand = () => {
    if (publisher) {
      const message = new ROSLIB.Message({ data: command });
      publisher.publish(message);
      console.log('Published command:', command);
      setCommand(''); // Clear the command input after publishing
    }
  };

  return (
    <div style={{ border: '1px solid black', padding: '10px', flex: '1 1 200px' }}>
      <h2>ROS Node Controller</h2>
      
      <div style={{ marginBottom: '10px' }}>
        <input
          type="text"
          placeholder="Enter command"
          value={command}
          onChange={(e) => setCommand(e.target.value)}
          style={{ width: '70%', marginRight: '10px' }}
        />
        <button onClick={handlePublishCommand}>Send Command</button>
      </div>

      <h3>Terminal Output</h3>
      <pre style={{ border: '1px solid gray', padding: '10px', whiteSpace: 'pre-wrap' }}>
        {terminalOutput || 'No output received yet.'}
      </pre>
    </div>
  );
};

export default RosNodeController;
