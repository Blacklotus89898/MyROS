// src/RosComponent.js
import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';

const RosComponent = () => {
  const [ros, setRos] = useState(null);
  const [message, setMessage] = useState('');

  const [me, setMe] = useState('');

  //At the start
  useEffect(() => {
    // Initialize the ROS connection
    const rosInstance = new ROSLIB.Ros({
      url: 'ws://localhost:9090', // Replace with your ROS master WebSocket URL
    });

    rosInstance.on('connection', () => {
      console.log('Connected to ROS!');
    });

    rosInstance.on('error', (error) => {
      console.error('Error connecting to ROS:', error);
    });

    rosInstance.on('close', () => {
      console.log('Connection to ROS closed.');
    });

    setRos(rosInstance);
    console.log(rosInstance);
    console.dir(rosInstance);
    setMe(JSON.stringify(rosInstance));

    // Subscribe to a ROS topic
    const listener = new ROSLIB.Topic({
      ros: rosInstance,
      name: '/chatter', // Replace with your ROS topic name
      messageType: 'std_msgs/String', // Replace with your ROS message type
    });

    listener.subscribe((msg) => {
      console.log('Received message:', msg);
      setMessage(msg.data);
    });

    // Cleanup on component unmount
    return () => {
      listener.unsubscribe();
      rosInstance.close();
    };
  }, []);

  return (
    <div>
      <h1>ROS Web Interface</h1>
      <h3>The ros instance: {JSON.stringify(ros)}</h3>
      <p>Latest message: {message}</p>
    </div>
  );
};

export default RosComponent;

