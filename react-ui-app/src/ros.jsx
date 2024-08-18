import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import {useRos} from './rosContext';
// To be remade
const RosComponent = () => {
  const [message, setMessage] = useState('');
  const {ros }= useRos();

  //At the start
  useEffect(() => {
    if (ros != null) {

    // Subscribe to a ROS topic
    const listener = new ROSLIB.Topic({
      ros: ros,
      name: '/chatter', // Replace with your ROS topic name
      messageType: 'std_msgs/String', // Replace with your ROS message type
    });

    listener.subscribe((msg) => {
      console.log('Received message:', msg);
      setMessage(msg.data);
    });
    }
  }, [ros]);

  return (
    <div>
      <h1>ROS Web Interface</h1>
      <h3>The ros instance: {JSON.stringify(ros)}</h3>
      <p>Latest message: {message}</p>
    </div>
  );
};

export default RosComponent;

