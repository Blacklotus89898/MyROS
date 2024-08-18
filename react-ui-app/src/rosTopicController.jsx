import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';

const RosTopicController = () => {
  const [message, setMessage] = useState('');
  const [topic, setTopic] = useState('/chatter');
  const [type, setType] = useState('std_msgs/String');
  const [msg, setMsg] = useState('');
  const { ros, connection } = useRos();
  const [listener, setListener] = useState(null);

  useEffect(() => {
    if (ros) {
      handleListener();
    }

    // Cleanup on component unmount or ros changes
    return () => {
      if (listener) {
        listener.unsubscribe();
        console.log('Unsubscribed from previous listener');
      }
    };
  }, [ros, topic, type]); // Depend on ros, topic, and type

  const handleListener = () => {
    if (listener) {
      listener.unsubscribe();
      console.log('Unsubscribed from previous listener');
    }

    // Create a new listener
    const newListener = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: type,
    });

    // Set up the subscription
    newListener.subscribe((msg) => {
      console.log('Received message:', msg);
      setMessage(msg.data);
    });

    // Update listener state
    setListener(newListener);
    console.log('Subscribed to topic:', topic);
  };

  const handlePublish = () => {
    if (!ros) {
      console.error('ROS instance is not available');
      return;
    }

    if (!listener) {
      console.error('Listener is not initialized');
      return;
    }

    const messageToSend = new ROSLIB.Message({ data: msg });
    listener.publish(messageToSend);
    console.log('Published message:', msg);
  };

  return (
    <div>
      <h2>Topic Controller Connection: {connection}</h2>
      <h3>
        <input
          type="text"
          placeholder="Topic name"
          value={topic}
          onChange={(e) => setTopic(e.target.value)}
        />
      </h3>
      <h3>
        <input
          type="text"
          placeholder="Topic type: std_msgs/String"
          value={type}
          onChange={(e) => setType(e.target.value)}
        />
      </h3>
      <h3>
        <input
          type="text"
          placeholder="Topic message"
          value={msg}
          onChange={(e) => setMsg(e.target.value)}
        />
      </h3>
      <h3>
        <button onClick={handlePublish}>Publish message</button>
      </h3>
      <h3>
        <button onClick={handleListener}>Listen to topic</button>
      </h3>
      <p>Latest message: {message}</p>
    </div>
  );
};

export default RosTopicController;
