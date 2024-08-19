import React, { useEffect, useState } from 'react';
import ROSLIB from 'roslib';
import { useRos } from './rosContext';
import RosTopicForm from './rosTopicForm.jsx'; // Import the RosTopicForm component

const RosTopicController = () => {
  const [message, setMessage] = useState('');
  const [msg, setMsg] = useState('');
  const { ros } = useRos();
  const [listener, setListener] = useState(null);
  const [topic, setTopic] = useState('/chatter');
  const [type, setType] = useState('std_msgs/String');

  useEffect(() => {
    if (ros) {
      handleListener();
    }

    return () => {
      if (listener) {
        listener.unsubscribe();
        console.log('Unsubscribed from previous listener');
      }
    };
  }, [ros, topic, type]);

  const handleListener = () => {
    if (listener) {
      listener.unsubscribe();
      console.log('Unsubscribed from previous listener');
    }

    const newListener = new ROSLIB.Topic({
      ros: ros,
      name: topic,
      messageType: type,
    });

    newListener.subscribe((msg) => {
      console.log('Received message:', msg);
      setMessage(msg.data);
    });

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

  // Callback function to handle the form submission
  const handleFormSubmit = ({ topicName, messageType }) => {
    setTopic(topicName);
    setType(messageType);
  };

  return (
    <div style={{ border: '1px solid black', padding: '10px', flex: '1 1 200px' }}>
      <h2>Topic Controller</h2>

      {/* Use RosTopicForm component */}
      <RosTopicForm onMessageReceived={handleFormSubmit} />

      <hr />

      <h3>Listen to Topic</h3>
      <p>Latest message: {message}</p>

      <hr />

      <h3>Write to the Topic</h3>
      <input
        type="text"
        placeholder="Topic message"
        value={msg}
        onChange={(e) => setMsg(e.target.value)}
      />
      <button onClick={handlePublish}>Publish message</button>
    </div>
  );
};

export default RosTopicController;
