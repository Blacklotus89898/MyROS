import React, { useState } from 'react';
import useRosTopic from './useRosTopic.js';
import RosTopicForm from './rosTopicForm.jsx';

// reusable form implementation
const RosTopicSubscriber = () => {
  const [topicName, setTopicName] = useState('');
  const [messageType, setMessageType] = useState('');

  const message = useRosTopic(topicName, messageType);

  const handleFormSubmit = ({ topicName, messageType }) => {
    setTopicName(topicName);
    setMessageType(messageType);
  };

  return (
    <div style={{ border: '1px solid black', padding: '10px', flex:'1 1 200px' }}>
      <h2>ROS Topic Subscriber</h2>
      <RosTopicForm onMessageReceived={handleFormSubmit} />
      <div>
        <h3>Received Message:</h3>
        
        <pre>{message ? JSON.stringify(message) : 'No message data received'}</pre>
      </div>
    </div>
  );
};

export default RosTopicSubscriber;
