import React, { useState } from 'react';

const RosTopicForm = ({ onMessageReceived }) => {
  const [topicName, setTopicName] = useState('');
  const [messageType, setMessageType] = useState('');

  const handleSubmit = (e) => {
    e.preventDefault();
    // Trigger the callback function to notify the parent component
    if (onMessageReceived) {
      onMessageReceived({ topicName, messageType });
    }
  };

  return (
    <div>
      <form onSubmit={handleSubmit}>
        <div>
          <label>
            Topic Name:
            <input
              type="text"
              value={topicName}
              onChange={(e) => setTopicName(e.target.value)}
              placeholder="/ui_topic"
            />
          </label>
        </div>
        <div>
          <label>
            Message Type:
            <input
              type="text"
              value={messageType}
              onChange={(e) => setMessageType(e.target.value)}
              placeholder="std_msgs/String"
            />
          </label>
        </div>
        <button type="submit">Subscribe</button>
      </form>
    </div>
  );
};

export default RosTopicForm;
