import React, { useState } from 'react';

// parent callback function passed as onMessageReceived
const RosTopicForm = ({ onMessageReceived }) => {
  const [topicName, setTopicName] = useState('ui_topic');
  const [messageType, setMessageType] = useState('std_msgs/String');

  const handleSubmit = (e) => {
    e.preventDefault();
    if (onMessageReceived) {
      onMessageReceived({ topicName, messageType });
    }
  };

  return (
    <div style={{ border: '1px solid black', padding: '10px', flex:'1 1 200px'}}>
      <form onSubmit={handleSubmit} style={{display:'flex', justifyContent:'space-around'}}>
        <div>
          <label>
            Topic Name: 
            </label>
            <input
              type="text"
              value={topicName}
              onChange={(e) => setTopicName(e.target.value)}
              placeholder="/ui_topic"
            />
            <hr />
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
