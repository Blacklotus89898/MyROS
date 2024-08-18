// src/Terminal.js
// or just publish and subscribe ot the terminal node controller
import React, { useState, useEffect, useRef } from 'react';

const Terminal = () => {
  const [input, setInput] = useState('');
  const [output, setOutput] = useState('');
  const inputRef = useRef(null);
  const wsRef = useRef(null);

  useEffect(() => {
    // Initialize WebSocket connection
    wsRef.current = new WebSocket('ws://localhost:3001');

    wsRef.current.onopen = () => {
      console.log('WebSocket connection established');
    };

    wsRef.current.onmessage = (event) => {
      setOutput(prevOutput => prevOutput + event.data + '\n');
    };

    wsRef.current.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    wsRef.current.onclose = () => {
      console.log('WebSocket connection closed');
    };

    return () => {
      if (wsRef.current) {
        wsRef.current.close();
      }
    };
  }, []);

  const handleInputChange = (e) => {
    setInput(e.target.value);
  };

  const handleKeyDown = (e) => {
    if (e.key === 'Enter') {
      e.preventDefault();
      if (wsRef.current) {
        wsRef.current.send(input);
        setOutput(prevOutput => prevOutput + `> ${input}\n`);
        setInput('');
      }
    }
  };

  return (
    <div style={{ display: 'flex', flexDirection: 'column', height: '100vh', fontFamily: 'Arial, sans-serif' }}>
      <div style={{ flex: 1, padding: '10px', backgroundColor: '#000', color: '#0f0', overflowY: 'auto', whiteSpace: 'pre-wrap' }}>
        {output}
      </div>
      <input
        type="text"
        value={input}
        onChange={handleInputChange}
        onKeyDown={handleKeyDown}
        style={{ padding: '10px', border: 'none', backgroundColor: '#333', color: '#fff', fontFamily: 'monospace', fontSize: '16px', outline: 'none' }}
        autoFocus
      />
    </div>
  );
};

export default Terminal;
