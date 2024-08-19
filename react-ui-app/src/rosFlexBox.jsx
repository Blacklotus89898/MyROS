import React from 'react';

function RosFlexBox({ children, style }) {
  return (
    <div
      style={{
        flex: '1 1 200px', 
        minWidth: '400px',
        padding: '10px',
        boxSizing: 'border-box',
        backgroundColor: '#f0f0f0',
        borderRadius: '8px',
        border:'1px solid black',
        // boxShadow: '0 0 15px rgba(0, 255, 0, 0.6)', // Green glow effect
        ...style, 
      }}
    >
      {children}
    </div>
  );
}

export default RosFlexBox;
