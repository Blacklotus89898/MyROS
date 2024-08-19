import React, { useState, useRef } from 'react';

function RosResizableBox({ children, style }) {
    const [dimensions, setDimensions] = useState({ width: 700, height: 600 });
    const boxRef = useRef(null);
    const isResizing = useRef(false);
    const startX = useRef(0);
    const startY = useRef(0);
    const startWidth = useRef(0);
    const startHeight = useRef(0);

    const handleMouseDown = (e) => {
        isResizing.current = true;
        startX.current = e.clientX;
        startY.current = e.clientY;
        startWidth.current = boxRef.current.offsetWidth;
        startHeight.current = boxRef.current.offsetHeight;
        document.addEventListener('mousemove', handleMouseMove);
        document.addEventListener('mouseup', handleMouseUp);
    };

    const handleMouseMove = (e) => {
        if (isResizing.current) {
            const newWidth = Math.max(100, startWidth.current + (e.clientX - startX.current));
            const newHeight = Math.max(100, startHeight.current + (e.clientY - startY.current));
            setDimensions({ width: newWidth, height: newHeight });
        }
    };

    const handleMouseUp = () => {
        isResizing.current = false;
        document.removeEventListener('mousemove', handleMouseMove);
        document.removeEventListener('mouseup', handleMouseUp);
    };

    return (
        <div
            ref={boxRef}
            style={{
                width: `${dimensions.width}px`,
                height: `${dimensions.height}px`,
                minWidth: '200px',
                minHeight: '150px',
                padding: '10px',
                boxSizing: 'border-box',
                backgroundColor: '#f0f0f0',
                borderRadius: '8px',
                position: 'relative',
                ...style,
            }}
        >
            {children}
            <div
                onMouseDown={handleMouseDown}
                style={{
                    width: '10px',
                    height: '10px',
                    backgroundColor: '#000',
                    position: 'absolute',
                    bottom: '0',
                    right: '0',
                    cursor: 'nwse-resize',
                }}
            />
        </div>
    );
}

export default RosResizableBox;
