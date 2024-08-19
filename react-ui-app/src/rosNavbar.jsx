import { Link } from 'react-router-dom';
import RosStatus from './rosStatus.jsx';

function RosNavbar() {
    return (
        <nav style={{ border: '4px solid black' }}>
            <ul style={{
                display: 'flex',
                justifyContent: 'space-evenly'
            }}>
                <Link to="/">Main</Link>
                <Link to="/sandbox">Sandbox</Link>
            </ul>
            <RosStatus></RosStatus>
        </nav>
    );
}

export default RosNavbar;
