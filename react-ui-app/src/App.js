import './App.css';
import { RosProvider } from './rosContext.js'
import { BrowserRouter as Router, Route, Routes, Link } from 'react-router-dom';
import Main from './Pages/Main.jsx';
import Sandbox from './Pages/Sandbox.jsx';
import RosStatus from './rosStatus.jsx';

function App() {
  return (
    <Router>

      <RosProvider>
        <nav>
          <ul>
            <li><Link to="/">Main</Link></li>
            <li><Link to="/sandbox">Sandbox</Link></li>
          </ul>
         <RosStatus></RosStatus> 
        </nav>

        <Routes>
          <Route path="/" element={<Main />} />
          <Route path="/sandbox" element={<Sandbox />} />
        </Routes>
      </RosProvider>
    </Router>
  );
}

export default App;
