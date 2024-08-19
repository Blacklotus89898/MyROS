import './App.css';
import { RosProvider } from './rosContext.js'
import { BrowserRouter as Router, Route, Routes } from 'react-router-dom';
import Main from './Pages/Main.jsx';
import Sandbox from './Pages/Sandbox.jsx';
import RosNavbar from './rosNavbar.jsx';

function App() {
  return (
    <Router>
      <RosProvider>
        <RosNavbar></RosNavbar>
        <Routes>
          <Route path="/" element={<Main />} />
          <Route path="/sandbox" element={<Sandbox />} />
        </Routes>
      </RosProvider>
    </Router>
  );
}

export default App;
