@echo off
echo Starting Physical AI & Humanoid Robotics Curriculum Documentation...

REM Change to the project directory
cd /d "C:\Users\NOOR\Desktop\Physical AI & Humanoid Robotics"

REM Check if Node.js is installed
node --version >nul 2>&1
if errorlevel 1 (
    echo Node.js is not installed or not found in PATH.
    echo Please install Node.js (version 18 or higher) from https://nodejs.org/
    pause
    exit /b 1
)

REM Check if npm is available
npm --version >nul 2>&1
if errorlevel 1 (
    echo npm is not available. This is unexpected if Node.js is properly installed.
    echo Please verify your Node.js installation.
    pause
    exit /b 1
)

REM Check if node_modules directory exists
if not exist "node_modules" (
    echo node_modules directory not found.
    echo Please run "install-node-modules.bat" first to install dependencies.
    pause
    exit /b 1
)

echo.
echo Starting Docusaurus documentation server...
echo This will open the curriculum documentation in your browser at http://localhost:3000
echo Press Ctrl+C to stop the server when you're done.

echo.
echo Starting server...
npm start

if errorlevel 1 (
    echo Failed to start the documentation server. Check the error messages above.
    pause
    exit /b 1
)