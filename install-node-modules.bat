@echo off
echo Installing Node.js modules for Physical AI & Humanoid Robotics Curriculum Documentation...

REM Change to the project directory
cd /d "C:\Users\NOOR\Desktop\Physical AI & Humanoid Robotics"

REM Check if Node.js is installed
node --version >nul 2>&1
if errorlevel 1 (
    echo Node.js is not installed or not found in PATH.
    echo Please install Node.js (version 18 or higher) from https://nodejs.org/
    echo.
    echo After installing Node.js, run this script again.
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

echo Node.js version:
node --version

echo npm version:
npm --version

echo.
echo Installing dependencies from package.json...

REM Install dependencies
npm install

if errorlevel 1 (
    echo Failed to install dependencies. Check the error messages above.
    pause
    exit /b 1
)

echo.
echo Dependencies installed successfully!

echo.
echo To start the documentation site, run:
echo   npm start
echo.
echo This will launch the Physical AI & Humanoid Robotics curriculum documentation
echo at http://localhost:3000 in your default browser.

pause