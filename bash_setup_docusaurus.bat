@echo off
echo Setting up Docusaurus documentation site...

REM Change to the project directory
cd /d "C:\Users\NOOR\Desktop\Physical AI & Humanoid Robotics"

REM Create necessary directories
if not exist "src\css" mkdir src\css
if not exist "static\img" mkdir static\img

REM Create a basic static image folder structure
echo. > static\img\.gitkeep

REM Create the complete docs structure
if not exist "docs\modules\ros2-fundamentals" mkdir docs\modules\ros2-fundamentals
if not exist "docs\modules\simulation-environments" mkdir docs\modules\simulation-environments
if not exist "docs\modules\ai-perception" mkdir docs\modules\ai-perception
if not exist "docs\modules\vla-integration" mkdir docs\modules\vla-integration

REM Copy the existing content to the proper locations
copy "docs\modules\ros2-fundamentals\index.md" "docs\modules\ros2-fundamentals\index.md" >nul 2>&1
copy "docs\modules\simulation-environments\index.md" "docs\modules\simulation-environments\index.md" >nul 2>&1
copy "docs\modules\ai-perception\index.md" "docs\modules\ai-perception\index.md" >nul 2>&1
copy "docs\modules\vla-integration\index.md" "docs\modules\vla-integration\index.md" >nul 2>&1

REM Create basic Docusaurus files if they don't exist
if not exist "docusaurus.config.js" (
    echo Creating basic docusaurus.config.js...
    REM We already have this file, so we'll skip
)

if not exist "package.json" (
    echo Creating package.json...
    echo { > package.json
    echo   "name": "physical-ai-humanoid-robotics", >> package.json
    echo   "version": "1.0.0", >> package.json
    echo   "private": true, >> package.json
    echo   "scripts": { >> package.json
    echo     "docusaurus": "docusaurus", >> package.json
    echo     "start": "docusaurus start", >> package.json
    echo     "build": "docusaurus build", >> package.json
    echo     "swizzle": "docusaurus swizzle", >> package.json
    echo     "deploy": "docusaurus deploy", >> package.json
    echo     "clear": "docusaurus clear", >> package.json
    echo     "serve": "docusaurus serve", >> package.json
    echo     "write-translations": "docusaurus write-translations", >> package.json
    echo     "write-heading-ids": "docusaurus write-heading-ids" >> package.json
    echo   }, >> package.json
    echo   "dependencies": { >> package.json
    echo     "@docusaurus/core": "^3.1.0", >> package.json
    echo     "@docusaurus/preset-classic": "^3.1.0", >> package.json
    echo     "@mdx-js/react": "^3.0.0", >> package.json
    echo     "clsx": "^2.0.0", >> package.json
    echo     "prism-react-renderer": "^2.3.0", >> package.json
    echo     "react": "^18.0.0", >> package.json
    echo     "react-dom": "^18.0.0" >> package.json
    echo   }, >> package.json
    echo   "devDependencies": { >> package.json
    echo     "@docusaurus/module-type-aliases": "^3.1.0", >> package.json
    echo     "@docusaurus/types": "^3.1.0" >> package.json
    echo   }, >> package.json
    echo   "browserslist": { >> package.json
    echo     "production": [ >> package.json
    echo       ">0.5%%", >> package.json
    echo       "not dead", >> package.json
    echo       "not op_mini all" >> package.json
    echo     ], >> package.json
    echo     "development": [ >> package.json
    echo       "last 3 chrome version", >> package.json
    echo       "last 3 firefox version", >> package.json
    echo       "last 5 safari version" >> package.json
    echo     ] >> package.json
    echo   }, >> package.json
    echo   "engines": { >> package.json
    echo     "node": ">=18.0" >> package.json
    echo   } >> package.json
    echo } >> package.json
)

echo Docusaurus setup files created successfully!
echo To run the documentation site:
echo   1. Make sure Node.js and npm are installed
echo   2. Run: npm install
echo   3. Run: npm start
pause