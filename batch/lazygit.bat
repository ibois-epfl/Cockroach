@ECHO OFF
set /p id="Enter commit:"
git add .
git commit -m "%id%"
git push