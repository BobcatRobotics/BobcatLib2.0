@echo off
echo Formatting
call gradlew.bat :spotlessApply
echo Publish
call gradlew.bat publish
echo Copy
Xcopy /E /y .\\build\\repos\\releases\\YourLibTemplate .\\yagsl\\repos\\YourLibTemplate