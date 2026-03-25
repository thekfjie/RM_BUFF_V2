@echo off
setlocal
call "%~dp0run_yolo_video_test.bat" ^
  --parameter "E:\RM\rm_vision\examples\example_for_prediction\7_dark_red_big\parameter.yaml" ^
  --output "E:\RM\rm_vision\RM_BUFF_V2.1\tests\output\big_red_full.avi" ^
  --show
exit /b %ERRORLEVEL%
