@REM Skript to easy regenerate the CAN code
@REM Just change the node name BrakeF to the name of your ECU and adapt paths if necessary

canzero ecu --db ..\canzero\pod2023 --node SensorF --path . --filename dbc_parser

@echo off
echo.
pause 1
