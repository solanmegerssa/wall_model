@echo off
set MATLAB=C:\Program Files\MATLAB\R2017b
"%MATLAB%\bin\win64\gmake" -f new_wall_model.mk  ISPROTECTINGMODEL=NOTPROTECTING
