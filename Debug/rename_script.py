#This script deletes the existing .bin file and renames the .binary to .bin

import os
import subprocess

commandString="ST-LINK_CLI.exe -c SWD -p \"D:\projects\stm32\drone_project\IMU_project\Debug\IMU_project.bin\" 0x08000000 -Rst"


print('----------------------------------------------------')

if os.path.exists("IMU_project.bin"):
  os.remove("IMU_project.bin")
  print('delete success')
else:
  print("The file: 'IMU_project.bin' does not exist") 

if os.path.exists("IMU_project.binary"):
  os.rename('IMU_project.binary','IMU_project.bin')
  print('rename success success')
  #run st-link_cli on shell
  process = subprocess.Popen(commandString)
  #wait for st-link_cli to exit
  process.communicate()
else:
  print("The file: 'IMU_project.binary' does not exist") 

print('----------------------------------------------------\n')

