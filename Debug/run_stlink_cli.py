import subprocess
subprocess.Popen("ST-LINK_CLI.exe -c SWD -p \"D:\projects\stm32\drone_project\IMU_project\Debug\IMU_project.bin\" 0x08000000 -Rst")