#!/bin/bash
cd /home/vscode
wget 'https://github.com/wpilibsuite/vscode-wpilib/releases/download/v2025.2.1/vscode-wpilib-2025.2.1.vsix'
wget https://github.com/Mechanical-Advantage/AdvantageScope/releases/download/v4.1.6/advantagescope-linux-x64-v4.1.6.deb
sudo dpkg -i /home/vscode/advantagescope-linux-x64-v4.1.6.deb