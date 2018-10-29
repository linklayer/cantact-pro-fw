# CANtact Pro Firmware

This repository contains firmware for the CANtact Pro device. This firmware is
provided as a project for the NXP MCUXpresso IDE, which is available for free
from NXP.

## IDE and SDK

This IDE can be downloaded from NXP [here](https://www.nxp.com/support/developer-resources/software-development-tools/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE).

The project has been tested using MCUXpresso IDE v10.2.1 [Build 795] [2018-07-25] 
and SDK version 2.4.1. Since SDK file are bundled with the project, you should
not need to install the SDK.

## Importing Project

1. To import the project to MCUXpresso, first clone it using git: `git clone ssh://git@gitlab.com:linklayer/cantact-pro.git`
2. Open MCUXpresso 
3. Select File -> Import... and choose "Existing Projects into Workspace"
4. Click "Browse" beside "Select root directory" and select the location of the cloned firmware
5. Check the box beside the projects in the "Projects:" list
6. Click Finish
