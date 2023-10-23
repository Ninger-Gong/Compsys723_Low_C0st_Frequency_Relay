This template is starting point for creating a project based on your custom C code.
It will provide you a default project to which you can add your software files. To
add files to a project, manually copy the file into the application directory (e.g. 
using Windows Explorer), then right click on your application project and select 
refresh.

You can also add files to the project using the Nios II Software Build Tools for Eclipse import function. 
Select File -> Import. 
Expand General and select File System in the Import Window and click Next.
Identify the appropriate source and destination directories.
Check the files you want to add and click Finish.

Instructions for running your assignment on a DE2 - 115 board
To run the project with the Altera DE2-115 board, please follow the instructions below
1. Connect the board to the computer, and connect the VGA screen, connect keyboard to the DE2-115
2. Program the board in Quartus II 13.0, with Quartus Programmer using provided freq_relay_controller.sof file
3. Open Nios II IDE, import test_freetos and test_freetos_bsp to workspace
Build the Project
4. Create a new hardware run configuration and make sure the target connection is USB blaster
5. Select Run As Nios II Hardware
