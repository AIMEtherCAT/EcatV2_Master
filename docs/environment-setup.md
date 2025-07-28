## EtherCAT Environment Setup Tutorial

### BIOS Setup

Since BIOS layouts vary by manufacturer, the configuration items listed below may have different names or be missing on your machine. If you cannot find specific items, feel free to skip them.

* **Disable** Race To Halt (RTH)
* **Disable** Hyper-Threading
* **Disable** Virtualization Support
* **Disable** C-State Support
* If your system can **lock the CPU frequency**, **disable** TurboBoost/SpeedStep/SpeedShift, and set the CPU frequency to a fixed value

### System Setup

**Ubuntu 22.04** is recommended for this application.

#### Register Ubuntu One Account

Go to https://login.ubuntu.com/ and register for an account.

#### Attach and enable Ubuntu Pro

When you finish the system installation, enable the **Ubuntu Pro**. It can be enabled in the pop-up window when entering the system for the first time after installation. If you missed this window, you can also use the command ``sudo pro attach`` in the terminal to enable it.

![pro-attach](img/pro-attach.png)

#### Enable Realtime-Kernel

After attaching to the Ubuntu Pro, open the terminal.

If you are using Intel **12th** Gen CPU, use the command ``sudo pro enable realtime-kernel-- variant=intel-iotg`` in the terminal to enable the realtime-kernel patch, which is optimised for this generation of CPU.

If not, use the command ``sudo pro enable realtime-kernel`` to enable the generic realtime-kernel patch.

![rt-kernel](img/rt-kernel.png)

When it finishes, restart your computer.

#### Isolate a CPU core

Select one core that you want to use to run the SOEM independently.

For CPUs without a distinction between performance and efficiency cores, you can directly select CPU0.

For Hybrid Architecture CPUs (with both performance and efficiency cores), it's recommended to use the efficiency cores if their base frequency exceeds 2 GHz; otherwise, the performance cores are preferred.

Please note that CPU numbering **starts from 0**, and typically the performance cores come first, followed by the efficiency cores.

After selecting a core number (Let's refer to it as **X**), we can know the ID of other core numbers (Let's refer to it as **Y**). 

Then edit the grub boot commands by appending this content to the end of the **linux** command: ``nohz=on nohz_full=X rcu_nocbs=X isolcpus=X irqaffinity=Y``

![grub-cmd](img/grub-cmd.png)

For example, if we have an 8-core CPU and we select CPU0 to isolate, then X=0 and Y=1,2,3,4,5,6,7, and the final content to be appended will be ``nohz=on nohz_full=0 rcu_nocbs=0 isolcpus=0 irqaffinity=1,2,3,4,5,6,7``

This step can be easily done by using the **grub-customizer** application, which is a grub configuration editor. Please search for the installation manual yourself.

<img src="img/grub-customizer.png" alt="grub-customizer" style="zoom:50%;" />

After finishing, reboot your computer.

If you want to confirm whether the configuration was successful, you can use the **htop** monitor. If the CPU usage of the selected CPU in htop can continuously stay at 0% or 1%, it indicates that the configuration succeeded. 

![htop](img/htop.png)

You can also use the command `cat /proc/cmdline` to check if the GRUB boot parameters are saved. 

![cmdline](img/cmdline.png)

If any errors occur, please read the manual and try again.

#### Setup ROS2 Workspace

Before this step, please confirm your local SSH public key is added to your GitHub account, which means your computer can access the repository https://github.com/AIMEtherCAT/EcatV2_Master

After that, enter your workspace folder and create a ``src`` folder.

Run the command ``git submodule add git@github.com:AIMEtherCAT/EcatV2_Master.git src/EcatV2_Master`` to add some_wrapper to your project.

If this project updates in the future, use the command ``git submodule update --recursive --remote`` to update code for some_wrapper.

### Done

After finishing these steps, your system is now available to run the SOEM application. Please then refer to the next tutorial about how to use our soem_wrapper application.