## EtherCAT Environment Setup Tutorial

### BIOS Setup

Since BIOS layouts vary by manufacturer, the configuration items listed below may have different names or be missing on your machine. If you cannot find specific items, feel free to skip them.

* **Disable** Race To Halt (RTH)
* ~~**Disable** Hyper-Threading~~ (Updated: If you system resources is limited, you may don't want to lose half of your
  threads. In this case you can ignore this item.)
* **Disable** Virtualization Support
* **Disable** C-State Support
* If your system can **lock the CPU frequency**, **disable** TurboBoost/SpeedStep/SpeedShift, and set the CPU frequency to a fixed value

### System Setup

**Ubuntu 22.04** with **ROS2 Humble** is recommended for this application.

**Ubuntu 24.04** with **ROS2 Jazzy** is also working well.

#### Register Ubuntu One Account

Go to https://login.ubuntu.com/ and register for an account.

#### Attach and enable Ubuntu Pro

When you finish the system installation, enable the **Ubuntu Pro**. It can be enabled in the pop-up window when entering the system for the first time after installation. If you missed this window, you can also use the command ``sudo pro attach`` in the terminal to enable it.

![pro-attach](img/pro-attach.png)

#### Enable Realtime-Kernel

After attaching to the Ubuntu Pro, open the terminal.

~~If you are using Intel **12th** Gen CPU, use the command ``sudo pro enable realtime-kernel --variant=intel-iotg`` in the terminal to enable the realtime-kernel patch, which is specially optimised for this generation of CPU.~~ (update: no very impressive difference identified, just choose whatever you like)

If not, use the command ``sudo pro enable realtime-kernel`` to enable the generic realtime-kernel patch.

![rt-kernel](img/rt-kernel.png)

When it finishes, restart your computer.

#### Isolate a CPU core

Select one core that you want to use to run the SOEM independently.

For CPUs without a distinction between performance and efficiency cores, you can directly select CPU0.

For Hybrid Architecture CPUs (with both performance and efficiency cores), it's recommended to use the efficiency cores if their base frequency exceeds 2 GHz; otherwise, the performance cores are preferred.

> **Note1.** For CPUs **with hyper-threading function enabled**, you should isolate **entire** physical cores. This
> means, for example, if your CPU has 2 cores and 4 threads, then **CPU0 & CPU1 belong to core 0**, and CPU2 & CPU3 belong
> to core 1. Since a full core needs to be isolated, you must at least isolate **CPU0 and CPU1** in this case. You can
> then choose **first one** of them **for later use** as needed.

> **Note2.** Please note that CPU numbering **starts from 0**, and typically the performance cores come first, followed by the efficiency cores.

After selecting a core number (Let's refer to it as **X**), we can know the ID of other core numbers (Let's refer to it
as **Y**).

Then edit the grub boot commands by appending this content to the end of the **linux** command: ``nohz=on nohz_full=X rcu_nocbs=X isolcpus=X irqaffinity=Y``

![grub-cmd](img/grub-cmd.png)

For example, if we have an 8-core CPU, and we select CPU0 to isolate, then X=0 and Y=1,2,3,4,5,6,7, and the final
content to be appended will be ``nohz=on nohz_full=0 rcu_nocbs=0 isolcpus=0 irqaffinity=1,2,3,4,5,6,7``

This step can be easily done by using the **grub-customizer** application, which is a grub configuration editor. Please search for the installation manual yourself.

<img src="img/grub-customizer.png" alt="grub-customizer" style="zoom:50%;" />

After finishing, reboot your computer.

If you want to confirm whether the configuration was successful, you can use the **htop** monitor. If the CPU usage of
the selected CPU in htop can continuously stay at 0% or 1%, it indicates that the configuration succeeded.

![htop](img/htop.png)

You can also use the command `cat /proc/cmdline` to check if the GRUB boot parameters are saved.

![cmdline](img/cmdline.png)

If any errors occur, please read the manual and try again.

#### Setup ROS2 Workspace

Before this step, please confirm your local SSH public key is added to your GitHub account, which means your computer can access the repository https://github.com/AIMEtherCAT/EcatV2_Master

After that, enter your workspace folder and create a ``src`` folder.

If you haven't initialized a git repository here, please run the command ``git init`` first.

Then, run the command ``git submodule add git@github.com:AIMEtherCAT/EcatV2_Master.git src/EcatV2_Master`` to add
`soem_wrapper` to your project.

After that, use the command ``git submodule update --init --recursive`` to fetch the code of `soem`.

If this project updates in the future, you can use the command ``git submodule update --recursive --remote`` to 
update code for soem_wrapper.

### Flash EEPROM For EtherCAT Module

Now you may have cloned our ``soem_wrapper `` into your workspace.

Go to the root path of your workspace, you should now be able to access these two folders: ``src/EcatV2_Master/tools`` and ``src/EcatV2_Master/eeproms``

![tools-tree.png](img/tools-tree.png)

Use the command ``sudo su`` to switch to the root user. Then use the ``chmod +x src/EcatV2_Master/tools/*`` command to make these tools executable.

In the folder ``src/EcatV2_Master/eeproms``, you can find EEPROM files for all available modules. Select one that you want to flash into your module. Use the command ``src/EcatV2_Master/tools/eepromtool <NIC name> <slave id> -w <eeprom file path>`` to flash the EEPROM for your EtherCAT module.

For example, if you want to flash a ``H750 Universal Module``, your NIC name is ``enp3s0``, and this module is the first module in the connection chain (which means it connects to your computer's NIC **directly**). Then the command you use would be ``src/EcatV2_Master/tools/eepromtool enp3s0 1 -w src/EcatV2_Master/eeproms/58100H750_UniversalModule.bin``.

You could run this command multiple times to ensure flashing progress is finished successfully.

![flash-done.png](img/flash-done.png)

After that, you need to power off and re-power on the EtherCAT module to make it effective.

### Flash Firmware for MCU

Usually, this step should be done before you get the EtherCAT module from the manufacturing partners, but if you want to
update the firmware or have any problems, you can re-flash the firmware of your EtherCAT module.

You could build it yourself, and we also provide built artifacts; you can find them on the GitHub Action page.

Since this may not be a compulsory step, you may need to find out how to flash firmware by yourself. The format of our
pre-built artifacts is ``.elf``

Firmwares available now:

* [H750 Universal Module](https://github.com/AIMEtherCAT/EcatV2_AX58100_H750_Universal)

### Done

After finishing these steps, your system is now available to run the SOEM application. Please then refer to the next tutorial about how to run your first test.