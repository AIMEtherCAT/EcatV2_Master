## FAQ

### 1. After enabling the real-time kernel, the system cannot boot

In such cases, the system may appear to **hang at the log output screen** during startup, or there may be **no signal output** or **a black screen** right after boot.

If you encounter this situation, enter the GRUB boot loader and modify the startup command. This can be done by pressing the **ESC** key **ONCE** **<u>after</u>** the **BIOS logo appears** and **<u>before</u>** the **system starts to boot**.

If everything is done well, you will now enter a page like this, with a title called GNU GRUB.

![grub](img/grub.png)

Now, select the **first one** and then press **E**, you will enter the editor window like this.

![grub-editor](img/grub-editor.jpg)

Append ``nomodeset`` to the end of the Linux command, which means here.

![grub-editor-done](img/grub-editor-done.jpg)

After modification, press **F10** to boot. Now your system should be able to boot into the system.

But this modification is a one-time solution. To prevent such issues later, manually edit and save the GRUB boot parameters again after entering the system. You can refer to the **Isolate a CPU core** section in the [environment setup tutorial](environment-setup.md) for guidance on this process.