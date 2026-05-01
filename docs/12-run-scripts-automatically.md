12. [How to Automatically Run Scripts When Your Computer Starts](#how-to-automatically-run-scripts-when-your-computer-starts)


# How to Automatically Run Scripts When Your Computer Starts
This guide explains in simple terms how to make your computer automatically run a script or program every time it starts or when you log in. You don’t need to be a programmer, just follow the steps below.

Imagine you built a small weather station or camera project on your Raspberry Pi. Every time you restart it, you have to open the terminal and run a command to start your program. That gets tiring quickly. By using the methods below, you can make your Raspberry Pi automatically start your script every time it powers on, no typing needed. 

The @reboot method tells your Pi, “Run this every time I start up,” while the systemd method keeps your program running in the background even if something crashes.
Once you set this up, you can just plug in your Pi and your project will launch on its own. This makes your setup work like a real automated system, ideal for sensors, data loggers, dashboards, or smart displays that need to run all the time.


**For Linux Users (including Raspberry Pi)**
Linux systems can run scripts automatically at startup or login. You can choose between a quick cron method or a more reliable systemd service.
**Option 1:** Using `@reboot` in crontab 
1.	Open the Terminal.
2.	Type:
3.	crontab -e
This opens your personal startup list.

4.	Scroll to the bottom and add this line:
5.	`@reboot /home/pi/myscript.sh`
  - Replace "/home/pi/myscript.sh" with the full path to your script.
6.	Save and exit (Ctrl + X, then Y, then Enter).
7.	Make the script executable:
8.	chmod +x /home/pi/myscript.sh
9.	Reboot your Pi. The script will start automatically after every boot.

**Option 2:** Using systemd (More Reliable and Professional Way)
1.	Open Terminal.
2.	Create a new service file:
- `sudo nano /etc/systemd/system/myscript.service`
3.	Paste this inside: [Unit]
4. Description=Run my script at boot
5. After=network-online.target
6. Wants=network-online.target
7. [Service]
8. ExecStart=/home/pi/myscript.sh
9. WorkingDirectory=/home/pi
10. StandardOutput=inherit
11. StandardError=inherit
12. Restart=always
13. User=pi
14. [Install]
15. WantedBy=multi-user.target
Change paths or username if needed.
16. Save and exit (Ctrl + X, then Y, then Enter).
17. Enable and start the service:
18. sudo systemctl daemon-reload
19. sudo systemctl enable myscript.service
20. sudo systemctl start myscript.service
21. Check its status:
22. systemctl status myscript.service
23. Reboot to confirm it launches automatically.

Extra Tips
- Always use the full path to your script and test it manually first.
- Add a line at the top of your script to define the shell, for example:
  - `#!/bin/bash`
  - If your script depends on Wi-Fi or other services, start it with a delay:
      - `sleep 10`
    - To view logs, use:
      - `journalctl -u myscript.service -e`
