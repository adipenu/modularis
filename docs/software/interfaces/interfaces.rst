Interfaces
==========

This page documents all the interfaces that are available for the Modularis project.

Tether Interface
=================

The following describes the steps involved in utilizing the tether with bluerobotics fathom connectors to ssh
into the onboard computer (raspberry pi in this case) to run commands and control Modularis.

Setting IPv4 address
^^^^^^^^^^^^^^^^^^^^

How to Connect to a Raspberry Pi Directly with an Ethernet Cable - YouTube

If you're connecting a Raspberry Pi to a topside computer via an Ethernet cable and you want them to communicate directly, you typically need to ensure they are on the same network with compatible IP addresses. Setting the IPv4 address on the topside computer's Ethernet settings to match the Raspberry Pi's IP address is one way to enable communication between the two devices.

Here's a general guide to achieve this:

1. Assign Static IP Address to Raspberry Pi:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - Access your Raspberry Pi's terminal or SSH into the Pi.
   - Edit the network configuration file on the Raspberry Pi. For example, for Raspberry Pi OS, you can use:
     
.. code-block:: bash

    sudo nano /etc/dhcpcd.conf

     
- Add or modify the following lines to set a static IP address:

.. code-block:: bash     

    interface eth0  # Use the appropriate interface (eth0 for Ethernet)
    static ip_address=10.138.204.129  # Set a static IP address for the Raspberry Pi
    static routers=192.168.1.1  # Set the IP address of the router or topside computer
    static domain_name_servers=8.8.8.8  # Set DNS servers (optional)

    
- Save the changes and exit the editor. Then, restart networking or reboot the Raspberry Pi to apply the new settings.

2. Configure Topside Computer:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - On the topside computer (e.g., Windows, macOS, or Linux), access the network settings for the Ethernet interface.
   - Set a static IP address that matches the network configuration of the Raspberry Pi. For instance, if you set the Raspberry Pi's IP address to 192.168.1.2, set the topside computer's IP address to something like 192.168.1.3.
   - Ensure that the subnet mask and other network settings match those configured on the Raspberry Pi.

3. Connect Devices:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - Use an Ethernet cable to physically connect the Raspberry Pi to the Ethernet port on the topside computer.

4. Testing Connection:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - After configuring both devices with compatible static IP addresses, test the connection by attempting to ping each device from the other. For example, from the topside computer's command prompt/terminal:

.. code-block:: bash

     ping 10.138.204.129  # Replace with Raspberry Pi's IP address

   
Similarly, try to ping the topside computer from the Raspberry Pi.



By setting compatible static IP addresses on both the Raspberry Pi and the topside computer within the same network range, you can establish a direct communication link between the two devices via Ethernet. Adjust the IP addresses and network settings as needed based on your specific network requirements.

SHH
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once you have set up a network connection between two devices, such as a topside computer and a Raspberry Pi, and have confirmed they can communicate with each other using compatible static IP addresses, you can utilize SSH (Secure Shell) to remotely access and control the Raspberry Pi from the topside computer.

Here's how SSH works in this context:

1. Enable SSH on the Raspberry Pi (if not already enabled):
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - Access your Raspberry Pi's terminal or GUI interface.
   - Run the following command:

.. code-block:: bash   

        sudo raspi-config

     
- In the configuration menu, navigate to "Interfacing Options" or "SSH" and enable SSH. Save the settings and exit.

2. Retrieve the Raspberry Pi's IP Address:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - On the Raspberry Pi, you can find its IP address by running:
   
.. code-block:: bash

     hostname -I

     
- Note down the IP address assigned to the Raspberry Pi. This is the address you'll use to SSH into the Pi.

Hostname: `aprilab`

3. SSH into the Raspberry Pi from the Topside Computer:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - On the topside computer (e.g., Windows, macOS, or Linux), open a terminal or command prompt.
   - Use the `ssh` command followed by the username and IP address of the Raspberry Pi to initiate the SSH connection. The default username on Raspberry Pi OS is usually `pi`.

.. code-block:: bash     

     ssh aprilab@10.138.204.129  # Replace with your Raspberry Pi's IP address

     
- If this is the first time connecting, you might receive a security prompt asking to confirm the authenticity of the host. Type 'yes' to proceed and enter the password for the Raspberry Pi's user account (default password for user 'pi' is usually 'raspberry' if unchanged).

**Password:** `apr1lab`

4. Controlling the Raspberry Pi via SSH:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - Once logged in through SSH, you'll have access to the Raspberry Pi's terminal command line. You can execute commands, edit files, manage software, and perform any other tasks as if you were directly using the Raspberry Pi's terminal.

5. Closing the SSH Session:
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
   - To exit the SSH session and return to the topside computer's terminal, simply type:
     
     exit

     
     This command will log you out of the Raspberry Pi and return you to the topside computer's command prompt.


