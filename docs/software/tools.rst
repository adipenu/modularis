Software Tools
==============

Git Installation and Configuration
-----------------------------------

To install git you will need to open the terminal and enter the following commands.

.. code-block:: bash

    sudo apt install git

This command will install git from the APT tool usually installed in most linux distros by default. Once installed you will need to configure a few things. To do this run the following commands:

.. code-block:: bash

    git config --global user.name "FirstName LastName"
    git config --global user.email "email@email.com"

This should mostly setup whatever you need till the first time you push a repository to whichever Git distributor you are using. 
Then it will ask for a user name and password, generally we use GitHub which requires a Personal Access Token which you can generate
under Settings -> Developer Settings -> and Personal Access Tokens. To ensure you don't need to keep entering this on your computer you can run
the following command

.. code-block:: bash 

    git config --global credential.helper store

This will store your access token with the local git so you don't have to keep re-entering it.

Visual Studio Code Installation
-------------------------------

To install vscode you can run the following command, though sometimes you have to download it from their website and run another command which is given after.

.. code-block:: bash

    snap install code

If this does not work, go to the `Visual Studio Code website <https://code.visualstudio.com/Download>`_ and download the linux package. Then you can run the follwing command to install it.

.. code-block:: bash

    cd Downloads && sudo dpkg -i <package_name.deb>

.. note:: Please change the ``package_name.deb`` to what ever your package is named.

This should be all you need to run everything that we have provided.

Recommended Extensions
^^^^^^^^^^^^^^^^^^^^^^
Extensions can be installed within the application by using the "Extensions" side bar. Just search the names and click install.

#. Python by Microsoft
#. ROS by Microsoft
#. YAML by Redhat

Byobu
-----
`Byobu <https://www.byobu.org/>`_ is a useful tool when working with multiple terminals. It is recommened to get familiar with it or any other terminal mutliplexer.

.. code-block:: bash

    sudo apt install byobu