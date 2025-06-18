
# Zephyr setting up guide


## Cloning the Zephyr repository

1. Open a terminal and clone ADI's Zephyr repository by running
`git clone https://github.com/analogdevicesinc/zephyr.git`

2. Checkout the `adin6310_switch` branch
`git checkout adin6310_switch`

# Linux

## Setting up Zephyr

1. Install Zephyr's dependencies by running the following in a terminal:
```
	sudo apt install --no-install-recommends git cmake ninja-build gperf \
	ccache dfu-util device-tree-compiler wget \
	python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
	make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1 python3-venv
```
  2. Setup a Python virtual environment in the zephyr directory:
`cd zephyr`
`python3 -m venv .venv`
`source .venv/bin/activate`

3. Install the Python dependencies
`pip3 install west`
`pip3 install -r scripts/requirements.txt`

4. Create a west project
west is Zephyr's metatool used for building projects, flashing, running tests and managing external modules.
First, we'll have to create an empty west project in the `zephyr` directory:
`west init -l .`

5. Pull Zephyr's external modules and export the CMake package:
`west update`
`west zephyr-export` 

6. Install Zephyr SDK:
This is a package which contains compilers (plus the associated libraries and GNU tools) for all the CPU architectures supported by Zephyr. In order to download and install the SDK, run the following in the `zephyr` directory:
```
wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_linux-x86_64.tar.xz
```
`tar xvf zephyr-sdk-0.16.8_linux-x86_64.tar.xz`
`cd zephyr-sdk-0.16.8`
`./setup.sh`

Next, you'll have to set the `ZEPHYR_TOOLCHAIN_VARIANT` variable:
`export ZEPHYR_TOOLCHAIN_VARIANT=zephyr`

In order to permanently set the variable's value, you may export it in your terminal's rc file (.bashrc, .zshrc etc). Otherwise, you'll have to set it every time you open a new terminal.

# Windows
## Setting up Zephyr
1. Install Chocolatey package manager.
This will be used in order to install some of the dependencies required by Zephyr (cmake, ninja, gperf, python311, git, dtc-msys2, wget, 7zip). Installing them from their official web site will also work.

	Open a PowerShell terminal and type:
	`choco feature enable -n allowGlobalConfirmation`
	This will allow installing packages without the need to confirm each of them.

	And then, we can install Chocolatey
	```
	Set-ExecutionPolicy Bypass -Scope Process -Force; [System.Net.ServicePointManager]::SecurityProtocol = [System.Net.ServicePointManager]::SecurityProtocol -bor 3072; iex ((New-Object System.Net.WebClient).DownloadString('https://community.chocolatey.org/install.ps1'))
	```
	Press Enter and wait for Chocolatey to install.

2. Install the required dependencies:
	```
	choco install cmake --installargs 'ADD_CMAKE_TO_PATH=System'
	choco install ninja gperf python311 git dtc-msys2 wget 7zip
	```

3. Navigate to the `zephyr` directory and create a Python virtual environment:

	`python -m venv .venv`

	Next, you'll need to activate it. This step has to be done every time you open a new terminal.

	`.venv\Scripts\activate.bat`

4. Install the Python dependencies
`pip3 install west`
`pip3 install -r scripts/requirements.txt`

5. Create a west project
west is Zephyr's metatool used for building projects, flashing, running tests and managing external modules.
First, we'll have to create an empty west project in the `zephyr` directory:

	`west init -l .`

6. Pull Zephyr's external modules and export the CMake package:
`west update`
`west zephyr-export` 

7. Install Zephyr SDK:
This is a package which contains compilers (plus the associated libraries and GNU tools) for all the CPU architectures supported by Zephyr. In order to download and install the SDK, run the following in the `zephyr` directory:
	```
	wget https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.8/zephyr-sdk-0.16.8_windows-x86_64.7z
	```
	```
	7z x zephyr-sdk-0.16.8_windows-x86_64.7z
	cd zephyr-sdk-0.16.8
	setup.cmd
	```

## Compiling the ADIN6310 project
The ADIN6310 example project can be found under `samples/application_development/adin6310`. In addition, you will need to have the source code for the ADIN6310 driver.

In order to compile the project you need to run the following:

`west build -b adin6310t1l/max32690/m4 samples/application_development/adin6310 -DLIB_ADIN6310_PATH=...`

The path to the ADIN6310 SES driver specified using the LIB_ADIN6310_PATH variable shouldn't contain any spacing.

The ADIN6310 software package should have the following structure:
```
.
└── ADINx310_TSN_Driver_Library_Rel5.0.0 <-- LIB_ADIN6310_PATH should be set to this
    ├── bin
    ├── doc
    ├── examples
    ├── lic
    └── src
```
## Programming the EVAL-ADIN6310T1LEBZ board

Depending on the debug probe used, the microcontroller may be programmed as following:

1. SEGGER J-Link:
Please note that you will need to have the J-Link software toolchain (may be downloaded from this page https://www.segger.com/downloads/jlink/) installed and accessible from your `PATH` variable (both for Windows and Linux).
	- Using west:
		`west flash --runner=jlink`
	- Using the JFlash (or JFlashLite) utility:
	Open JFlashLite and select the MAX32690 MCU as the target. Then, you can program the .hex file found at the `build/zephyr/zephyr.hex` path (in the `zephyr` directory).
