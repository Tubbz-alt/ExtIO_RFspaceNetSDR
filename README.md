# ExtIO_RFspaceSDR
Winrad/HDSDR plugin ExtIO for the RFspace NetSDR, NetSDR+ and CloudIQ receiver

URL: https://github.com/hayguen/ExtIO_RFspaceNetSDR

## Software Defined Radio (SDR) receiver

The hardware receiver(s) were/are manufactured by RFspace.

See http://www.rfspace.com/

This software was initially developed for for the NetSDR model.
It does also run with the compatible NetSDR+ model.
And also with the different CloudIQ model.

Network protocol specification for the interface is available at
http://www.moetronix.com/svdownload.htm
labeled 'NetSDR Interface Spec'


## Origin of source code

The ExtIO was developed at PROCITEC GmbH,
a company for signal monitoring and analysis.

See https://procitec.de

They kindly gave permission to release the source code under LGPL (Lesser General Public License) Version 3.

Authors are Sebastian Balthasar and Hayati Ayguen. On questions, contact Hayati at h_ayguen@web.de


## Step-by-step installation

* install your favorite SDR software, e.g. HDSDR from http://www.hdsdr.de/ .
Other ExtIO compatible software like Winrad or Studio1 might also work.

* download ExtIO_RFspaceNetSDR.DLL https://github.com/hayguen/ExtIO_RFspaceNetSDR/releases

* copy downloaded file into your SDR software's installation directory (default=C:\Program Files (x86)\HDSDR).
Do NOT try to unzip directly into this directory! Because of Windows' user rights management this will fail. Unzip somewhere else first, then copy it to the installation directory.

* start the SDR software and close it again, after getting an error message, caused by wrong configuration.

* Edit one of config_Default-profile.reg (or config_NetSDR-profile.reg) with a Text-Editor,
e.g. Notepad++. Enter your IP address(es) and check if you need to modify the portnumber.
Use config_Default-profile.reg if you start HDSDR with it's installed shortcut.

* With a custom 'NetSDR' profile,
see https://sites.google.com/site/g4zfqradio/installing-and-using-hdsdr#Advanced ,
you might edit config_NetSDR-profile.reg

* Then doubleclick to import those settings, when HDSDR is closed

* exit and restart SDR software and select ExtIO_RFspaceSDR.DLL if demanded


## Compilation notes

Besides cloning this repository, you also need to clone following libraries from same directory:

* https://github.com/hayguen/tinythreadpp  for threading

* https://github.com/hayguen/clsocket  for tcp/ip socket communication

Microsoft Visual Studio Express 2013 (MSVC) IDE is used for compilation.
Other compilers (gcc/mingw) should work, when adding suitable project files or fixing CMakeLists.txt.


## Status

Compiles. In test.

ExtIO control GUI for configuration is missing! To be implemented ..

Workaround by editing Windows' registry directly with 'regedit.exe' for use with HDSDR.
 Registry path: HKEY_CURRENT_USER\Software\HDSDR\ExtIO_RFspaceSDR.dll

