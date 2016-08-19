====
HOW TO IMPORT THIS PROJECT INTO LPCExpresso IDE v8.1.4+
====

This procedure is how I got things downloaded from Github on Windows 10 on my Surface Pro 2 laptop using GitBash command line from Git. Feel free to adapt to your local operating system and methods and share.

1. Clone this repo from Github via your favorite method.

2. Make sure that there is a Binaries folder (empty ok) under the Example01 folder:

	cd <main folder of project>
	mkdir Example01/Binaries

3. Create a .ZIP file (compressed folder on Windows) containing the following folder trees:

	CMSISv1p30_LPC17xx/
	Example01/
	FreeRTOS_Library/
	FreeRTOS-Products/
	lpc17xx.cmsis.driver.library/

4. Remove the preceding folders and their contents in their entirety (trust me!):

	rmdir -f C*
	rmdir -f E*
	rmdir -f F*
	rmdir -f lpc*

5. Open the IDE to this folder as a workspace (it will create a .metadata folder if needed).

6. Use 'Import project ...' in the Quick Settings (or File menu) to import the ZIP file you created.

7. You should now have everything you need. Best of luck with this project! Let me know if you have a more streamlined version of this process (mike@azuresults.com).

-- Mike Mehr
Mountain View, California, US
August 18, 2016

====
Comments:
MM 2016/08/18 - If you skip the Binaries folder, the IDE seemingly cannot create it itself.
If you skip the folder removal, the IDE refuses to load the project, since files already exist.
However, it will not refresh without loading the ZIP file in the first place.
I will try skipping step 4 in favor of an F5 refresh of the project, pretty sure I tried that.
Feel free to add if improved.
