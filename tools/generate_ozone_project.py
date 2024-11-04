#!/usr/bin/env python3
#
# Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
#
# SPDX-License-Identifier: BSD-3-Clause
#
#
# Little script to build a header file including every other header file in the SDK!
# (also checks we don't have "conflicting" header-filenames)
# Edit the IGNORE_DIRS variable to filter out which directories get looked in.
#
# Usage:
#
# tools/generate_ozone_project PICO_PLATFORM ELF-File Ozone-Project-File
#
# Example:
# # ./generate_ozone_project 'rp2350-arm-s' '/Users/mringwal/Projects/pico/pico-examples/ninja/pico_w/bt/gatt_counter/picow_bt_example_gatt_counter_poll.elf' 'project.jdebug'

import sys

# Inline project template
project_template = \
'''

// Stripped down version of generic Ozone project file for pico-sdk

/*********************************************************************
*
*       OnProjectLoad
*
* Function description
*   Project load routine. Required.
*
**********************************************************************
*/
void OnProjectLoad (void) {{

 	Project.SetDevice ("{device_type}");
 	Project.SetTargetIF ("SWD");
 	Project.SetTIFSpeed ("4 MHz");
 	Project.AddSvdFile ("$(InstallDir)/Config/CPU/{cpu_type}.svd");

 	File.Open ("{elf_file}");
}}
'''

# Map PICO_PLATFORM to SEGGER Ozone
device_mapping = {
	'rp2040'       : ('RP2040_M0_0',   'Cortex-M0'  ),
	'rp2350-arm-s' : ('RP2350_M33_0',  'Cortex-M33F'),
	'rp2350-riscv' : ('RP2350_RV32_0', 'RV32IFD'    )
}

def generate_project(pico_platform, elf_file, out_file):
	# Get platform info
	device_type, cpu_type = device_mapping[pico_platform]
	with open(out_file, 'w') as writer:
		writer.write(project_template.format(device_type=device_type, cpu_type=cpu_type, elf_file=elf_file))


# Input from command line
if len(sys.argv) != 4:
    print("Usage: {} PICO_PLATFORM ELF-File Ozone-Project-File".format(os.path.basename(sys.argv[0])))
    sys.exit(1)
_, PICO_PLATFORM, ELF_FILE, OUT_FILE = sys.argv

# generate project
generate_project(PICO_PLATFORM, ELF_FILE, OUT_FILE)


