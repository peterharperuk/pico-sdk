#!/usr/bin/env python3
#
# Copyright (c) 2024 Raspberry Pi Ltd.
#
# SPDX-License-Identifier: BSD-3-Clause
#
#
# Simple script to check basic validity of a board-header-file
#
# Usage:
#
# tools/check_board_header.py src/boards/include/boards/<board.h>


import re
import sys
import os.path
import json
import warnings

from collections import namedtuple

# warnings off by default, because some boards use the same pin for multiple purposes
show_warnings = False

interfaces_json = "src/rp2040/rp2040_interface_pins.json"
if not os.path.isfile(interfaces_json):
    raise Exception("{} doesn't exist".format(interfaces_json))

board_header = sys.argv[1]
if not os.path.isfile(board_header):
    raise Exception("{} doesn't exist".format(board_header))

with open(interfaces_json) as interfaces_fh:
    interfaces = json.load(interfaces_fh)
    # convert instance-keys to integers (allowed by Python but not by JSON)
    for interface in interfaces:
        for instance in list(interfaces[interface]):
            interfaces[interface][int(instance)] = interfaces[interface].pop(instance)

DefineType = namedtuple("DefineType", ["name", "value", "resolved_value", "lineno"])

defines = dict()
pins = dict() # dict of lists
has_include_guard = False
has_board_detection = False
has_include_suggestion = False
expected_include_suggestion = "/".join(board_header.split("/")[-2:])
expected_include_guard = "_" + re.sub(r"\W", "_", expected_include_suggestion.upper())
expected_board_detection = re.sub(r"\W", "_", expected_include_suggestion.split("/")[-1].upper()[:-2])

with open(board_header) as header_fh:
    last_ifndef = None
    last_ifndef_lineno = -1
    board_detection_is_next = False
    for lineno, line in enumerate(header_fh.readlines()):
        lineno += 1
        # strip trailing comments
        line = re.sub(r"(?<=\S)\s*//.*$", "", line)

        # look for board-detection comment
        if re.match("// For board detection", line):
            board_detection_is_next = True
            continue
        # check include-suggestion
        m = re.match(r"^// This header may be included by other board headers as \"(.+?)\"", line)
        if m:
            include_suggestion = m.group(1)
            if include_suggestion == expected_include_suggestion:
                has_include_suggestion = True
            else:
                raise Exception("{}:{}  Suggests including \"{}\" but file is named \"{}\"".format(board_header, lineno, include_suggestion, expected_include_suggestion))
        # look for "#ifndef BLAH_BLAH"
        m = re.match(r"^#ifndef (\w+)\s*$", line)
        if m:
            last_ifndef = m.group(1)
            last_ifndef_lineno = lineno
        # look for "#define BLAH_BLAH" or "#define BLAH_BLAH 42"
        m = re.match(r"^#define (\w+)(?:\s+(.+?))?\s*$", line)
        if m:
            #print(m.groups())
            name = m.group(1)
            value = m.group(2)
            # check all uppercase
            if name != name.upper():
                raise Exception("{}:{}  Expected \"{}\" to be all uppercase".format(board_header, lineno, name))
            # check that adjacent #ifndef and #define lines match up
            if last_ifndef_lineno + 1 == lineno:
                if last_ifndef != name:
                    raise Exception("{}:{}  #ifndef {} / #define {} mismatch".format(board_header, last_ifndef_lineno, last_ifndef, name))
            if value:
                try:
                    # most board-defines are integer values
                    value = int(value, 0)
                except ValueError:
                    pass

                # resolve nested defines
                resolved_value = value
                while resolved_value in defines:
                    resolved_value = defines[resolved_value].resolved_value
            else:
                resolved_value = None

            define = DefineType(name, value, resolved_value, lineno)

            # check the include-guard define
            if re.match(r"^_BOARDS_(\w+)_H$", name):
                # check it has an #ifndef
                if last_ifndef_lineno +1 != lineno:
                    raise Exception("{}:{}  Include-guard #define {} is missing an #ifndef".format(board_header, lineno, name))
                if value:
                    raise Exception("{}:{}  Include-guard #define {} shouldn't have a value".format(board_header, lineno, name))
                if len(defines):
                    raise Exception("{}:{}  Include-guard #define {} should be the first define".format(board_header, lineno, name))
                if name == expected_include_guard:
                    has_include_guard = True
                else:
                    raise Exception("{}:{}  Found include-guard #define {} but expected {}".format(board_header, lineno, name, expected_include_guard))
            # check board-detection define
            if board_detection_is_next:
                board_detection_is_next = False
                if value:
                    raise Exception("{}:{}  Board-detection #define {} shouldn't have a value".format(board_header, lineno, name))
                # this is a bit messy because pico.h does "#define RASPBERRYPI_PICO" and metrotech_xerxes_rp2040.h does "#define XERXES_RP2040"
                if name.endswith(expected_board_detection) or expected_board_detection.endswith(name):
                    has_board_detection = True
                else:
                    raise Exception("{}:{}  Board-detection #define {} should end with {}".format(board_header, lineno, name, expected_board_detection))
            # check for multiply-defined values
            if name in defines:
                raise Exception("{}:{}  Multiple definitions for {} ({} and {})".format(board_header, lineno, name, defines[name].value, value))
            else:
                defines[name] = define

            # check for pin-conflicts
            if name.endswith("_PIN"):
                if resolved_value is None:
                    raise Exception("{}:{}  {} is set to an undefined value".format(board_header, lineno, name))
                elif not isinstance(resolved_value, int):
                    raise Exception("{}:{}  {} resolves to a non-integer value {}".format(board_header, lineno, name, resolved_value))
                else:
                    if resolved_value in pins and resolved_value == value:
                        if show_warnings:
                            warnings.warn("{}:{}  Both {} and {} claim to be pin {}".format(board_header, lineno, pins[resolved_value][0].name, name, resolved_value))
                        pins[resolved_value].append(define)
                    else:
                        if not (0 <= resolved_value <= 29):
                            raise Exception("{}:{}  Pin {} for {} is outside of the allowed range".foramt(board_header, lineno, resolved_value, name))
                        pins[resolved_value] = [define]

#import pprint; pprint.pprint(dict(sorted(defines.items(), key=lambda x: x[1].lineno)))

# check for invalid DEFAULT mappings
for name, define in defines.items():
    m = re.match(r"^(PICO_DEFAULT_(\w+))_(\w+)_PIN$", name)
    if m:
        instance_name = m.group(1)
        interface = m.group(2)
        function = m.group(3)
        if interface == "WS2812":
            continue
        if interface not in interfaces:
            raise Exception("{}:{}  {} is defined but {} isn't in {}".format(board_header, define.lineno, name, interface, interfaces_json))
        if instance_name not in defines:
            raise Exception("{}:{}  {} is defined but {} isn't defined".format(board_header, define.lineno, name, instance_name))
        instance_define = defines[instance_name]
        instance = instance_define.resolved_value
        if instance not in interfaces[interface]:
            raise Exception("{}:{}  {} is set to an invalid instance {}".format(board_header, instance_define.lineno, instance_define, instance))
        if function not in interfaces[interface][instance]:
            raise Exception("{}:{}  {} is defined but {} isn't a valid function for {}".format(board_header, define.lineno, name, function, instance_define))
        if define.resolved_value not in interfaces[interface][instance][function]:
            raise Exception("{}:{}  {} is set to {} which isn't a valid pin for {} on {} {}".format(board_header, define.lineno, name, define.resolved_value, function, interface, instance))

if not has_include_guard:
    raise Exception("{} has no include-guard (expected {})".format(board_header, expected_include_guard))
if not has_board_detection and expected_board_detection != "NONE":
    raise Exception("{} has no board-detection #define (expected {})".format(board_header, expected_board_detection))
# lots of headers don't have this
#if not has_include_suggestion:
#    raise Exception("{} has no include-suggestion (expected {})".format(board_header, expected_include_suggestion))

