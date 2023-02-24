#ifndef _PICO_STDIO_BT_H
#define _PICO_STDIO_BT_H

// PICO_CONFIG: PICO_STDIO_BT_DEFAULT_CRLF, Default state of CR/LF translation for BT STDIO output, type=bool, default=PICO_STDIO_DEFAULT_CRLF, group=pico_stdio_bt
#ifndef PICO_STDIO_BT_DEFAULT_CRLF
#define PICO_STDIO_BT_DEFAULT_CRLF PICO_STDIO_DEFAULT_CRLF
#endif

bool stdio_bt_init(void);

#endif
