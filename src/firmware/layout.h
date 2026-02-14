#pragma once

#define MAX_TEXT_LEN    4096
#define CHAR_W          6
#define CHAR_H          8
#define MARGIN_X        0
#define MARGIN_Y        0
#define SCREEN_W        240
#define SCREEN_H        320
#define STATUS_H        10
#define COLS_PER_LINE   ((SCREEN_W - MARGIN_X * 2) / CHAR_W)
#define ROWS_PER_SCREEN ((SCREEN_H - MARGIN_Y - STATUS_H) / CHAR_H)

#define TERM_ROWS       100
#define TERM_COLS       COLS_PER_LINE
