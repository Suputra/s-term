#pragma once

#define KEYPAD_ROWS 4
#define KEYPAD_COLS 10

#define IS_LSHIFT(r, c) ((r) == 3 && (c) == 9)
#define IS_RSHIFT(r, c) ((r) == 3 && (c) == 5)
#define IS_SHIFT(r, c)  (IS_LSHIFT(r, c) || IS_RSHIFT(r, c))
#define IS_SYM(r, c)    ((r) == 3 && (c) == 8)
#define IS_ALT(r, c)    ((r) == 2 && (c) == 0)
#define IS_MIC(r, c)    ((r) == 3 && (c) == 6)
#define IS_DEAD(r, c)   ((r) == 3 && (c) <= 4)

static const char keymap_lower[KEYPAD_ROWS][KEYPAD_COLS] = {
    { 'q', 'w', 'e', 'r', 't', 'y', 'u', 'i', 'o', 'p' },
    { 'a', 's', 'd', 'f', 'g', 'h', 'j', 'k', 'l', '\b' },
    {  0,  'z', 'x', 'c', 'v', 'b', 'n', 'm', '\b', '\n' },
    {  0,   0,   0,   0,   0,   0,   0,  ' ',   0,   0 },
};

static const char keymap_upper[KEYPAD_ROWS][KEYPAD_COLS] = {
    { 'Q', 'W', 'E', 'R', 'T', 'Y', 'U', 'I', 'O', 'P' },
    { 'A', 'S', 'D', 'F', 'G', 'H', 'J', 'K', 'L', '\b' },
    {  0,  'Z', 'X', 'C', 'V', 'B', 'N', 'M', '\b', '\n' },
    {  0,   0,   0,   0,   0,   0,   0,  ' ',   0,   0 },
};

static const char keymap_sym[KEYPAD_ROWS][KEYPAD_COLS] = {
    { '#', '1', '2', '3', '(', ')', '_', '-', '+', '@' },
    { '*', '4', '5', '6', '/', ':', ';', '\'', '"', '\b' },
    {  0,  '7', '8', '9', '?', '!', ',', '.', '$', '\n' },
    {  0,   0,   0,   0,   0,   0,   0,  ' ',   0,   0 },
};
