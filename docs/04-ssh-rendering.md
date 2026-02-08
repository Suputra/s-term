# SSH Terminal Rendering Improvements

## Status: DONE (Phase 1 complete)

## Problem
The ANSI escape parser currently discards ALL escape sequences, only keeping
printable chars, newline, carriage return, backspace, and tab. This means:
- No cursor positioning (programs can't move cursor)
- No screen clearing (Ctrl+L doesn't visually clear)
- No line erasing (prompts with readline don't render correctly)
- Color codes stripped (fine for e-ink) but cursor codes also stripped (broken)

## Design: Minimal VT100 Parser
Handle these common CSI sequences (ESC [ params letter):

### Phase 1 - Critical
| Sequence | Meaning | Action |
|----------|---------|--------|
| CSI H / CSI ;H | Cursor home / position | Move term_cursor_row/col |
| CSI 2J | Clear screen | Clear all lines, cursor to 0,0 |
| CSI J | Clear from cursor to end | Clear lines below cursor |
| CSI K | Erase to end of line | Fill rest of current line with spaces |
| CSI A/B/C/D | Cursor up/down/right/left | Move cursor by N |

### Phase 2 - Nice to have
| Sequence | Meaning | Action |
|----------|---------|--------|
| CSI nP | Delete chars | Shift line left |
| CSI n@ | Insert chars | Shift line right |
| CSI nL | Insert lines | Scroll region down |
| CSI nM | Delete lines | Scroll region up |
| CSI r | Set scroll region | Track scroll region bounds |

### Rendering improvements
- Increase debounce to 200ms for fast output bursts
- Batch multiple SSH reads before triggering a render
- Force full refresh after clear-screen sequences
