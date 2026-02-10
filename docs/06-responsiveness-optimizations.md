# 06 - Responsiveness Optimizations

Optimizations to reduce keypress-to-screen latency and eliminate wasted work.

## High Impact

- [x] 1. Reduce terminal render throttle from 200ms to 80ms (line 1633)
- [x] 2. Skip-to-scroll in `drawLinesRange()` — avoid scanning from buffer pos 0 (line 1291)
- [ ] 3. Tighten `refreshLines()` partial window to actual dirty lines only — SKIPPED: status bar at bottom forces full-height partial window anyway
- [x] 4. Batch character printing — build line buffer, print per-line not per-char (lines 1312, 1408)
- [x] 5. Eliminate redundant `computeLayoutFrom()` in `drawStatusBar()` — pass cached result (line 1358)
- [x] 6. Terminal snapshot: copy only visible rows, not all 100 (line 1394)

## Medium Impact

- [x] 7. Reduce battery/WiFi check from 1s to 10s (line 2608)
- [x] 8. Combine double mutex acquisition in notepad render path (lines 1679-1693)
- [ ] 9. Replace polling `vTaskDelay(1)` with task notifications in display task (lines 1656, 1668, 1675)
- [ ] 10. Combine `cursorUp()`/`cursorDown()` into single-pass scan (lines 1735-1771)

## Minor

- [ ] 11. Only zero used CSI params, not full array (line 427)
- [x] 12. Reduce SSH receive no-data delay from 50ms to 20ms (line 1283)
- [ ] 13. Skip status bar redraw when status unchanged
