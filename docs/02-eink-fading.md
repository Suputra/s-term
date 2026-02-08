# E-Ink Fading / Ghosting Fix

## Status: DONE

## Problem
E-ink partial refreshes leave ghost artifacts that accumulate over time.
Currently a full clean refresh happens every 30 partial updates, but this
may not be aggressive enough (or may be too aggressive for fast typing).

## Design
1. Lower the partial_count threshold from 30 to 20 for more frequent cleaning
2. Add a manual full-refresh shortcut: Alt+F in both notepad and terminal modes
3. In terminal mode, use a longer debounce (200ms instead of 100ms) for
   fast-scrolling output to reduce unnecessary partial refreshes
4. After SSH disconnect/connect events, force a full clean refresh
