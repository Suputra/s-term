# Notepad-to-Terminal Text Paste

## Status: DONE

## Problem
Users want to draft notes in notepad mode, then paste them into an SSH session
(e.g., writing a command, config snippet, or message in notepad, then sending it).

## Design
- **Alt+P** in terminal mode: send entire notepad text_buf over SSH
- **Alt+V** in terminal mode: send notepad text_buf line-by-line with \r between lines
- The notepad buffer persists across mode switches (already the case)
- Read text_buf under mutex, then send via sshSendString
- Rate-limit the send (small delay between chunks) to avoid overwhelming the remote shell
- Show "Pasting..." in terminal status bar during send
