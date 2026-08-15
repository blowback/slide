- multi-file batch send should send the number of files in the batch
- add commands so we can do remote filing: discover volumes, select volume, list files
- One optional hardening for your side of the fence: a CALL uart_flush_rx in slide.asm just before send_fin (and/or at .tx_done) would make
  the sender immune to any control residue — the noisy-line recovery paths (NAK storms, window replays) can still legitimately queue more
  controls than one-read-per-wait consumes. Not needed for the clean path anymore, but it's cheap belt-and-braces in your SLIDE repo.
