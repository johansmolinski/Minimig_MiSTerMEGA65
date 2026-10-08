Amiga 500 for MEGA65
====================

This is a Git submodule of the Amiga 500 core for the MEGA65 (AExp). It is a
fork of the MiSTer FPGA Minimig-AGA core, which is based on Minimig by Dennis
van Weeren, with heavy later modifications by many different people.

**Go to https://github.com/sy2002/AExp to learn more.**

The MEGA65 port is based on the
[MiSTer2MEGA65 framework](https://github.com/sy2002/MiSTer2MEGA65). We forked
the [MiSTer core](https://github.com/MiSTer-devel/Minimig-AGA_MiSTer), so that
we can easily track upstream changes and merge them into our MEGA65 port as
needed. The following structure is being used:

* [MiSTer](https://github.com/sy2002/Minimig_MiSTerMEGA65/tree/MiSTer)
  branch: An unmodified mirror of the
  [original upstream MiSTer core](https://github.com/MiSTer-devel/Minimig-AGA_MiSTer)
  (upstream branch `MiSTer`). Upstream changes enter our fork here first and
  are then merged into `develop` as needed. The original upstream README
  lives in this branch.
* [master](https://github.com/sy2002/Minimig_MiSTerMEGA65) branch: Contains
  our stable modifications of the upstream MiSTer core. It equals `develop`
  at each official AExp release.
* [develop](https://github.com/sy2002/Minimig_MiSTerMEGA65/tree/develop)
  branch: Is our kind-of stable work-in-progress (WIP) development branch.
  The AExp main repository tracks this branch.

On modifications:

* Our strategy is to reduce the modifications to the upstream core to the
  bare minimum. Every change to an original file carries a dated provenance
  comment, and the original code is kept as a comment next to it.
* The scope is an Amiga 500: OCS chipset, PAL, 68000 CPU
  ([fx68k](https://github.com/ijor/fx68k)), 512 KB Chip RAM plus 512 KB Slow
  RAM. AGA, the 68020 (TG68K), IDE, the Toccata sound card and the SDRAM /
  turbo paths are not used and partly removed or tied off.
* We made sure that the code is actually synthesizing using Vivado (used for
  the MEGA65), which is stricter and more unforgiving than Quartus (used for
  the MiSTer): the Altera `altsyncram` / `altera_mf` memories are replaced by
  portable inferred block RAM templates, and a mechanical compatibility sweep
  fixed multi-driven registers, SystemVerilog array syntax and similar
  constructs.
* We added `rtl/minimig_m65.v`, a thin wrapper that renames the
  underscore-prefixed ports of `minimig.v` (illegal VHDL identifiers) and
  ties off the unused subsystems, so that the core can be instantiated from
  the VHDL world of the MEGA65 framework.
* We made the keyboard, mouse, battery-backed real-time clock and the ADF
  floppy host channel (`IO_FPGA`) compatible with the MEGA65 core. The
  keyboard uses a real CIA handshake (`kbd_ack`) instead of fixed timing.
* We extended Paula's floppy controller (`paula_floppy.v`) so that the
  MEGA65's internal 3.5" drive can act as a real Amiga floppy drive
  ("Hardware Floppy"), including a faithful `DSKBYTR` register for copy
  protections such as Rob Northen Copylock. With the Hardware Floppy
  switched off, the controller behaves exactly like the upstream one.
