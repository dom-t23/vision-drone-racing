"""Vision-based autonomous drone racing.

Packages:
    vdr.sim       -- headless simulator (dynamics, IMU, FPV camera renderer, course, scoring)
    vdr.autonomy  -- the flight stack (detector, PnP, ESKF, guidance, control)
    vdr.interface -- the ONLY data contract between the two
    vdr.harness   -- seeded multi-run regression harness

The autonomy stack never imports vdr.sim; tests/test_separation.py enforces this.
"""

__version__ = "0.1.0"
