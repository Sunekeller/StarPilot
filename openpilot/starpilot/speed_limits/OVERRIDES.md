# Driver inputs

Ordinary cruise +/- changes the driver's SET ceiling. It does not bypass SLC, create
a persistent override, or cross out the sign. The causal ledger still distinguishes
driver actions and consumed confirmations. SLC has no separate override reducer.

Normal accelerator handling belongs to the vehicle controls. SLC never raises or
latches its target from pedal speed. Confirmation remains available while the
driver is engaged, including when the native controls temporarily yield to the pedal.

Source loss uses the previous accepted limit plus the current offset, up to SET.
That remembered limit survives re-engagement and restarts; applying it still needs
current vehicle coordinates and system-longitudinal authority. See [README](README.md).
