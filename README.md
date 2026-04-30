# `kiwi-measurements`
This `no_std` package provides generic representation of measurements
acquired by the sensors on the Kiwi Mainboard.

# Including in your crates
## `Cargo.toml`:
Add the following to `[dependencies]`:
```toml
kiwi-measurements = { version = "0.2" }
```

## Features
- `defmt`: Allows `defmt` logging of `SingleMeasurement` struct and `CommonMeasurement` enum.
- `serde`: In case standard library is enabled, provides `serde::Serialize` and `serde::Deserialize`.
