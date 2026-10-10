# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->

## [Unreleased] - ReleaseDate

### Changed

* improved module documentation

## [0.1.0] - 2026-10-10

### Added

* The `rp-hub75` driver: one PIO state machine streams the framebuffer and two DMA
  channels play its BCM sequence, so the CPU takes one interrupt per frame.
* Four backend target features, exactly one required and none a default:
  `rphal-rp2040`, `rphal-rp2350`, `embassy-rp2040` and `embassy-rp2350`.
* `Hub75Pins16` for the fourteen-pin direct-drive bus and `Hub75Pins8` for the
  eight-signal latched bus, each derived from any GPIOs the PIO can drive and
  checked against the signal layout at startup.
* Bitplane framebuffers from `hub75-framebuffer`, direct-drive and latched, drawn
  with `embedded-graphics` and sent to the PIO without a conversion step.
* Panel tunings: `invert-clock`, `invert-oe`, `skip-black-pixels`,
  `tail-closes-latch`, the `lead-blank-*` / `trail-blank-*` / `inter-row-blank-*`
  delays and `reverse-row-order`.

<!-- next-url -->
[Unreleased]: https://github.com/liebman/rp-hub75/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/liebman/rp-hub75/compare/v0.1.0...v0.1.0
