# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Unreleased

## [0.0.6] 2023-07-05
### Fixed
- The leakage of descriptors for pins configured as outputs.

## [0.0.5] - 2023-07-05
### Fixed
- The leakage of descriptors for pins configured as inputs.
- Repeated closing of descriptors when destroying a GPIO object that has 
already been closed.

## [0.0.4] - 2023-05-25
### Added
- `kDeviceNotFound` error.

## [0.0.3] - 2023-05-25
### Changed
- Refactoring.

## [0.0.2] - 2023-05-24
### Changed
- Error handling approach.

## [0.0.1] - 2022-05-18
### Added
- Sources :)

[0.0.4]: https://github.com/uncellon/utoolbox-gpio/tree/v0.0.4
[0.0.3]: https://github.com/uncellon/utoolbox-gpio/tree/v0.0.3
[0.0.2]: https://github.com/uncellon/utoolbox-gpio/tree/v0.0.2
[0.0.1]: https://github.com/uncellon/utoolbox-gpio/tree/v0.0.1