## Third-party EtherCAT Devices

Besides our own EtherCAT modules, slaves made by other manufacturers can be placed in the same EtherCAT ring.
Each kind of device is implemented as a class derived from `aim::ecat::device::SlaveDevice` and registered with its
vendor id and product code.

### Supported devices

| Device                               | Vendor ID    | Product code | Docs                                        |
|--------------------------------------|--------------|--------------|---------------------------------------------|
| H750 Universal Module                | `0x00000000` | `3`          | [Configuration](configuration-generator.md) |
| H750 Universal Module (Large PDO V.) | `0x00000000` | `4`          | [Configuration](configuration-generator.md) |
| SBT908E digital loadcell amplifier   | `0x00000FFB` | `1`          | [SBT908E](task-info/sbt908e.md)             |

Every slave found on the bus must be registered, otherwise the master refuses to start and prints the vendor id and
product code of the unknown slave.

### Configuration

Our own modules are identified by their serial number (`sn{SN}`). Third-party devices are identified by their position
in the ring only, starting from 1, and use the key `slave{index}`:

```yaml
slaves:
  - sn4587574:                 # our own module, see the configuration generator
      ...
  - slave2:                    # second slave in the ring
      pub_topic: !std::string '/ecat/sbt908e/force'
```