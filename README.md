# ANT-BMS-ESP32-C3-UART-Comm

Reverse engineering and implementation of UART communication between ANT Battery Management Systems (BMS) and ESP32-C3-Mini microcontrollers with comprehensive protocol analysis and implementation.

## Project Overview

This project provides a complete solution for communicating with ANT-BMS systems using ESP32-C3-Mini microcontrollers, featuring both UART and Modbus protocol implementations, detailed protocol reverse engineering, and practical hardware integration examples.

## Project Structure

```
ANT-BMS-ESP32-C3-UART-Comm/
├── code/
│   ├── checksum/           # CRC validation utilities
│   ├── modbus/            # Modbus protocol implementation
│   └── uart/              # UART protocol implementation
├── data/
│   ├── protocol-captures/ # Raw protocol data captures
│   └── test-results/      # Validation and test data
├── docs/
│   ├── protocol-analysis/ # Reverse engineering documentation
│   ├── register-tables/   # BMS register mappings
├── hardware/
│   ├── datasheets/       # Component datasheets
│   └── schematics/       # Circuit diagrams
```




## Quick Start

###  Clone the Repository
```bash
git clone https://github.com/Merve-Sa/ANT-BMS-ESP32-C3-UART-Comm.git
cd ANT-BMS-ESP32-C3-UART-Comm
```





## Key Features

### Protocol Implementation
- Complete UART and Modbus protocol stacks
- Robust error handling and checksum validation
- Comprehensive register mapping and documentation
- Real-time data acquisition from BMS

### Hardware Integration
- ESP32-C3-Mini optimized firmware
- Modular hardware design

### Documentation
- Detailed reverse engineering analysis
- Protocol capture and analysis tools
- Complete register tables and command references



## Documentation Structure

### Protocol Analysis (`docs/protocol-analysis/`)
- **UART Analysis**: Complete packet structure and command documentation
- **Modbus Analysis**: Register mapping and function code implementation

### Register Tables (`docs/register-tables/`)
- Complete BMS register documentation
- Address mappings for all supported parameters
- Data format specifications

## Development Tools

### Checksum Validation (`code/checksum/`)
CRC calculation and validation utilities for protocol development:
```python
python antbms_crc_check.py --input <hex_data> --validate
```



## Academic Citation

If you use this project in academic research, please cite:

```
Sağlam, M. (2025). Reverse Engineering und Implementierung der UART-Kommunikation
zwischen BMS und ESP32-C3-Mini. Master's Thesis, Rheinland-Pfälzische Technische
Universität Kaiserslautern-Landau.
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Contact and Support

**Author**: Merve Sağlam  
**Institution**: Rheinland-Pfälzische Technische Universität Kaiserslautern-Landau  
**Email**: msaglam@rptu.de

For questions, issues, or contributions:
- Create an issue on GitHub
- Email the author for academic inquiries
- Check the documentation in `docs/` for detailed information

## Acknowledgments

- **Institution**: Rheinland-Pfälzische Technische Universität Kaiserslautern-Landau


---
