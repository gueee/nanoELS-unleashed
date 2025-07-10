# nanoELS Unleashed

**⚠️ IMPORTANT: This is an overhaul project based on the original nanoELS by Maxim Kachurovskiy**

All original work, design, and intellectual property belongs to **Maxim Kachurovskiy** - the creator of the original nanoELS project. This repository is a fork focused on improving usability and adding new features while maintaining full credit to the original author.

**Original Project:** [nanoELS by Maxim Kachurovskiy](https://github.com/kachurovskiy/nanoels)

---

## About This Project

This is an enhanced version of the nanoELS Electronic Lead Screw system, designed to improve upon the excellent work done by Maxim. The goal is to:

- **Improve usability** through better documentation and user experience
- **Add new features** while maintaining compatibility with original designs
- **Enhance reliability** and ease of assembly
- **Expand functionality** for modern workshop needs

### Enhanced Usability Features

- **Natural MPG Feel**: Improved manual pulse generator (MPG) support with acceleration based on encoder spin speed for more intuitive control
- **Touch Screen Integration**: Optional touch screen functionality for modern user interfaces
- **Single Axis Modes**: Dedicated modes for individual axis control and operation
- **Additional features planned as development progresses**

All hardware designs, software architecture, and core functionality remain the intellectual property of Maxim Kachurovskiy.

---

## Hardware Versions

### NanoEls H5 (Latest)
CNC and electronic lead screw controller based on ESP32-S3 that supports up to 3 axes:

- All the features of H4
- Cheaper and much easier to make than H4
- External PS2 keyboard
- High resolution touch screen
- More ports: joystick, MPGs, scales

See [h5 folder for hardware files, software and assembly](h5/).

### NanoEls H4
CNC and electronic lead screw controller based on ESP32-S3 that supports up to 4 axes:

- Automatic threads including multi-start
- Multi-pass turning, facing and cones
- Precise movements, soft limits and much more

See [h4 folder for hardware files, software, assembly and usage manual](h4/).

### NanoEls H2
Cheap DIY Electronic Lead Screw (ELS) based on Arduino Nano for metal lathes. No more greasy gear swapping! Control your metal lathe lead screw with a few clicks.

- Set leadscrew pitch for feed or thread
- Soft limits for the carriage
- Multi-start threads in 1 start of the spindle

See [h2 folder for parts list, software, assembly and usage manual](h2/).

### NanoEls H1
The original Arduino-based version that started it all.

See [h1 folder for hardware files, software and assembly](h1/).

---

## Hardware Requirements

It's suggested to use STEPPERONLINE CL57T closed-loop driver with NEMA 23 3NM motor or stronger with 200 step resolution mode (full step) and a 600 PPR optical rotary encoder. [See hardware.md for more info.](hardware.md)

---

## Credits & Acknowledgments

**Original Creator:** Maxim Kachurovskiy - [Original nanoELS Repository](https://github.com/kachurovskiy/nanoels)

This project would not exist without Maxim's original work, design, and dedication to the maker community. All original hardware designs, software architecture, and core functionality remain his intellectual property.

---

## Contributing

- Questions, problems and improvements: please start [a new GitHub Discussion](https://github.com/gueee/nanoELS-unleashed/discussions/new) or a new Issue
- Successful builds: please start a new GitHub Discussion with photos and comments

---

## License

This software and instructions are [provided as is](LICENSE), without warranty of any kind.

---

**Thank you, Maxim, for creating this amazing project and sharing it with the community!**
