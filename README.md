# TEKBOT WEST's readme <img src="https://upload.wikimedia.org/wikipedia/commons/a/ab/Flag_of_Panama.svg" alt="Bandera de Panamá" width="30"/>

<p align="center">
  <img src="https://github.com/user-attachments/assets/be75ac88-018e-48ce-b09f-6da08b648245" alt="TEKBOT (1)">
</p>

[![Instagram](https://img.shields.io/badge/Instagram-%23E9805F.svg?style=for-the-badge&logo=Instagram&logoColor=white)](https://www.instagram.com/tekbot_lab?utm_source=ig_web_button_share_sheet&igsh=ZDNlZDc0MzIxNw==)
[![Facebook](https://img.shields.io/badge/YouTube-%23E4445F.svg?style=for-the-badge&logo=Youtube&logoColor=white)](https://www.youtube.com/@TEKBOT_LAB)

This is the official repository for TEKBOT WEST, the team participating in the Panama WRO Regional phase to compete for a spot on the national team. Here you'll find everything related to the development of our robot as we strive towards the national and international finals!


# Repo's folder structure
```
📦 TEKBOT-WEST-WRO-FE
├── 📁 3D_printables # 3D models ready for printing robot components
├── 📁 schemes # Diagrams, schematics, and other electrical plans
├── 📁 src # Source code for robot control and challenge algorithms
├── 📁 team photos # Photos of the team and related activities
├── 📁 vehicle photos # Pictures of the robot at different stages and views
├── 📁 video # Videos of testing, demos, and competition runs
├── 📄 README.md # Main project documentation and overview
```
  
## Meet the team 👨‍💻
fotito
## Luis Hidalgo
### Age: 16
<img src="https://github.com/user-attachments/assets/c2d76d96-894a-4840-8a7d-57509f288950" alt="Imagen 1" width="400">

## Ericka Ceballos
### Age: 18


## Patric Sutherland
### Age: 17
<img src="https://github.com/user-attachments/assets/825efd1b-db29-480d-b972-22ae3139cd18" alt="Imagen 1" width="400">


## Our Coach
## Diego Delgado
### Age: 25
<img src="https://github.com/user-attachments/assets/6424b26b-7c32-4192-8623-f851a1f43a6e8" alt="Imagen 1" width="400">


# Components 🧱
A list of all the electrical and mechanical components on the robot.

| <img src="https://github.com/user-attachments/assets/719a51d8-4b14-402d-a462-ba1e4b071c2c" alt="Alt 1" width="200"/> | <img src="https://github.com/user-attachments/assets/3641b928-34c5-4d97-861c-fa08d40c9faa" alt="Alt 1" width="200"/> | <img src="https://github.com/user-attachments/assets/4adda023-cb88-4a00-9a89-d7be3f75bf26" alt="Alt 1" width="200"/> | 
| :------------: |:-------------:| :------------:|
|[Raspberry Pi 5 - 8GB RAM x1](https://store-usa.arduino.cc/products/arduino-nano?srsltid=AfmBOooU4-IrktQwXymxJgaV7MZPj3cBWDjg6AjQwBmYoQw8es2bz9ex)|[Microsoft LifeCam HD-3000 x1](https://a.co/d/42jYlB6)|[L298N motor driver x1](https://a.co/d/e4jJKCS)|
| <img src="https://github.com/user-attachments/assets/5b0ffc5d-ce02-4620-9849-15fdce566702" width="200"/> | <img src="https://github.com/user-attachments/assets/d4170adc-23b9-446f-bac0-1c50b966e00f" alt="Alt 1" width="200"/> | <img src="https://github.com/user-attachments/assets/b9cfe245-e774-4e4c-aed9-0e2c7445bf3c" alt="Alt 1" width="200"/> |
| [Arduino NANO RP2040 x1](https://a.co/d/9mUTqVe) |[FUNDUINO kit chassis x1](https://a.co/d/fpJSHg1)|[INIU Slim 10,000mAh Power Bank x1](https://a.co/d/1patlqb) |
| <img src="https://github.com/user-attachments/assets/0be65d89-2a79-418d-86e4-e2d6fee2a532" width="200"/> |<img src="https://github.com/user-attachments/assets/76266feb-d49a-4219-aae1-4e0dd4bc8ee0" alt="Alt 1" width="200"/>| <img src="https://github.com/user-attachments/assets/47f81a81-a3fc-4954-9094-194a25310c96" alt="Alt 1" width="200"/> |
|[Geekworm Raspberry Pi Wide Input Voltage Power Management x1](https://www.pololu.com/product/4863)|[MG996R High Torque Metal Gear Servo x1](https://a.co/d/cRVAc0u)|[3.7V Flat Top Lithium Rechargeable Battery x2](https://a.co/d/fZOg5VN)|

---

## ⚙️ Mobility Management

Our robot's mobility system is divided into two main components: **movement** and **steering**.

### Movement
The robot uses a **DC motor** mounted at the rear, connected to a **shared rear axle** via a mechanical linkage. This ensures both rear wheels rotate together, complying with WRO rules. The motor shaft (D-shaft) is securely coupled to the axle for efficient torque transfer.

The motor is controlled by an **L298N motor driver**, which is connected directly to the **Raspberry Pi**. This allows the Pi to manage motor speed and direction based on real-time vision processing.

### Steering
For steering, we use the **MG996R metal gear servo motor**, known for its high torque and precision. The robot employs an **Ackermann steering system**, included in the Funduino kit, which mimics real vehicle steering geometry for smoother and more accurate turns.

The **Arduino Nano RP2040** receives steering commands from the Raspberry Pi and controls the servo motor accordingly.

---

## 🔋 Power Management

Our robot uses a dual power system to efficiently manage energy distribution:

- A **power bank** supplies power to the **Raspberry Pi**, the **Microsoft LifeCam HD-3000 webcam**, and the **DC motor** (via the L298N driver). This ensures stable voltage and current for the SBC (Single Board Computer) and high-power components.
  
- A set of **3.7V Flat Top Lithium Rechargeable Batteries** powers the **Arduino Nano RP2040** and the **MG996R servo motor**. This separation helps prevent voltage drops and ensures reliable operation of the control and actuation systems.

We are planning to replace the power bank with the **Geekworm Raspberry Pi Wide Input Voltage Power Management Module**, which will allow us to:

- Power the Raspberry Pi and camera directly from the 3.7V lithium batteries.
- Provide regulated and protected power delivery to the SBC.
- Reduce weight and improve cable management by eliminating the external power bank.

This upgrade will result in a more compact, efficient, and reliable power system.

---

## 👁️ Sense and Object Detection

Our robot uses **computer vision** to perceive its environment and make autonomous decisions.

### Vision System
We use the **Microsoft LifeCam HD-3000** webcam in combination with **OpenCV (Open Source Computer Vision Library)**, a powerful toolkit for real-time image processing. All image processing is handled on the **Raspberry Pi**, which interprets the camera feed and:

- Sends **steering commands** to the **Arduino Nano RP2040**, which controls the MG996R servo motor.
- Directly controls the **DC motor** through the **L298N motor driver**.

This setup enables the robot to:

- Detect and avoid **black walls** on the track.
- Identify and avoid **obstacles**, including **red and green blocks**.
- Recognize the **magenta parking spot** and perform parking maneuvers.
- Count **orange and blue lines** on the mat to track laps (3 total), ensuring compliance with lap-counting requirements.


# Open Challenge strategy and code explanation
# Obstacle Challenge strategy and code explanation
---

Stay tuned for updates as we continue to improve our robot's performance and capabilities!

# References
https://www.matthewpeterkelly.com/tutorials/pdControl/index.html#:~:text=A%20proportional%2Dderivative%20(PD),car%20at%20some%20desired%20height



