# Platform for Channel-Based Encryption of Speech Communication with AES on 2.45 GHz
## Master thesis 2020-2021
Development of a hardware platform with encrypted, half-duplex voice communication using AES and channel-based generated symmetric keys.
- compact PCB design (75mm x 40mm)
- hardware design based around an STM32F415 MCU and an ADF7242 transceiver
- communication range up to 400m in Line-of-Sight and 25m in Non-Line-of-Sight
- 7 hours of continuous communication with up to 500 days of usage in standby
- explointing unique, reciprocal channel characteristics (received signal strength) for key generation to negate eavesdropping
- 128-bit keys updated every 1.6s with a high key entropy and a ultra-low key error rate
- multi-user communication compatible

<p align="center">
    <img src="/Images/Final_product.png">
</p>

Final paper included in "2022 16th European Conference on Antennas and Propagation (EuCAP)".

### Authors:
- Victor Van der Elst
- Ruben Wilssens

### Supervisors:
- ing. Jelle Jocqué
- ing. Joryan Sennesael
- prof. dr. ing. Patrick Van Torre
- prof. dr. ir. Jo Verhaevert
