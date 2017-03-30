# ble\_app\_uart\_c example in Nordic nRF5 SDK 13.0.0, with bonded multi-link support

This example is based on the central and peripheral example involving ble\_nus and ble\_nus\_c of Nordic nRF5 SDK 13.0.0. It consists of two projects:

  - ble\_app\_uart\_c, the central project which is modified to support multiple central connection with bonding.
  - ble\_app\_uart, the peripheral project which is modified to support bonding.

### Features

  - Bonding support
  - Mutiple central connection
  - Support both nRF52832 and nRF52840
  - Support Keil4, Keil5, GCC and IAR compilers


You can also:
  - Fall back to single connection by simply changing CENTRAL_LINK_COUNT.
  - Support more concurrent connection by changing both CENTRAL_LINK_COUNT and application start address.
  - Disable bonding by simply changing SEC_PARAM_BOND.
  - ...etc.

### Motivations
> Recently, there has been complaints on the lack of multilink central example
> with bonding support. The ble\_app\_multilink\_central example does not
> demonstrate the use of ble\_* service modules. Even the ble\_app\_uart\_c
> example supports ble\_nus\_c, the client for Nordic UART service, it does
> not support multiple connections. Both examples do not support bonding,
> either. Therefore, this example is created to cope with the need.



### Compilation
This example requires [SDK 13](http://developer.nordicsemi.com/nRF5_SDK/nRF5_SDK_v13.x.x/) to compile.

In the example directory of SDK 13.0.0.

```sh
$ git clone https://github.com/pkchan/SDK13.ble_app_uart_c.bonded_multilink.git
```

Choose your compiler. For example, Keil5

```sh
$ cd SDK13.ble_app_uart_c.bonded_multilink 
$ uv4 -r -j0 ble_app_uart/pca10040/s132/arm5_no_packs/ble_app_uart_pca10040_s132.uvprojx
$ uv4 -r -j0 ble_app_uart_c/pca10056/s140/arm5_no_packs/ble_app_uart_c_pca10056_s140.uvprojx
```

### Others

| Tools | Link |
| ------ | ------ |
| nRF52840 Tools | [Available on Nordic 52840 download page.](http://www.nordicsemi.com/eng/Products/nRF52840#Downloads) |
| nRF52832 Tools | [Available on Nordic 52832 download page.](http://www.nordicsemi.com/eng/Products/Bluetooth-low-energy/nRF52832#Downloads) |



### Development

For any contribution, please kindly fork, change and send pull request.

Please do NOT fork directly from master. Instead, please fork from development or document branches.
  - For documentation, please fork from doc/0000_readme.
  - For modifications on the central project ble\_app\_uart\_c, please fork from dev/0000\_Central.
  - For modifications on the peripheral project ble\_app\_uart, please fork from dev/0001\_Peripheral.

If there are any changes required to SDK files, please:
  - Put all modifications in directory SDK.mod. Create the directory if it does not exist.
  - Make a clean copy of the file from the SDK, commit and then make changes, in order for others to understand your modifications.

License
----

As stated in the header of each c/h file:
 
Copyright (c) 2014 - 2017, Nordic Semiconductor ASA

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
 
2. Redistributions in binary form, except as embedded into a Nordic
   Semiconductor ASA integrated circuit in a product or a software update for
   such product, must reproduce the above copyright notice, this list of
   conditions and the following disclaimer in the documentation and/or other
   materials provided with the distribution.

 3. Neither the name of Nordic Semiconductor ASA nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.
 
 4. This software, with or without modification, must only be used with a
    Nordic Semiconductor ASA integrated circuit.
 
 5. Any software provided in binary form under this license must not be reverse
    engineered, decompiled, modified and/or disassembled.
 
 THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


