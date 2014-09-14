/**
  @page Demonstration
  
  @verbatim
  ******************** (C) COPYRIGHT 2013 STMicroelectronics *******************
  * @file    Demonstration/readme.txt  
  * @author  MCD Application Team
  * @version VX.Y.Z
  * @date    DD-Month-YYYY
  * @brief   Demonstration Description.
  ******************************************************************************
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *   
  ******************************************************************************
   @endverbatim

@par Demonstration Description 

This demonstration consists on using the USER push button B1 connected to PA0 and 
the two leds LED4 and LED3 connected respectively to PC8 and PC9.

In the demonstration  the LED3 is initially blinking each 200ms.
  - For the first pressing on the USER button, the speed of the led blinking is incremented. 
    The LED3 blinks each 100ms.
  - For the second pressing on the USER button, LED3 stops blinking and the status is off.
  - A third pressing on the USER button causes the initial behavior of the LED3.

Each time the USER button is pressed, the LED4 is turned on for 1s.


@par Directory contents 

  - Demonstration/stm32f0xx_conf.h    Library Configuration file
  - Demonstration/stm32f0xx_it.c      Interrupt handlers
  - Demonstration/stm32f0xx_it.h      Interrupt handlers header file
  - Demonstration/main.c              Main program
  - Demonstration/main.h              Header file for Main program
  - Demonstration/system_stm32f0xx.c  STM32F0xx system source file
  
@note The "system_stm32f0xx.c" file contains the system clock configuration for
      STM32F0xx devices, and is customized for use with STM32F0308-DISCO Kit. 
      The STM32F0xx is configured to run at 48 MHz, following the three  
      configuration below:
        + PLL_SOURCE_HSI
           - HSI (~8 MHz) used to clock the PLL, and the PLL is used as system 
             clock source.  
        + PLL_SOURCE_HSE          
           - HSE (8 MHz) used to clock the PLL, and the PLL is used as system
             clock source.
           - The HSE crystal is not provided with the Discovery Kit, some 
             hardware modification are needed in manner to connect this crystal.
             For more details, refer to section "4.7 OSC clock" in "STM32F030 Value Line
             discovery kit User manual (UM1658)"
        + PLL_SOURCE_HSE_BYPASS   
           - HSE bypassed with an external clock (fixed at 8 MHz, coming from 
             ST-Link circuit) used to clock the PLL, and the PLL is used as 
             system clock source.
           - Some  hardware modification are needed in manner to bypass the HSE 
             with clock coming from the ST-Link circuit.
             For more details, refer to section "4.7 OSC clock" in "STM32F030 Value Line
             discovery kit User manual (UM1658)"
      User can select one of the three configuration in system_stm32f0xx.c file
      (default configuration is PLL_SOURCE_HSI).
         

@par Hardware and Software environment

  - This example runs on STM32F030x Devices.
  
  - This example has been tested with STMicroelectronics STM32F0308-DISCO (MB1134) 
    RevA and can be easily tailored to any other supported device and development board.
  
  - STM32F0308-DISCO Set-up 
    - LED4 and LED3 are connected respectively to PC.8 and PC.9 pins.
    - Use USER push button connected to PA.0 pin.


@par How to use it ? 

In order to make the program work, you must do the following :

 + EWARM
    - Open the IO_Toggle.eww workspace 
    - Rebuild all files: Project->Rebuild all
    - Load project image: Project->Debug
    - Run program: Debug->Go(F5)

 + MDK-ARM
    - Open the IO_Toggle.uvproj project
    - Rebuild all files: Project->Rebuild all target files
    - Load project image: Debug->Start/Stop Debug Session
    - Run program: Debug->Run (F5)    

 + TrueSTUDIO for ARM
    - Open the TrueSTUDIO for ARM toolchain.
    - Click on File->Switch Workspace->Other and browse to TrueSTUDIO workspace 
      directory.
    - Click on File->Import, select General->'Existing Projects into Workspace' 
      and then click "Next". 
    - Browse to the TrueSTUDIO workspace directory and select the project "IO_Toggle" 
    - Rebuild all project files: Select the project in the "Project explorer" 
      window then click on Project->build project menu.
    - Run program: Select the project in the "Project explorer" window then click 
      Run->Debug (F11)

 * <h3><center>&copy; COPYRIGHT STMicroelectronics</center></h3>
 */
